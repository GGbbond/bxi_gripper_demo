#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <linux/can.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "bxi_pci_drv.h"

#define SERVER_PORT 9999
#define CAN_BUS_COUNT 7
#define PI_F 3.14159265358979323846f
#define DEG_TO_RAD(x) ((x) * PI_F / 180.0f)
#define RAD_TO_DEG(x) ((x) * 180.0f / PI_F)
#define P_MIN (-12.5f)
#define P_MAX 12.5f
#define V_MIN (-45.0f)
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN (-40.0f)
#define T_MAX 40.0f
#define TEMP_MIN (-30.0f)
#define TEMP_MAX 150.0f
#define BOOT_DELAY_MS 1500LL
#define CONTROL_PERIOD_US 2000
#define TELEMETRY_PERIOD_MS 20LL
#define MOVE_PEAK_FACTOR 1.875f
#define ZERO_MAX_ATTEMPTS 3
#define ZERO_FRAME_REPEAT 3
#define ZERO_FEEDBACK_TIMEOUT_MS 1000LL
#define ZERO_POSITION_TOLERANCE_DEG 2.0f
/* Critically damped slider follower: ~0.3 s to settle, no per-target velocity reset. */
#define STREAM_FOLLOW_RATE 20.0f
#define TORQUE_LIMIT_RELEASE_RATIO 0.90f
#define TORQUE_FILTER_ALPHA 0.10f
#define TORQUE_FEEDBACK_TIMEOUT_MS 100LL
#define TORQUE_BACKOFF_START_RATIO 1.05f
#define TORQUE_BACKOFF_MIN_DEG_S 0.5f
#define TORQUE_BACKOFF_GAIN_DEG_S_PER_NM 5.0f
#define TORQUE_BACKOFF_MAX_DEG_S 10.0f
#define BACKEND_REVISION "torque-limit-live-v5"

typedef struct {
    float position;
    float velocity;
    float torque;
    float mos_temp;
    float rotor_temp;
    unsigned int sequence;
    long long updated_ms;
    int valid;
} motor_feedback;

typedef enum {
    MOTION_IDLE = 0,
    MOTION_MOVE,
    MOTION_STREAM
} motion_kind;

typedef struct {
    motion_kind kind;
    float start_position;
    float start_velocity;
    float target_position;
    float peak_velocity;
    float acceleration;
    float duration;
    long long started_ms;
} motion_state;

static atomic_int g_running = 1;
static pthread_mutex_t g_state_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_send_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_client_fd = -1;
static int g_server_fd = -1;
static unsigned int g_bus = 2;
static unsigned int g_motor_id = 1;
static int g_power_on = 0;
static int g_enable_requested = 0;
static int g_mode_entered = 0;
static int g_control_ready = 0;
static long long g_power_started_ms = 0;
static float g_command_position = 0.0f;
static float g_command_velocity = 0.0f;
static float g_command_kp = 0.0f;
static float g_command_kd = 0.0f;
static float g_target_kp = 300.0f;
static float g_target_kd = 5.0f;
static float g_position_min_deg = -360.0f;
static float g_position_max_deg = 360.0f;
static int g_torque_limit_enabled = 0;
static float g_torque_limit_nm = 1.0f;
static float g_filtered_abs_torque = 0.0f;
static unsigned int g_torque_filter_sequence = 0;
static int g_torque_limited = 0;
static motor_feedback g_feedback;
static motion_state g_motion;
static pthread_t g_control_thread;

static long long monotonic_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (long long)now.tv_sec * 1000LL + now.tv_nsec / 1000000LL;
}

static float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void reset_torque_supervisor_locked(void)
{
    g_torque_limited = 0;
    g_filtered_abs_torque = 0.0f;
    g_torque_filter_sequence = 0;
}

static uint32_t float_to_uint(float value, float low, float high, int bits)
{
    const float span = high - low;
    const uint32_t max_value = (1U << bits) - 1U;
    value = clampf(value, low, high);
    return (uint32_t)((value - low) * (float)max_value / span);
}

static float uint_to_float(uint32_t value, float low, float high, int bits)
{
    const uint32_t max_value = (1U << bits) - 1U;
    return (float)value * (high - low) / (float)max_value + low;
}

static void pack_control(uint8_t data[8], float p, float v, float kp, float kd)
{
    const uint32_t pi = float_to_uint(p, P_MIN, P_MAX, 16);
    const uint32_t vi = float_to_uint(v, V_MIN, V_MAX, 12);
    const uint32_t kpi = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    const uint32_t kdi = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    const uint32_t ti = float_to_uint(0.0f, T_MIN, T_MAX, 12);

    data[0] = (uint8_t)(pi >> 8);
    data[1] = (uint8_t)pi;
    data[2] = (uint8_t)(vi >> 4);
    data[3] = (uint8_t)(((vi & 0xFU) << 4) | (kpi >> 8));
    data[4] = (uint8_t)kpi;
    data[5] = (uint8_t)(kdi >> 4);
    data[6] = (uint8_t)(((kdi & 0xFU) << 4) | (ti >> 8));
    data[7] = (uint8_t)ti;
}

static int send_packet(unsigned int bus, unsigned int motor_id,
                       const uint8_t data[8])
{
    canfd_packet packet;
    memset(&packet, 0, sizeof(packet));
    packet.bus = bus;
    packet.frame.can_id = motor_id;
    packet.frame.len = 8;
    packet.frame.flags = CANFD_BRS | CANFD_FDF;
    memcpy(packet.frame.data, data, 8);
    return canfd_send_packet(&packet, 1);
}

static int send_mode(unsigned int bus, unsigned int motor_id, uint8_t command)
{
    uint8_t data[8];
    memset(data, 0xFF, sizeof(data));
    data[7] = command;
    return send_packet(bus, motor_id, data);
}

static void send_line(const char *format, ...)
{
    char message[512];
    va_list args;
    int fd;

    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    pthread_mutex_lock(&g_send_lock);
    fd = g_client_fd;
    if (fd >= 0) {
        (void)send(fd, message, strlen(message), MSG_NOSIGNAL);
    }
    pthread_mutex_unlock(&g_send_lock);
}

static void log_line(const char *format, ...)
{
    char body[400];
    va_list args;
    va_start(args, format);
    vsnprintf(body, sizeof(body), format, args);
    va_end(args);
    fprintf(stdout, "%s\n", body);
    fflush(stdout);
    send_line("LOG %s\n", body);
}

static void power_off_locked(void)
{
    unsigned int bus = g_bus;
    unsigned int motor_id = g_motor_id;
    int entered = g_mode_entered;

    g_motion.kind = MOTION_IDLE;
    g_enable_requested = 0;
    g_control_ready = 0;
    g_mode_entered = 0;
    g_command_velocity = 0.0f;
    g_command_kp = 0.0f;
    g_command_kd = 0.0f;
    reset_torque_supervisor_locked();
    pthread_mutex_unlock(&g_state_lock);
    if (entered) {
        send_mode(bus, motor_id, 0xFD);
        usleep(50000);
    }
    motor_pwr_set(0);
    pthread_mutex_lock(&g_state_lock);
    g_power_on = 0;
    g_power_started_ms = 0;
}

static int pci_rx_callback(void *arg, canfd_packet *packet)
{
    (void)arg;
    if (!packet || packet->frame.len < 8) return 0;

    const unsigned int raw_id = packet->frame.can_id & CAN_EFF_MASK;
    const unsigned int feedback_id = packet->frame.data[0] & 0x0FU;
    int became_ready = 0;
    float p, v, t, mos, rotor;

    pthread_mutex_lock(&g_state_lock);
    if (packet->bus != g_bus || feedback_id != g_motor_id ||
        raw_id != (feedback_id | 0x10U)) {
        pthread_mutex_unlock(&g_state_lock);
        return 0;
    }

    const uint8_t *data = packet->frame.data;
    const uint32_t pi = ((uint32_t)data[1] << 8) | data[2];
    const uint32_t vi = ((uint32_t)data[3] << 4) | (data[4] >> 4);
    const uint32_t ti = ((uint32_t)(data[4] & 0x0F) << 8) | data[5];
    p = uint_to_float(pi, P_MIN, P_MAX, 16);
    v = uint_to_float(vi, V_MIN, V_MAX, 12);
    t = uint_to_float(ti, T_MIN, T_MAX, 12);
    mos = uint_to_float(data[6], TEMP_MIN, TEMP_MAX, 8);
    rotor = uint_to_float(data[7], TEMP_MIN, TEMP_MAX, 8);

    g_feedback.position = p;
    g_feedback.velocity = v;
    g_feedback.torque = t;
    g_feedback.mos_temp = mos;
    g_feedback.rotor_temp = rotor;
    g_feedback.sequence++;
    g_feedback.updated_ms = monotonic_ms();
    g_feedback.valid = 1;
    if (g_mode_entered && !g_control_ready) {
        g_command_position = p;
        g_command_velocity = 0.0f;
        g_command_kp = 0.0f;
        g_command_kd = g_target_kd;
        g_control_ready = 1;
        became_ready = 1;
    }
    pthread_mutex_unlock(&g_state_lock);

    if (became_ready) {
        send_line("MOTOR_POWER_READY 1\n");
        log_line("Motor feedback ready on CAN%u, ID %u", packet->bus, feedback_id);
    }
    return 0;
}

static void update_move_locked(long long now_ms, int *completed)
{
    float elapsed = (float)(now_ms - g_motion.started_ms) / 1000.0f;
    if (elapsed >= g_motion.duration) {
        g_command_position = g_motion.target_position;
        g_command_velocity = 0.0f;
        g_motion.kind = MOTION_IDLE;
        *completed = 1;
        return;
    }

    const float u = clampf(elapsed / g_motion.duration, 0.0f, 1.0f);
    const float u2 = u * u;
    const float u3 = u2 * u;
    const float u4 = u3 * u;
    const float u5 = u4 * u;
    const float delta = g_motion.target_position - g_motion.start_position;
    const float velocity_term = g_motion.start_velocity * g_motion.duration;
    const float c3 = 10.0f * delta - 6.0f * velocity_term;
    const float c4 = -15.0f * delta + 8.0f * velocity_term;
    const float c5 = 6.0f * delta - 3.0f * velocity_term;
    g_command_position = g_motion.start_position + velocity_term * u +
                         c3 * u3 + c4 * u4 + c5 * u5;
    g_command_velocity = (velocity_term + 3.0f * c3 * u2 +
                          4.0f * c4 * u3 + 5.0f * c5 * u4) /
                         g_motion.duration;
}

static void update_stream_locked(float dt)
{
    if (!isfinite(dt) || dt <= 0.0f) return;
    const float error = g_motion.target_position - g_command_position;
    const float old_velocity = g_command_velocity;
    const float rate = STREAM_FOLLOW_RATE;
    /* Exact velocity of a critically damped second-order follower over dt.
     * Close targets therefore reduce speed continuously rather than being
     * chased at peak speed and snapped to rest when crossed. */
    const float follow_velocity =
        (old_velocity + (rate * rate * error - rate * old_velocity) * dt) *
        expf(-rate * dt);
    const float wanted_velocity = clampf(
        follow_velocity, -g_motion.peak_velocity, g_motion.peak_velocity);
    const float max_change = g_motion.acceleration * dt;
    g_command_velocity = old_velocity + clampf(
        wanted_velocity - old_velocity, -max_change, max_change);
    /* Keep p_des and v_des consistent, including speed changes and reversals.
     * The UI range constrains requested targets, not measured/start positions:
     * hard-clamping the latter creates an instantaneous position command step. */
    g_command_position += 0.5f * (old_velocity + g_command_velocity) * dt;
}

static void update_torque_filter_locked(void)
{
    if (!g_feedback.valid || g_feedback.sequence == g_torque_filter_sequence) return;
    const float sample = fabsf(g_feedback.torque);
    if (g_torque_filter_sequence == 0) {
        g_filtered_abs_torque = sample;
    } else {
        g_filtered_abs_torque +=
            (sample - g_filtered_abs_torque) * TORQUE_FILTER_ALPHA;
    }
    g_torque_filter_sequence = g_feedback.sequence;
}

static void restart_move_from_reference_locked(long long now_ms)
{
    if (g_motion.kind != MOTION_MOVE) return;
    const float distance = fabsf(g_motion.target_position - g_command_position);
    g_motion.start_position = g_command_position;
    g_motion.start_velocity = 0.0f;
    g_motion.duration = fmaxf(
        0.05f, distance * MOVE_PEAK_FACTOR / g_motion.peak_velocity);
    g_motion.started_ms = now_ms;
}

static int torque_limit_blocks_closing_locked(long long now_ms)
{
    if (!g_torque_limit_enabled || g_motion.kind == MOTION_IDLE) {
        g_torque_limited = 0;
        return 0;
    }

    /* Decreasing angle closes this gripper. Never block an opening command:
     * the operator must be able to release the load while torque is high. */
    const int closing =
        g_motion.target_position < g_command_position - DEG_TO_RAD(0.001f);
    if (!closing) {
        g_torque_limited = 0;
        return 0;
    }

    const int feedback_stale =
        !g_feedback.valid || now_ms - g_feedback.updated_ms > TORQUE_FEEDBACK_TIMEOUT_MS;
    const float raw_abs_torque = fabsf(g_feedback.torque);
    if (feedback_stale || (!g_torque_limited &&
            (raw_abs_torque >= g_torque_limit_nm ||
             g_filtered_abs_torque >= g_torque_limit_nm))) {
        g_torque_limited = 1;
    } else if (g_torque_limited &&
               raw_abs_torque <= g_torque_limit_nm * TORQUE_LIMIT_RELEASE_RATIO &&
               g_filtered_abs_torque <= g_torque_limit_nm * TORQUE_LIMIT_RELEASE_RATIO) {
        g_torque_limited = 0;
        restart_move_from_reference_locked(now_ms);
    }
    return g_torque_limited;
}

static void update_torque_limited_reference_locked(float dt)
{
    const float measured = fmaxf(
        fabsf(g_feedback.torque), g_filtered_abs_torque);
    const float excess = measured - g_torque_limit_nm * TORQUE_BACKOFF_START_RATIO;
    g_command_velocity = 0.0f;
    if (excess <= 0.0f) return;

    /* Back off slowly in the opening direction. Reducing position error also
     * reduces the motor's internally generated MIT PD torque. */
    const float backoff_deg_s = clampf(
        TORQUE_BACKOFF_MIN_DEG_S + excess * TORQUE_BACKOFF_GAIN_DEG_S_PER_NM,
        TORQUE_BACKOFF_MIN_DEG_S, TORQUE_BACKOFF_MAX_DEG_S);
    const float upper = DEG_TO_RAD(g_position_max_deg);
    const float step = DEG_TO_RAD(backoff_deg_s) * dt;
    if (g_command_position < upper) {
        g_command_position = fminf(upper, g_command_position + step);
        g_command_velocity = DEG_TO_RAD(backoff_deg_s);
    }
}

static void *control_loop(void *arg)
{
    (void)arg;
    long long last_ms = monotonic_ms();
    long long last_telemetry = 0;

    while (atomic_load(&g_running)) {
        uint8_t data[8];
        unsigned int bus, motor_id;
        int send_control = 0;
        int send_enter = 0;
        int completed = 0;
        motor_feedback feedback;
        float trace_position, trace_velocity, trace_target;
        float trace_filtered_torque, trace_torque_limit;
        int trace_power, trace_ready;
        int trace_torque_limit_enabled, trace_torque_limited;
        long long trace_time;
        long long now = monotonic_ms();
        float dt = clampf((float)(now - last_ms) / 1000.0f, 0.001f, 0.02f);
        last_ms = now;

        pthread_mutex_lock(&g_state_lock);
        bus = g_bus;
        motor_id = g_motor_id;
        if (g_power_on && g_enable_requested && !g_mode_entered &&
            now - g_power_started_ms >= BOOT_DELAY_MS) {
            g_mode_entered = 1;
            g_control_ready = 0;
            g_feedback.valid = 0;
            send_enter = 1;
        }
        if (g_power_on && g_mode_entered && g_control_ready) {
            int torque_blocked = 0;
            update_torque_filter_locked();
            if (g_motion.kind == MOTION_MOVE) {
                torque_blocked = torque_limit_blocks_closing_locked(now);
                if (torque_blocked) update_torque_limited_reference_locked(dt);
                else update_move_locked(now, &completed);
            } else if (g_motion.kind == MOTION_STREAM) {
                torque_blocked = torque_limit_blocks_closing_locked(now);
                if (torque_blocked) update_torque_limited_reference_locked(dt);
                else update_stream_locked(dt);
            } else {
                g_torque_limited = 0;
            }
            const float blend = 1.0f - expf(-dt / 0.02f);
            if (!torque_blocked) {
                g_command_kp += (g_target_kp - g_command_kp) * blend;
            }
            g_command_kd += (g_target_kd - g_command_kd) * blend;
            pack_control(data, g_command_position, g_command_velocity,
                         g_command_kp, g_command_kd);
            send_control = 1;
        }
        feedback = g_feedback;
        trace_position = g_command_position;
        trace_velocity = g_command_velocity;
        trace_target = g_motion.kind == MOTION_IDLE
            ? g_command_position : g_motion.target_position;
        trace_power = g_power_on;
        trace_ready = g_control_ready;
        trace_time = monotonic_ms();
        trace_filtered_torque = g_filtered_abs_torque;
        trace_torque_limit = g_torque_limit_nm;
        trace_torque_limit_enabled = g_torque_limit_enabled;
        trace_torque_limited = g_torque_limited;
        pthread_mutex_unlock(&g_state_lock);

        if (send_enter) {
            send_mode(bus, motor_id, 0xFC);
            log_line("MIT mode enabled on CAN%u, ID %u", bus, motor_id);
        }
        if (send_control) send_packet(bus, motor_id, data);
        if (completed) send_line("CLAW_MOVE_COMPLETE\n");
        if (now - last_telemetry >= TELEMETRY_PERIOD_MS) {
            last_telemetry = now;
            if (feedback.valid) {
                send_line("POS %.3f\nVEL %.3f\nTORQUE %.3f\nTEMP_MOS %.2f\nTEMP_ROTOR %.2f\n",
                          RAD_TO_DEG(feedback.position), RAD_TO_DEG(feedback.velocity),
                          feedback.torque, feedback.mos_temp, feedback.rotor_temp);
            }
            /* Atomic snapshot for comparing reference commands and feedback.
             * Invalid feedback is explicitly marked by age=-1. */
            send_line("TRACE %lld %.6f %.6f %.6f %.6f %.6f %.6f %u %lld %d %d "
                      "%d %.6f %.6f %d\n",
                      trace_time, RAD_TO_DEG(trace_position), RAD_TO_DEG(trace_velocity),
                      RAD_TO_DEG(trace_target), RAD_TO_DEG(feedback.position),
                      RAD_TO_DEG(feedback.velocity), feedback.torque, feedback.sequence,
                      feedback.valid ? trace_time - feedback.updated_ms : -1LL,
                      trace_power, trace_ready, trace_torque_limit_enabled,
                      trace_torque_limit, trace_filtered_torque, trace_torque_limited);
            send_line("TORQUE_LIMIT_STATUS %d %.6f %.6f %d\n",
                      trace_torque_limit_enabled, trace_filtered_torque,
                      trace_torque_limit, trace_torque_limited);
        }
        usleep(CONTROL_PERIOD_US);
    }
    return NULL;
}

static int start_motion(float position_deg, float speed_deg_s,
                        float kp, float kd, int stream)
{
    int result = 0;
    pthread_mutex_lock(&g_state_lock);
    if (!(g_power_on && g_mode_entered && g_control_ready)) {
        result = -1;
    } else if (position_deg < g_position_min_deg || position_deg > g_position_max_deg) {
        result = -2;
    } else {
        const float target = DEG_TO_RAD(clampf(position_deg, -360.0f, 360.0f));
        const float speed = DEG_TO_RAD(fabsf(speed_deg_s));
        g_target_kp = clampf(kp, KP_MIN, KP_MAX);
        g_target_kd = clampf(kd, KD_MIN, KD_MAX);
        if (stream) {
            g_motion.kind = MOTION_STREAM;
            g_motion.target_position = target;
            g_motion.peak_velocity = speed;
            g_motion.acceleration = fmaxf(DEG_TO_RAD(360.0f), speed / 0.05f);
        } else {
            const float distance = fabsf(target - g_command_position);
            g_motion.kind = MOTION_MOVE;
            g_motion.start_position = g_command_position;
            g_motion.start_velocity = g_command_velocity;
            g_motion.target_position = target;
            g_motion.peak_velocity = speed;
            g_motion.duration = fmaxf(0.05f, distance * MOVE_PEAK_FACTOR / speed);
            g_motion.started_ms = monotonic_ms();
        }
    }
    pthread_mutex_unlock(&g_state_lock);
    return result;
}

static void stop_and_hold(void)
{
    pthread_mutex_lock(&g_state_lock);
    g_motion.kind = MOTION_IDLE;
    g_command_position = g_feedback.valid ? g_feedback.position : g_command_position;
    g_command_velocity = 0.0f;
    g_torque_limited = 0;
    pthread_mutex_unlock(&g_state_lock);
}

static void handle_zero(int fd)
{
    unsigned int bus, motor_id;
    int allowed;
    int feedback_received = 0;
    int send_failed = 0;
    uint8_t zero_data[8];
    (void)fd;
    memset(zero_data, 0xFF, sizeof(zero_data));
    zero_data[7] = 0xFE;

    pthread_mutex_lock(&g_state_lock);
    allowed = g_power_on && g_mode_entered;
    bus = g_bus;
    motor_id = g_motor_id;
    if (allowed) {
        g_motion.kind = MOTION_IDLE;
        g_enable_requested = 0;
        g_control_ready = 0;
        g_mode_entered = 0;
        g_command_velocity = 0.0f;
        reset_torque_supervisor_locked();
    }
    pthread_mutex_unlock(&g_state_lock);
    if (!allowed) {
        send_line("ERROR claw zero requires motor power\n");
        return;
    }

    send_line("CLAW_ZERO_STARTED\n");
    for (int attempt = 1; attempt <= ZERO_MAX_ATTEMPTS; attempt++) {
        unsigned int old_sequence;
        int verified = 0;

        if (send_mode(bus, motor_id, 0xFD) < 0) {
            send_failed = 1;
            break;
        }
        usleep(80000);
        for (int repeat = 0; repeat < ZERO_FRAME_REPEAT; repeat++) {
            if (send_packet(bus, motor_id, zero_data) < 0) {
                send_failed = 1;
                break;
            }
            usleep(20000);
        }
        if (send_failed) break;
        usleep(50000);
        pthread_mutex_lock(&g_state_lock);
        old_sequence = g_feedback.sequence;
        pthread_mutex_unlock(&g_state_lock);
        if (send_mode(bus, motor_id, 0xFC) < 0) {
            send_failed = 1;
            break;
        }

        const long long deadline = monotonic_ms() + ZERO_FEEDBACK_TIMEOUT_MS;
        while (monotonic_ms() < deadline) {
            unsigned int sequence;
            float position;
            pthread_mutex_lock(&g_state_lock);
            sequence = g_feedback.sequence;
            position = g_feedback.position;
            pthread_mutex_unlock(&g_state_lock);
            if (sequence != old_sequence) {
                feedback_received = 1;
                old_sequence = sequence;
                if (fabsf(position) <= DEG_TO_RAD(ZERO_POSITION_TOLERANCE_DEG)) {
                    verified = 1;
                    break;
                }
            }
            usleep(5000);
        }

        if (verified) {
            pthread_mutex_lock(&g_state_lock);
            g_enable_requested = 1;
            g_mode_entered = 1;
            g_control_ready = 1;
            g_command_position = 0.0f;
            g_command_velocity = 0.0f;
            g_command_kp = 0.0f;
            g_command_kd = g_target_kd;
            pthread_mutex_unlock(&g_state_lock);
            send_line("CLAW_ZERO_COMPLETE\n");
            log_line("Motor zero completed on CAN%u, ID %u", bus, motor_id);
            return;
        }

        send_mode(bus, motor_id, 0xFD);
        if (attempt < ZERO_MAX_ATTEMPTS) {
            send_line("CLAW_ZERO_RETRY %d\n", attempt + 1);
            log_line("Zero verification not stable; retrying (%d/%d)",
                     attempt + 1, ZERO_MAX_ATTEMPTS);
            usleep(100000);
        }
    }

    pthread_mutex_lock(&g_state_lock);
    g_enable_requested = 0;
    g_mode_entered = 0;
    g_control_ready = 0;
    pthread_mutex_unlock(&g_state_lock);
    if (send_failed) {
        send_line("ERROR claw zero command failed\n");
    } else if (feedback_received) {
        send_line("ERROR claw zero verification failed\n");
    } else {
        send_line("ERROR claw zero feedback timeout\n");
    }
}

static void handle_command(int fd, const char *command)
{
    unsigned int value;
    float position, speed, kp, kd;

    if (sscanf(command, "SET_CLAW_CAN %u", &value) == 1) {
        pthread_mutex_lock(&g_state_lock);
        if (value >= CAN_BUS_COUNT || g_power_on) {
            pthread_mutex_unlock(&g_state_lock);
            send_line("ERROR power off before changing CAN\n");
            return;
        }
        g_bus = value;
        g_feedback.valid = 0;
        reset_torque_supervisor_locked();
        pthread_mutex_unlock(&g_state_lock);
        send_line("CLAW_CAN_SET %u\n", value);
        return;
    }
    if (sscanf(command, "SET_MOTOR_ID %u", &value) == 1) {
        pthread_mutex_lock(&g_state_lock);
        if (value > 8 || g_power_on) {
            pthread_mutex_unlock(&g_state_lock);
            send_line("ERROR power off before changing motor ID\n");
            return;
        }
        g_motor_id = value;
        g_feedback.valid = 0;
        reset_torque_supervisor_locked();
        pthread_mutex_unlock(&g_state_lock);
        send_line("MOTOR_ID_SET %u\n", value);
        return;
    }
    if (sscanf(command, "SET_POSITION_LIMITS %f %f", &position, &speed) == 2) {
        if (!isfinite(position) || !isfinite(speed) || position < -360.0f ||
            speed > 360.0f || position > 0.0f || speed < 0.0f || position >= speed) {
            send_line("ERROR invalid position limits\n");
            return;
        }
        pthread_mutex_lock(&g_state_lock);
        if (g_power_on) {
            pthread_mutex_unlock(&g_state_lock);
            send_line("ERROR power off before changing position limits\n");
            return;
        }
        g_position_min_deg = position;
        g_position_max_deg = speed;
        pthread_mutex_unlock(&g_state_lock);
        send_line("POSITION_LIMITS_SET %.2f %.2f\n", position, speed);
        return;
    }
    if (sscanf(command, "SET_TORQUE_LIMIT %u %f", &value, &position) == 2) {
        if (value > 1 || !isfinite(position) || position < 0.05f ||
            position > T_MAX) {
            send_line("ERROR invalid torque limit\n");
            return;
        }
        pthread_mutex_lock(&g_state_lock);
        g_torque_limit_enabled = (int)value;
        g_torque_limit_nm = position;
        reset_torque_supervisor_locked();
        pthread_mutex_unlock(&g_state_lock);
        send_line("TORQUE_LIMIT_SET %u %.2f\n", value, position);
        return;
    }
    if (strcmp(command, "MOTOR_POWER_ON") == 0) {
        pthread_mutex_lock(&g_state_lock);
        if (!g_power_on) {
            pthread_mutex_unlock(&g_state_lock);
            if (motor_pwr_set(1) != 0) {
                send_line("ERROR motor power on failed\n");
                return;
            }
            pthread_mutex_lock(&g_state_lock);
            g_power_on = 1;
            g_power_started_ms = monotonic_ms();
            g_feedback.valid = 0;
            reset_torque_supervisor_locked();
        }
        g_enable_requested = 1;
        pthread_mutex_unlock(&g_state_lock);
        send_line("MOTOR_POWERING_ON\n");
        log_line("Motor power on; waiting for controller boot");
        return;
    }
    if (strcmp(command, "MOTOR_POWER_OFF") == 0) {
        pthread_mutex_lock(&g_state_lock);
        if (g_power_on) power_off_locked();
        pthread_mutex_unlock(&g_state_lock);
        send_line("MOTOR_POWER_OFF_COMPLETE\n");
        log_line("Motor power off");
        return;
    }
    if (strcmp(command, "MOTOR_POWER_STATUS") == 0) {
        pthread_mutex_lock(&g_state_lock);
        const int ready = g_power_on && g_mode_entered && g_control_ready;
        pthread_mutex_unlock(&g_state_lock);
        send_line("MOTOR_POWER_READY %d\n", ready);
        return;
    }
    if (sscanf(command, "CLAW_MOVE %f %f %f %f", &position, &speed, &kp, &kd) == 4 ||
        sscanf(command, "CLAW_STREAM %f %f %f %f", &position, &speed, &kp, &kd) == 4) {
        const int stream = strncmp(command, "CLAW_STREAM", 11) == 0;
        if (!isfinite(position) || !isfinite(speed) || !isfinite(kp) || !isfinite(kd) ||
            position < -360.0f || position > 360.0f || speed <= 0.0f) {
            send_line("ERROR invalid claw move\n");
        } else {
            const int motion_result = start_motion(position, speed, kp, kd, stream);
            if (motion_result == -1) {
                send_line("ERROR motor is not ready\n");
            } else if (motion_result == -2) {
                send_line("ERROR position outside configured limits\n");
            } else if (!stream) {
                send_line("CLAW_MOVE_STARTED\n");
            }
        }
        return;
    }
    if (strcmp(command, "CLAW_STOP") == 0) {
        stop_and_hold();
        send_line("CLAW_HOLDING\n");
        return;
    }
    if (sscanf(command, "SET_GAINS %f %f", &kp, &kd) == 2) {
        if (!isfinite(kp) || !isfinite(kd) || kp < KP_MIN || kp > KP_MAX ||
            kd < KD_MIN || kd > KD_MAX) {
            send_line("ERROR invalid gains\n");
            return;
        }
        pthread_mutex_lock(&g_state_lock);
        g_target_kp = kp;
        g_target_kd = kd;
        pthread_mutex_unlock(&g_state_lock);
        send_line("GAINS_SET %.2f %.2f\n", kp, kd);
        return;
    }
    if (strcmp(command, "CLAW_DISABLE") == 0) {
        unsigned int bus, motor_id;
        pthread_mutex_lock(&g_state_lock);
        bus = g_bus;
        motor_id = g_motor_id;
        g_motion.kind = MOTION_IDLE;
        g_enable_requested = 0;
        g_control_ready = 0;
        g_mode_entered = 0;
        reset_torque_supervisor_locked();
        pthread_mutex_unlock(&g_state_lock);
        send_mode(bus, motor_id, 0xFD);
        send_line("CLAW_DISABLED\n");
        return;
    }
    if (strcmp(command, "CLAW_ENABLE") == 0) {
        pthread_mutex_lock(&g_state_lock);
        if (!g_power_on) {
            pthread_mutex_unlock(&g_state_lock);
            send_line("ERROR claw enable requires motor power\n");
            return;
        }
        g_motion.kind = MOTION_IDLE;
        g_command_velocity = 0.0f;
        g_control_ready = 0;
        g_mode_entered = 0;
        g_enable_requested = 1;
        reset_torque_supervisor_locked();
        pthread_mutex_unlock(&g_state_lock);
        send_line("CLAW_ENABLING\n");
        log_line("Re-enabling claw motor without cycling hardware power");
        return;
    }
    if (strcmp(command, "CLAW_ZERO") == 0) {
        handle_zero(fd);
        return;
    }
    if (strcmp(command, "PING") == 0) {
        send_line("PONG\n");
        return;
    }
    if (strcmp(command, "SHUTDOWN") == 0) {
        send_line("SHUTTING_DOWN\n");
        atomic_store(&g_running, 0);
        shutdown(fd, SHUT_RDWR);
        if (g_server_fd >= 0) shutdown(g_server_fd, SHUT_RDWR);
        return;
    }
    send_line("ERROR unknown command\n");
}

static void reset_client_session(void)
{
    pthread_mutex_lock(&g_state_lock);
    if (g_power_on) power_off_locked();
    pthread_mutex_unlock(&g_state_lock);
}

static void signal_handler(int signal_number)
{
    (void)signal_number;
    atomic_store(&g_running, 0);
    if (g_client_fd >= 0) shutdown(g_client_fd, SHUT_RDWR);
    if (g_server_fd >= 0) shutdown(g_server_fd, SHUT_RDWR);
}

int main(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = signal_handler;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    signal(SIGPIPE, SIG_IGN);

    if (bxi_pci_init(pci_rx_callback, NULL, -1) != 0) {
        fprintf(stderr, "BXI PCI initialization failed\n");
        return 1;
    }
    if (pthread_create(&g_control_thread, NULL, control_loop, NULL) != 0) {
        fprintf(stderr, "Control thread creation failed\n");
        bxi_pci_exit();
        return 1;
    }

    g_server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_server_fd < 0) {
        perror("socket");
        atomic_store(&g_running, 0);
        pthread_join(g_control_thread, NULL);
        bxi_pci_exit();
        return 1;
    }
    int option = 1;
    setsockopt(g_server_fd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(SERVER_PORT);
    if (bind(g_server_fd, (struct sockaddr *)&address, sizeof(address)) != 0 ||
        listen(g_server_fd, 1) != 0) {
        perror("bind/listen");
        atomic_store(&g_running, 0);
    } else {
        log_line("BXI gripper backend listening on 127.0.0.1:%d", SERVER_PORT);
    }

    while (atomic_load(&g_running)) {
        struct sockaddr_in client_address;
        socklen_t client_length = sizeof(client_address);
        int fd = accept(g_server_fd, (struct sockaddr *)&client_address, &client_length);
        if (fd < 0) {
            if (atomic_load(&g_running)) perror("accept");
            continue;
        }
        pthread_mutex_lock(&g_send_lock);
        g_client_fd = fd;
        pthread_mutex_unlock(&g_send_lock);
        send_line("HELLO BXI_GRIPPER_DEMO 1\n");
        log_line("Client connected; backend %s (%s %s)",
                 BACKEND_REVISION, __DATE__, __TIME__);

        char input[2048];
        size_t used = 0;
        while (atomic_load(&g_running)) {
            ssize_t received = recv(fd, input + used, sizeof(input) - used - 1, 0);
            if (received <= 0) break;
            used += (size_t)received;
            input[used] = '\0';
            char *line_start = input;
            char *newline;
            while ((newline = strchr(line_start, '\n')) != NULL) {
                *newline = '\0';
                if (newline > line_start && newline[-1] == '\r') newline[-1] = '\0';
                if (*line_start) handle_command(fd, line_start);
                line_start = newline + 1;
            }
            used -= (size_t)(line_start - input);
            memmove(input, line_start, used);
            if (used == sizeof(input) - 1) used = 0;
        }
        reset_client_session();
        pthread_mutex_lock(&g_send_lock);
        if (g_client_fd == fd) g_client_fd = -1;
        pthread_mutex_unlock(&g_send_lock);
        close(fd);
        log_line("Client disconnected; motor power forced off");
    }

    reset_client_session();
    atomic_store(&g_running, 0);
    pthread_join(g_control_thread, NULL);
    if (g_server_fd >= 0) close(g_server_fd);
    bxi_pci_exit();
    return 0;
}
