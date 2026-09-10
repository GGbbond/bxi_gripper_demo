/* Tests call production C functions; PCI is replaced by stubs, never hardware. */
#define main gripper_backend_main
#include "../backend/gripper_backend.c"
#undef main
#include <assert.h>

int bxi_pci_init(canfd_rx_call cb, void *arg, int cpu)
{ (void)cb; (void)arg; (void)cpu; return 0; }
int bxi_pci_exit(void) { return 0; }
int motor_pwr_set(unsigned int power) { (void)power; return 0; }
int canfd_send_packet(canfd_packet *packet, unsigned int count)
{ (void)packet; (void)count; return 0; }

static void reset(float position, float velocity)
{
    g_power_on = g_mode_entered = g_control_ready = 1;
    g_command_position = DEG_TO_RAD(position);
    g_command_velocity = DEG_TO_RAD(velocity);
    g_position_min_deg = -360.0f;
    g_position_max_deg = 360.0f;
}

static void target(float position, float speed)
{
    assert(start_motion(position, speed, 300.0f, 5.0f, 1) == 0);
}

static void step(float dt)
{
    const float before_p = g_command_position;
    const float before_v = g_command_velocity;
    update_stream_locked(dt);
    const float dv = fabsf(g_command_velocity - before_v);
    if (dv > g_motion.acceleration * dt + 1e-5f) {
        fprintf(stderr, "acceleration violation: dv=%.5f, allowed=%.5f deg/s\n",
                RAD_TO_DEG(dv), RAD_TO_DEG(g_motion.acceleration * dt));
        abort();
    }
    /* Position and velocity must describe the same integrated trajectory. */
    const float expected_step = 0.5f * (before_v + g_command_velocity) * dt;
    assert(fabsf(g_command_position - before_p - expected_step) < 1e-6f);
    assert(isfinite(g_command_position) && isfinite(g_command_velocity));
}

int main(void)
{
    /* Configured travel limits are also enforced in the backend. */
    reset(0.0f, 0.0f);
    g_position_min_deg = -20.0f;
    g_position_max_deg = 30.0f;
    assert(start_motion(-20.0f, 10.0f, 300.0f, 5.0f, 1) == 0);
    assert(start_motion(30.0f, 10.0f, 300.0f, 5.0f, 1) == 0);
    assert(start_motion(-20.1f, 10.0f, 300.0f, 5.0f, 1) == -2);
    assert(start_motion(30.1f, 10.0f, 300.0f, 5.0f, 1) == -2);

    /* A target moved close to the current trajectory must not reset velocity. */
    reset(10.0f, 180.0f);
    target(10.1f, 180.0f);
    step(0.002f);

    /* User reproduction: zero, then slow unidirectional 0.1-degree steps. */
    for (int direction = -1; direction <= 1; direction += 2) {
        reset(0.0f, 0.0f);
        float max_v = 0.0f, max_dv = 0.0f;
        for (int tick = 0; tick < 5000; tick++) {
            if (tick % 10 == 0) target(direction * (tick / 10 + 1) * 0.1f, 180.0f);
            const float before_v = g_command_velocity, before_p = g_command_position;
            step(0.002f);
            assert(direction * (g_command_position - before_p) >= -1e-6f);
            max_v = fmaxf(max_v, fabsf(g_command_velocity));
            max_dv = fmaxf(max_dv, fabsf(g_command_velocity - before_v));
        }
        printf("slow drag %s: peak v %.3f deg/s, peak dv %.3f deg/s\n",
               direction > 0 ? "+" : "-", RAD_TO_DEG(max_v), RAD_TO_DEG(max_dv));
        assert(RAD_TO_DEG(max_v) < 6.0f);
        for (int tick = 0; tick < 1000; tick++) step(0.002f);
        assert(fabsf(RAD_TO_DEG(g_command_position) - direction * 50.0f) < 0.01f);
    }

    /* Retarget/reverse/change speed with variable scheduling intervals. */
    reset(0.0f, 0.0f);
    for (int tick = 0; tick < 10000; tick++) {
        if (tick % 71 == 0) target((tick / 71) % 2 ? -100.0f : 100.0f,
                                  (tick / 71) % 3 ? 180.0f : 10.0f);
        step(tick % 13 ? 0.002f : 0.02f);
    }

    /* Current feedback can exceed the UI target range; never clamp it in one tick. */
    reset(400.0f, 0.0f);
    target(350.0f, 180.0f);
    step(0.002f);
    assert(RAD_TO_DEG(g_command_position) > 399.9f);
    for (int tick = 0; tick < 5000; tick++) step(0.002f);
    assert(fabsf(RAD_TO_DEG(g_command_position) - 350.0f) < 0.01f);
    puts("Backend slider regression tests passed (PCI stubs).");
    return 0;
}
