# BXI 夹爪控制 Demo

面向客户演示和夹爪联调的独立桌面软件，适用于采用 BXI PCI 控制卡、CAN 通信和 50L 电机的齿轮齿条夹爪。软件包含图形界面和本机硬件后端，不依赖原“双电机对拖测试台”。

客户可通过目标位置、滑动条、Xbox 类手柄右扳机或多点动作程序控制夹爪，并实时查看位置、速度、反馈扭矩和温度。软件还提供活动范围限制、软件力矩限制、位置置零、失能、急停下电和运动诊断导出。

## 1. 使用前必读

> **机械运动警告：** 夹爪可能造成挤压、夹伤或设备损坏。首次调试必须固定夹爪，清空运动区域，以低速、小行程验证方向，并保证操作者能够立即切断硬件电源。软件“急停并下电”不能替代符合设备安全等级要求的硬件急停、固件限流和机械限位。

本软件的控制约定如下：

- **角度减小表示夹紧，角度增大表示松开。** 软件限力的方向判断也使用该约定。如果实际机构方向相反，必须先停止使用并修改电机安装方向或控制配置，不能直接依赖软件限力。
- 位置单位为电机角度 `deg`，速度单位为电机角速度 `deg/s`，扭矩单位为 `N·m`。这些数值不一定等于夹爪指尖的位移、速度和夹持力。
- “活动范围”限制的是软件允许发送的目标位置，不是硬件安全限位。机械端仍需保留限位、缓冲和足够余量。
- 置零会把夹爪**当前位置重新定义为 0°**。置零后，原有位置数值和动作文件可能不再对应原来的机械位置。
- 电机“失能”会撤销电机保持力，但硬件电源仍然开启；负载可能因重力或外力突然移动。

## 2. 运行环境与硬件准备

推荐环境：

- Ubuntu 22.04 x86_64
- BXI PCI 控制卡及已正确安装的 PCI 内核驱动
- 50L 电机夹爪、匹配的电源和 CAN 接线
- `pkexec`/Polkit 系统授权组件
- 可选：支持 Linux joystick 接口的 Xbox 类 USB 或无线手柄

接线和上电前请确认：

1. 夹爪已牢固安装，机械活动范围内没有人员、线缆或障碍物。
2. 电机电源规格、CAN-H/CAN-L 和终端电阻符合硬件要求。
3. 已知正确的 CAN 通道和电机 ID。本项目默认值为 `CAN2`、`ID 1`，现场配置可能不同。
4. 有独立切断电机电源的方法，并已确认硬件急停有效。

## 3. 客户安装与启动

### 3.1 使用安装包

给客户分发以下文件：

```text
bxi_gripper_demo-linux-x86_64-installer.run
```

安装：

```bash
chmod +x bxi_gripper_demo-linux-x86_64-installer.run
./bxi_gripper_demo-linux-x86_64-installer.run
```

安装完成后，在应用菜单中搜索“BXI 夹爪控制演示”。也可以从终端启动：

```bash
~/.local/share/bxi_gripper_demo/app/bxi_gripper_demo
```

卸载软件：

```bash
./bxi_gripper_demo-linux-x86_64-installer.run --uninstall
```

卸载不会删除客户自行导出的动作 JSON、本机设置或诊断 CSV。

### 3.2 首次连接

1. 启动软件，确认界面显示“未连接”。
2. 在电机未上电时选择正确的 CAN 通道和电机 ID。
3. 点击“启动并连接”，在系统授权窗口中确认。后端需要硬件访问权限，因此启动时会调用 `pkexec`。
4. 等待界面显示“已连接”。
5. 点击“夹爪电机上电”。电机控制器启动并返回有效反馈后，界面会显示“电机已上电并保持当前位置”。
6. 先设置安全的活动范围，再以低速、小角度执行首次运动。

点击“断开连接”或关闭软件时，界面会请求后端退出并等待清理完成。界面启动后端时使用 `--managed`，通过继承的标准输入管道管理后端生命周期：即使 TCP 连接失败、丢失或界面崩溃，管道 EOF 也会触发后端下电和 `bxi_pci_exit()`。关闭主窗口同时关闭日志子窗口。此机制不能处理驱动调用永久卡住、内核故障或后端遭到 SIGKILL 的情况。仍不能把该机制作为唯一的安全保护。

## 4. 界面与操作说明

### 4.1 设备与实时反馈

“设备”区域用于选择 CAN 通道和电机 ID。为了避免把控制报文发给错误设备，只能在电机下电时修改。

实时卡片含义：

| 项目 | 含义 |
| --- | --- |
| 实时位置 | 电机反馈位置，单位 `deg` |
| 实时速度 | 电机反馈速度，单位 `deg/s` |
| 反馈扭矩 | 电机协议返回的估算扭矩，夹紧时本夹爪通常为负值 |
| MOS / 转子温度 | 驱动功率器件和电机转子温度 |

反馈值来自 CAN 报文。没有有效反馈时，不应继续执行运动。

### 4.2 活动范围与位置控制

“活动范围最小值/最大值”共同限定滑动条、目标位置、手柄映射、动作程序和后端可接受的位置。请在电机下电时设置，并在机械极限前预留安全余量。

位置控制参数：

| 参数 | 作用 | 使用建议 |
| --- | --- | --- |
| 目标位置 | 点到点运动的终点 | 首次只改变很小角度，确认运动方向 |
| 峰值速度 | 五次 S 曲线允许达到的最大速度 | 空载低速验证后再逐步提高 |
| Kp | 位置刚度，数值越大位置误差产生的校正扭矩通常越大 | 使用经过验证的电机参数，不要为了“夹得更紧”盲目增大 |
| Kd | 速度阻尼 | 与 Kp 配套调试，过大或过小都可能影响稳定性 |

点击“前往位置”后，软件使用五次 S 曲线生成平滑参考。近似段时间为：

```text
段时间 ≈ 1.875 × |目标角度 - 起始角度| ÷ 峰值速度
```

拖动滑动条时，软件使用连续限速、限加速度跟随，不会在每个小刻度强制停下。拖动前仍应确认滑块当前值和夹爪实际位置一致，尤其是在置零、重新使能或更改活动范围后。

“命令”显示软件最近请求的位置，“实时”显示电机反馈位置。两者存在小幅动态误差属于正常现象；误差持续增大或方向相反时应立即停止。

为防止误触，所有数值输入框和下拉框都不会响应鼠标滚轮。请点击输入框后使用键盘输入或上下箭头调整。

### 4.3 Kp/Kd、失能、重新使能与置零

- 修改 Kp/Kd 后点击“应用 Kp/Kd”才会发送新参数。
- 点击“电机失能”后，电机停止主动保持，但电机电源仍开启。按钮随后会变为“重新使能”，无需重新给整机断电即可恢复控制。
- “位置置零”会停止手柄和动作程序，短暂失能电机，发送置零指令并等待新坐标反馈；反馈不稳定时软件会自动重试。执行前必须把机构放在可重复且安全的机械基准位置。
- 置零完成后应重新确认活动范围，并从小角度开始验证；不要直接运行使用旧坐标制作的动作程序。

### 4.4 软件力矩限制

软件力矩限制用于在夹紧方向上抑制电机扭矩继续升高，设置范围为 `0.05–40 N·m`。

操作方式：

1. 在输入框中填写力矩上限。
2. 点击“应用限力值”，提交新的上限值。仅修改输入框不会改变当前正在使用的数值。
3. 勾选“启用软件限力”会**立即启用**当前数值；取消勾选会**立即关闭**限力，不需要再点击按钮。
4. 进度条显示滤波后的扭矩绝对值和上限；出现“限力中”表示后端已介入。

夹紧扭矩为负值时，软件按绝对值判断。例如上限为 `0.50 N·m`，反馈为 `-0.90 N·m` 时会触发限力。控制逻辑为：

- 反馈扭矩达到上限时暂停继续向夹紧方向推进。
- 超过上限约 5% 时，参考位置会缓慢向松开方向回退，以降低位置控制产生的扭矩。
- 原始值和滤波值都降到上限的 90% 以下后，才允许继续夹紧，减少临界点附近的高频启停。
- 松开方向始终放行，确保高负载时仍能释放物体。
- 夹紧过程中扭矩反馈超过 100 ms 未更新时，后端暂停继续夹紧。
- 轨迹结束后位置控制仍可能保持夹持力，因此限力在位置保持阶段也持续工作；已经夹住物体后再勾选限力同样会生效。

软件采用 MIT 位置控制，电机输出扭矩大致受位置误差、Kp、速度误差和 Kd 共同影响。软件限力通过 CAN 反馈监控并调整位置参考，不是电机固件内部的硬扭矩钳位，因此采样和控制延迟可能造成短时超调。

> **扭矩不等于指尖夹持力。** 指尖力还与传动比、齿轮齿条有效半径、机构效率、摩擦和双指受力方式有关。需要以牛顿为单位控制夹持力时，应进行机构标定，并使用合适的力传感器或固件闭环。

### 4.5 Xbox 类手柄右扳机控制

右扳机提供线性位置控制：松开扳机对应活动范围最大角度，完全按下对应最小角度。若扳机深度为 `d`（0 到 1），则：

```text
目标角度 = 最大角度 - d × (最大角度 - 最小角度)
```

使用步骤：

1. 连接手柄，点击“刷新”，选择 `/dev/input/js*` 设备。
2. 选择右扳机轴。常见 Xbox/PowerA 手柄为“轴 4（常见 Xbox RT）”，不同型号可能不同。
3. **完全松开右扳机**，再点击“启用右扳机控制”。启用瞬间软件会先命令活动范围最大角度。
4. 缓慢按压扳机，观察“扳机按下”进度条和夹爪位置。
5. 点击“停止右扳机控制”退出。手柄断开或电机失去就绪状态时，软件也会自动停止手柄控制。

启用手柄会停止正在运行的动作程序，并暂时禁用滑动条和手动位置按钮，防止多个控制源同时发送位置目标。软件力矩限制可以与手柄同时使用，建议夹取物体前先验证限力值。

手柄没有响应时，可在终端查看全部轴和按键：

```bash
sudo apt install joystick
jstest /dev/input/js0
```

按下右扳机，观察哪个 `Axes` 数值变化，再在软件中选择相同轴号。

### 4.6 动作程序

动作程序用于重复演示开合、停留和多位置运动：

1. 把夹爪移动到安全起点，点击“添加实时位置”。第一行是动作起点。
2. 设置下一个目标位置，点击“添加目标位置”，重复添加所需动作点。
3. 每一行可编辑位置、峰值速度或段时间，以及到达后的等待时间。修改峰值速度时软件自动计算段时间；修改段时间时自动反算峰值速度。
4. 点击“开始”执行。暂停或停止后，电机会保持当前位置；只有失能或下电才会释放保持力。
5. 勾选“循环播放”后，最后一点完成会以“回起点速度”返回第一点并继续循环。
6. 使用“导出动作程序”保存为 JSON，在同一坐标零点和活动范围下可再次导入。

## 5. 程序实现与夹爪控制示例

本节面向需要阅读源码或把夹爪控制集成到自己上位机的开发人员。示例展示的是程序控制链路和实现方法，不是界面操作步骤。

### 5.1 软件架构

```text
gripper_demo.py（PyQt5 客户端）
  ├─ 手动位置、滑块、动作表、手柄输入
  ├─ 20 ms 输入采样和状态接收
  └─ TCP 文本命令（127.0.0.1:9999）
                    │
                    ▼
gripper_backend.c（单客户端硬件后端）
  ├─ 命令校验与状态机
  ├─ 五次 S 曲线 / 连续目标跟随
  ├─ 软件力矩监控
  ├─ 2 ms MIT 控制周期
  └─ BXI PCI 用户态驱动
                    │
                    ▼
           CAN FD 电机控制与反馈
```

界面不直接按鼠标事件发送 CAN 帧，而是把目标传给后端。后端以固定控制周期持续生成一致的 `p_des` 和 `v_des`，再打包成 MIT 控制帧。这样即使界面短时卡顿，CAN 控制周期也不会跟着界面刷新率变化。

后端目前只监听本机回环地址并只服务一个客户端会话；客户端断开后，后端下电、释放 PCI 并退出，重新连接会启动新实例。自定义程序与图形界面不能同时控制同一个后端实例。

主要源码入口是 [`gripper_demo.py`](gripper_demo.py) 和 [`backend/gripper_backend.c`](backend/gripper_backend.c)。当前 TCP 接口用于本 Demo 的本机进程通信；客户产品若长期依赖该接口，应锁定协议版本，并为命令、响应和兼容性增加独立测试。

### 5.2 本机 TCP 控制协议

客户端连接 `127.0.0.1:9999` 后应先等待：

```text
HELLO BXI_GRIPPER_DEMO 1
```

每条命令必须以换行符 `\n` 结束。常用命令如下：

| 命令 | 说明 | 关键前提 |
| --- | --- | --- |
| `SET_CLAW_CAN <0..6>` | 选择 CAN 通道 | 必须下电 |
| `SET_MOTOR_ID <0..8>` | 选择电机 ID | 必须下电 |
| `SET_POSITION_LIMITS <min> <max>` | 设置软件位置范围，单位 deg | 必须下电；`min ≤ 0 ≤ max` 且 `min < max` |
| `SET_GAINS <kp> <kd>` | 设置 MIT 位置增益 | `0≤Kp≤500`，`0≤Kd≤5` |
| `SET_TORQUE_LIMIT <0或1> <Nm>` | 关闭/启用并更新软件限力 | 可在上电时修改；范围 `0.05–40` |
| `MOTOR_POWER_ON` | 打开电机硬件电源并请求进入 MIT 模式 | 必须继续等待就绪反馈 |
| `MOTOR_POWER_STATUS` | 查询控制是否就绪 | 返回 `MOTOR_POWER_READY 0/1` |
| `CLAW_MOVE <pos> <speed> <kp> <kd>` | 五次 S 曲线点到点运动 | 电机已就绪且目标在活动范围内 |
| `CLAW_STREAM <pos> <speed> <kp> <kd>` | 更新连续跟随目标 | 用于滑块、手柄等连续输入 |
| `CLAW_STOP` | 停止当前轨迹并保持反馈位置 | 电机仍保持使能 |
| `CLAW_ZERO` | 当前位置置零并自动重新使能 | 电机已上电；机械位置安全 |
| `CLAW_DISABLE` / `CLAW_ENABLE` | 失能/重新使能电机 | `CLAW_ENABLE` 要求硬件电源仍开 |
| `MOTOR_POWER_OFF` | 退出控制模式并关闭电机电源 | 推荐在客户端退出前执行 |
| `SHUTDOWN` | 终止后端进程 | 仅由拥有后端生命周期的客户端使用 |

后端会异步发送 `POS`、`VEL`、`TORQUE`、温度、`TRACE` 和 `TORQUE_LIMIT_STATUS`。TCP 是字节流，一次 `recv()` 可能得到半行或多行；集成程序必须缓存数据并按 `\n` 拆行，不能假设一次接收等于一条消息。

### 5.3 Python 控制示例

先在一个终端独立启动后端：

```bash
sudo ./build/bin/gripper_backend
```

下面代码展示正确的连接、配置、上电等待和运动命令顺序。示例位置仅用于说明，实际运行前必须替换为经过验证的安全参数。

```python
import socket
import time


class GripperClient:
    def __init__(self, host="127.0.0.1", port=9999):
        self.sock = socket.create_connection((host, port), timeout=3.0)
        self.stream = self.sock.makefile("r", encoding="utf-8", newline="\n")
        self.wait_for("HELLO BXI_GRIPPER_DEMO 1", timeout=3.0)

    def send(self, command):
        self.sock.sendall((command + "\n").encode("utf-8"))

    def wait_for(self, prefix, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.sock.settimeout(max(0.1, deadline - time.monotonic()))
            line = self.stream.readline()
            if not line:
                raise ConnectionError("后端连接已关闭")
            line = line.strip()
            if line.startswith("ERROR "):
                raise RuntimeError(line)
            if line.startswith(prefix):
                return line
        raise TimeoutError(f"等待 {prefix} 超时")

    def configure(self, can_bus, motor_id, position_min, position_max,
                  kp, kd, torque_limit_nm):
        self.send(f"SET_CLAW_CAN {can_bus}")
        self.wait_for("CLAW_CAN_SET")
        self.send(f"SET_MOTOR_ID {motor_id}")
        self.wait_for("MOTOR_ID_SET")
        self.send(f"SET_POSITION_LIMITS {position_min} {position_max}")
        self.wait_for("POSITION_LIMITS_SET")
        self.send(f"SET_GAINS {kp} {kd}")
        self.wait_for("GAINS_SET")
        self.send(f"SET_TORQUE_LIMIT 1 {torque_limit_nm}")
        self.wait_for("TORQUE_LIMIT_SET")

    def power_on(self):
        self.send("MOTOR_POWER_ON")
        self.wait_for("MOTOR_POWERING_ON")
        # 不能在 MOTOR_POWERING_ON 后立即运动；必须等电机返回有效反馈。
        while True:
            self.send("MOTOR_POWER_STATUS")
            if self.wait_for("MOTOR_POWER_READY", timeout=2.0).endswith(" 1"):
                return
            time.sleep(0.2)

    def move(self, position_deg, speed_deg_s, kp, kd):
        self.send(f"CLAW_MOVE {position_deg} {speed_deg_s} {kp} {kd}")
        self.wait_for("CLAW_MOVE_STARTED")

    def stream_target(self, position_deg, speed_deg_s, kp, kd):
        self.send(f"CLAW_STREAM {position_deg} {speed_deg_s} {kp} {kd}")

    def stop(self):
        self.send("CLAW_STOP")
        self.wait_for("CLAW_HOLDING")

    def power_off(self):
        self.send("MOTOR_POWER_OFF")
        self.wait_for("MOTOR_POWER_OFF_COMPLETE")


gripper = GripperClient()
try:
    # 示例假设 0° 接近闭合、120° 为张开，减小角度表示夹紧。
    gripper.configure(
        can_bus=2,
        motor_id=1,
        position_min=0.0,
        position_max=120.0,
        kp=300.0,
        kd=5.0,
        torque_limit_nm=0.50,
    )
    gripper.power_on()

    input("确认运动区域安全后按 Enter，夹爪将低速张开：")
    gripper.move(position_deg=110.0, speed_deg_s=20.0, kp=300.0, kd=5.0)
    gripper.wait_for("CLAW_MOVE_COMPLETE", timeout=15.0)

    input("确认可以夹取后按 Enter，夹爪将向小角度运动：")
    gripper.move(position_deg=20.0, speed_deg_s=10.0, kp=300.0, kd=5.0)
    # 接触物体且限力持续介入时，运动可能不会返回 MOVE_COMPLETE。
    # 实际应用应同时解析 TORQUE_LIMIT_STATUS，并提供取消/松开操作。
    time.sleep(3.0)

    # 增大角度为松开方向，限力不会阻断该命令。
    gripper.move(position_deg=110.0, speed_deg_s=20.0, kp=300.0, kd=5.0)
    gripper.wait_for("CLAW_MOVE_COMPLETE", timeout=15.0)
finally:
    # 生产程序还应处理 SIGINT、进程异常和通信中断。
    try:
        gripper.power_off()
    finally:
        gripper.sock.close()
```

该代码用于解释协议调用顺序，不是对任意机构都安全的即插即用参数。正式集成时应把接收处理放到独立线程或事件循环中，持续维护最新反馈、连接状态和急停状态。

### 5.4 点到点与连续目标的实现

`CLAW_MOVE` 适合明确终点的单次运动。后端不会把最终位置一步写入电机，而是生成五次多项式轨迹，使起止速度连续。代码位于 `update_move_locked()`，段时间按距离和峰值速度计算。

`CLAW_STREAM` 适合滑块、摇杆、扳机或网络遥操作。上位机可以约每 20 ms 更新目标，但后端不会在每个新目标到来时把速度清零。`update_stream_locked()` 使用临界阻尼跟随器，并限制速度变化量，再用梯形积分同步更新位置和速度参考。这一点用于避免连续拖动时出现“追目标—突然归零—再次加速”的顿挫。

右扳机的程序映射为：

```python
depth = (raw_axis + 32767) / 65534       # 双极轴 -32767..32767
depth = max(0.0, min(1.0, depth))
target = position_max - depth * (position_max - position_min)
send(f"CLAW_STREAM {target} {speed} {kp} {kd}")
```

部分手柄扳机上报 `0..32767`，因此实际代码会在启用时判断轴是单极还是双极。新设备应先用 `jstest` 验证轴号、松开值和按下值，再决定归一化公式。

### 5.5 MIT 控制帧与限力实现

后端发送的 MIT 控制量为：

```text
p_des, v_des, Kp, Kd, t_ff
```

本 Demo 的 `t_ff` 固定为 `0`，夹持扭矩来自电机内部的位置/速度 PD 控制。`pack_control()` 将参数压缩到 8 字节 CAN FD 数据区；当前编码量程为位置 `±12.5 rad`、速度 `±45 rad/s`、Kp `0..500`、Kd `0..5`、力矩字段 `±40 N·m`。

`±40 N·m` 是协议字段的编码量程，**不是电机输出扭矩的硬限制**。仅仅把 `t_ff` 设为 0 或改变编码量程，不能限制 Kp/Kd 产生的电机扭矩。

软件限力的程序逻辑是：

1. 每个 2 ms 控制周期更新一次反馈扭矩绝对值滤波。
2. 用目标/命令位置和反馈位置判断当前是否仍在向小角度夹紧。
3. 原始或滤波扭矩达到上限后冻结夹紧轨迹。
4. 超过上限 105% 时逐步增大命令角度，减小位置误差和 PD 扭矩。
5. 原始值与滤波值都低于上限 90% 后恢复轨迹。
6. 保持状态仍继续监控；显式松开目标始终放行。

相关实现位于 `update_torque_filter_locked()`、`torque_limit_blocks_closing_locked()` 和 `update_torque_limited_reference_locked()`。

### 5.6 二次开发的控制注意事项

- **必须等待真实反馈就绪。** `MOTOR_POWERING_ON` 只表示电源流程开始；收到 `MOTOR_POWER_READY 1` 后才能发送运动命令。后端首次进入 MIT 模式时会用反馈位置初始化 `p_des`，避免使能瞬间跳到旧目标。
- **不要直接跳变位置参考。** 点到点运动使用 S 曲线；连续输入使用 `CLAW_STREAM`。如果自己改写后端，也必须保证 `p_des`、`v_des` 和时间步一致，并限制速度与加速度。
- **不要在每次连续目标更新时清零速度。** 这会在慢拖、反向和靠近目标时产生重复加减速，严重时形成位置突变。
- **位置范围必须在后端再次校验。** 不能只依赖界面控件；动作文件、网络输入和手柄映射都必须经过同一范围检查。
- **置零期间必须清空旧目标。** 置零前停止连续输入和动作队列；置零后等待新反馈，再从新坐标生成目标，不能继续发送旧坐标系中的缓存命令。
- **反馈必须带时效判断。** 不要拿最后一次旧扭矩或旧位置无限期参与控制。本后端在夹紧时超过 100 ms 没有新扭矩反馈便暂停推进。
- **力矩方向与机构方向要分别确认。** 当前机构夹紧反馈通常为负，但阈值比较使用绝对值；是否允许运动则依据“小角度夹紧、大角度松开”的机构约定。
- **协议编码量程必须与电机固件一致。** 改变 `P/V/Kp/Kd/T` 的缩放常量会改变所有打包和解包结果，不能把量程常量当作限幅参数随意修改。
- **所有共享控制状态必须同步。** CAN 接收回调、TCP 命令线程和 2 ms 控制线程会并发读写状态；新增字段应放在同一状态锁保护下，发送锁不要与硬件控制锁形成反向嵌套。
- **始终保留松开和下电路径。** 限力状态不能拦截增大角度的松开命令；客户端异常、TCP 断开和进程退出都应进入失能/下电流程。
- **不要让多个控制源竞争。** 图形界面、客户程序、手柄和自动动作不能同时直接写目标。应由一个仲裁层明确当前控制权。
- **不要阻塞控制线程。** 日志、文件写入、网络等待和界面刷新必须与 2 ms CAN 发送周期解耦。
- **软件限力不是安全功能。** 正式产品仍需固件电流/扭矩限制、硬件急停、机械限位以及必要的力传感器闭环。

## 6. 正常停机与异常处理

正常结束演示：

1. 停止手柄或动作程序。
2. 将夹爪移动到安全的释放位置。
3. 点击“夹爪电机下电”。
4. 确认电机已经释放后再断开连接或关闭软件。

发生方向错误、异常声音、位置突变、温度异常、反馈丢失或机械干涉时：

1. 立即点击“急停并下电”。
2. 如软件没有响应，立即使用硬件急停或切断电机电源。
3. 排除机械、接线、坐标和参数问题后，先空载、低速、小行程复测。
4. 不要通过反复增大 Kp、速度或力矩上限来掩盖故障。

## 7. 日志、诊断与常见问题

运行日志区域和上方控制区之间的分隔线可以上下拖动。点击“放大日志”可打开独立窗口，并导出最近约两分钟的运动诊断 CSV。

CSV 主要字段：

| 字段 | 含义 |
| --- | --- |
| `command_position_deg` / `command_velocity_deg_s` | 后端实际生成的参考位置和速度 |
| `target_position_deg` | 当前目标位置 |
| `feedback_position_deg` / `feedback_velocity_deg_s` | 电机反馈位置和速度 |
| `feedback_torque_Nm` | 有符号反馈扭矩 |
| `feedback_sequence` / `feedback_age_ms` | 反馈序号和时效；`-1` 表示反馈无效 |
| `torque_limit_enabled` / `torque_limit_Nm` | 限力开关状态和设定值 |
| `filtered_abs_torque_Nm` / `torque_limit_active` | 滤波扭矩绝对值和限力介入状态 |

常见问题：

- **点击连接后超时：** 检查系统授权窗口、`pkexec`、PCI 内核驱动、控制卡和硬件连接。
- **上电后一直未就绪：** 检查 CAN 通道、电机 ID、CAN 接线、电机电源和电机固件启动状态。
- **运动方向相反：** 立即下电。软件限力假设角度减小为夹紧，方向未纠正前不要夹取物体。
- **限力没有介入：** 确认复选框已勾选、进度条显示正确上限，并确认日志包含 `backend torque-limit-hold-v7-lifecycle`。旧后端进程必须完全退出后再重启软件。
- **手柄没有反应：** 使用 `jstest` 找到实际右扳机轴，检查 `/dev/input/js*` 是否存在及当前用户是否有读取权限。
- **置零后目标不正确：** 置零已改变坐标系，应重新确认活动范围、滑块位置和动作文件。
- **出现位置突变：** 立即下电并保存诊断 CSV；检查电机固件的位置编码、跨圈/坐标边界限制和反馈连续性。

重新连接会开始一份新的诊断记录，断开后仍可导出当前记录。诊断中的命令参考不代表电机已确认执行，判断实机行为必须同时查看反馈字段。

## 8. 从源码运行

客户仅使用安装包时不需要 Python。开发和二次集成可按以下方式构建：

```bash
sudo apt update
sudo apt install gcc cmake make python3-venv pkexec polkitd
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
bash scripts/build_backend.sh
./run.sh
```

仓库已经包含构建后端所需的 BXI PCI 用户态依赖：

```text
backend/bxi_pci_drv/bxi_pci_drv.h
backend/bxi_pci_drv/libbxi_pci_drv.a
```

如需使用其他版本的驱动依赖：

```bash
BXI_PCI_DRV_DIR=/opt/bxi/bxi_pci_drv/lib bash scripts/build_backend.sh
```

后端只监听本机 `127.0.0.1:9999`，不对局域网开放。

## 9. 测试与客户包构建

不连接硬件的回归测试（生命周期测试使用随机本机 TCP 端口和 PCI 桩函数）：

```bash
cc -std=c11 -Wall -Wextra -Ibackend/bxi_pci_drv \
  tests/test_slider_backend.c -lpthread -lm -o /tmp/gripper_slider_test
/tmp/gripper_slider_test
QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s tests -p 'test_*.py' -v
```

构建客户安装包：

```bash
source .venv/bin/activate
python -m pip install pyinstaller
bash scripts/package_app.sh
```

输出文件：

```text
dist/bxi_gripper_demo-linux-x86_64-installer.run
dist/bxi_gripper_demo-linux-x86_64.tar.gz
```

安装包包含 Python、PyQt5、图形界面和用户态后端。目标计算机仍需具备兼容的 PCI 内核驱动、硬件权限配置和 Polkit。

## 10. 交付与许可说明

`backend/bxi_pci_drv/libbxi_pci_drv.a` 是预编译的 x86_64 静态库，所以当前源码和安装包面向兼容的 Ubuntu x86_64 系统。驱动依赖原目录未提供 LICENSE 文件；公开发布或向第三方再分发前，应确认该驱动库的授权范围，并在项目根目录补充适用的软件许可证、供应商信息和技术支持联系方式。

建议客户交付内容至少包含：

- 安装包及对应版本号/校验值
- 本 README
- 已验证的 CAN 通道、电机 ID、机械零点和活动范围
- 已验证的 Kp/Kd、速度和软件限力参数
- 适用于客户机构的动作程序 JSON
- 硬件接线说明、急停方案和售后联系方式
