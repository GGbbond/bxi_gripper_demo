# BXI 夹爪控制演示

这是从“双电机对拖测试台”中独立拆出的客户演示软件，只保留 50L 电机齿轮齿条夹爪所需功能。界面与后端均可独立运行，不依赖原测试台软件。

## 功能

- BXI PCI 控制卡 CAN0～CAN6、电机 ID 0～8 可选
- 电机安全上电、下电、失能、无需重新上电即可再次使能、当前位置置零和急停
- 实时位置、速度、扭矩、MOS 温度和转子温度反馈
- `-360°～360°` 目标位置、峰值速度、Kp/Kd 控制
- 五次 S 曲线点到点运动；滑块使用连续限速、限加速度跟随
- 滑块小步拖动采用连续阻尼跟随，约 0.3 秒收敛，不在每个小目标处强制速度归零
- 置零期间暂停控制报文，等待稳定的新坐标反馈；必要时自动重试，无需重复点击
- 多点动作编辑、暂停、停止、循环播放
- 动作程序 JSON 导入与导出
- 主界面运行日志和可独立放大的完整日志窗口
- 放大日志窗口可导出最近约两分钟的运动诊断 CSV（指令、反馈、反馈序号和时效）
- 后端只监听 `127.0.0.1:9999`，客户端断开或后端退出时强制下电

> 安全提示：电机和夹爪会产生机械运动。使用前必须固定机构、清空运动区域，并确保操作人员能够立即切断硬件电源。软件急停不能替代硬件急停。

## 源码运行

推荐 Ubuntu 22.04 x86_64。先安装基础环境：

```bash
sudo apt update
sudo apt install gcc cmake make python3-venv pkexec polkitd
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

编译硬件后端：

```bash
bash scripts/build_backend.sh
```

BXI PCI 用户态驱动依赖已经放在仓库内：

```text
backend/bxi_pci_drv/bxi_pci_drv.h
backend/bxi_pci_drv/libbxi_pci_drv.a
```

默认构建不再读取原测试台或其他绝对路径。需要临时替换驱动库时可以指定其他目录：

自定义驱动位置示例：

```bash
BXI_PCI_DRV_DIR=/opt/bxi/bxi_pci_drv/lib bash scripts/build_backend.sh
```

启动界面：

```bash
./run.sh
```

点击“启动并连接”后，软件会通过 `pkexec` 请求一次系统授权，以便后端访问 PCI 硬件和电机电源。

## 客户操作流程

1. 连接 PCI 控制卡、电机 CAN-H/CAN-L 和电机电源，固定好夹爪机构。
2. 打开软件，点击“启动并连接”，在系统授权窗口中确认。
3. 在下电状态选择 CAN 通道和电机 ID；50L 电机通常使用 `CAN2`、`ID 1`。
4. 点击“夹爪电机上电”，等待界面显示“电机已上电并保持当前位置”。
5. 先使用低速度、小角度测试方向，再按需要执行位置或动作程序。
6. 置零前把机构移动到确认安全的机械零位，再点击“位置置零”并二次确认。
7. 结束后点击“急停并下电”或“夹爪电机下电”。

“活动范围最小值/最大值”用于限制夹爪可接收的位置目标，默认范围为 -360° 到 360°。
请在电机下电时修改；该限制同时作用于滑块、手动前往、动作程序和后端命令。
为避免误操作，软件中的数值输入框和下拉框不会响应鼠标滚轮。

连接 Xbox 类 USB/无线手柄后，在“Xbox 类手柄控制”区域选择 `/dev/input/js*` 设备。
右扳机默认使用轴 4；如果进度条不响应，可切换轴编号。启用前应先松开右扳机：
进度条 0% 对应活动范围最大角度，100% 对应最小角度，中间位置按线性比例控制。
启用手柄会停止正在执行的动作程序，并暂时禁用滑块和手动位置控制。

如果上电后一直没有进入就绪状态，请检查 CAN 通道、电机 ID、接线、PCI 驱动和电机固件启动状态。

排查问题时，先确认连接日志中出现 `backend position-limits-v3`，用于确认连接到新后端。
重新编译不会替换已运行的后端进程；如日志仍是旧版本，需在下电后退出旧后端并重新启动软件。
复现异常后可点击“放大日志 → 导出运动诊断 CSV”。其中 `command_*` 是后端生成的控制参考，
不代表已确认电机执行；`feedback_*` 是选中电机的返回数据，`feedback_age_ms=-1` 表示反馈无效。
重新连接时开始新的诊断记录，断开后仍可导出本次数据。

软件回归测试（不连接硬件）：

```bash
cc -std=c11 -Wall -Wextra -Ibackend/bxi_pci_drv tests/test_slider_backend.c -lpthread -lm -o /tmp/gripper_slider_test
/tmp/gripper_slider_test
QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s tests -p 'test_*ui.py' -v
```

## 打包给客户

在与客户目标系统兼容的 Ubuntu x86_64 环境中执行：

```bash
source .venv/bin/activate
python -m pip install pyinstaller
bash scripts/package_app.sh
```

生成文件：

```text
dist/bxi_gripper_demo-linux-x86_64-installer.run
dist/bxi_gripper_demo-linux-x86_64.tar.gz
```

推荐把单文件安装包发给客户。客户安装命令：

```bash
chmod +x bxi_gripper_demo-linux-x86_64-installer.run
./bxi_gripper_demo-linux-x86_64-installer.run
```

安装后可在应用菜单搜索“BXI 夹爪控制演示”。卸载：

```bash
./bxi_gripper_demo-linux-x86_64-installer.run --uninstall
```

Python、PyQt5 和后端所需的用户态 PCI 静态库会被打进客户包；PCI 内核驱动、硬件权限组件和 `pkexec` 属于目标系统环境，不会被打包。

## 上传 GitHub 前

仓库现在已经包含构建夹爪软件所需的用户态依赖，不再依赖原测试台目录。`libbxi_pci_drv.a` 是 x86_64 静态库，因此源码默认面向 Ubuntu x86_64。

原驱动目录没有提供 LICENSE 文件。将仓库设为公开之前，请确认 BXI PCI 驱动静态库的公开发布授权，并在仓库根目录补充适用的许可证。

## 动作程序格式

导出的 `.json` 文件带有格式名和版本号，包含每个动作点的位置、峰值速度、估算段时间与到达后等待时间。软件只接受本格式且会再次校验位置、速度和等待时间，便于把调好的动作文件一同交付客户。
