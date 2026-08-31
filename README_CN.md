# FAIRINO FR5 + Quest 3 + IH01 遥操作项目

[English](README.md) | [中文](README_CN.md)

这是一个可移植、带安全门控的遥操作项目，用于通过 Meta Quest 3 手部追踪控制一台
法奥 FAIRINO FR5 机械臂和一只 IH01 灵巧手。Quest 手腕运动用于控制机械臂，21 个
OpenXR 手部关键点用于控制 IH01 的六个主动通道。

默认工作流只针对仿真；FR5 真机输出保持锁定。IH01 实体手保留在独立的
`IPE-quest-hand-teleop/` 项目内，并有自己的 EtherCAT 检查和人工解锁流程。

## 快速开始（不连接硬件）

```bash
cd /home/hzm/yyy/fairino-fr5-vr
make setup       # 首次安装
make arm-sim     # 单独打开 FR5 + IH01 仿真
```

Quest 已安装 APK 并通过 USB 授权后，只需：

```bash
make arm-teleop
```

该命令会检查 Quest、建立 ADB reverse、启动 IPE Quest Hand Teleop 应用并打开联合仿真。
头显中选择 **TCP Wired / localhost / 8000**。Quest 右手腕部平移驱动 FR5 数值逆解，同一只手的关键点驱动安装在末端的 IH01-X1-R；姿态控制仍在等待实测坐标标定。

在 `make ui` 的入口界面可选三种映射：右手（右腕+右手指）、左手（左腕+左手指）、双手（右腕控制 FR5、左手指控制 IH01）。也可在需要时通过 `make sim-teleop SIDE=left` 或 `SIDE=both` 选择。
默认坐标映射已调整为更符合站在机器人前方操作的方向：手向前对应 FR5 向前、手向右对应机器人横向、手向上对应末端上升。首次进入时仍会以当前手腕位置作为零点；若安装位置或操作者站位不同，应在 [teleop.yaml](config/teleop.yaml) 中完成现场标定后再用于真机。

## Quest 3 首次安装（新头显或换电脑）

`make install` 才会把 APK 安装到 Quest 3；`make reverse` 不安装程序，只建立电脑与头显之间的 `localhost:8000` 通道。首次使用请在手机 Meta Horizon App 开启开发者模式，用 USB-C 连接并解锁 Quest 3，在头显弹窗中允许 USB 调试并勾选始终允许。

```bash
cd /home/hzm/yyy/fairino-fr5-vr/IPE-quest-hand-teleop
make status      # 检查 adb 授权，必须显示 device
make install     # 安装 hand_tracking_streamer.apk
make reverse     # 建立 TCP 端口转发，不会再次安装 APK
```

看到 `Success` 和 `PASS: installed ... APK` 即表示安装完成；看到 `PASS: Quest localhost:8000 -> PC localhost:8000` 即表示通道建立成功。换另一台 Quest 3 时重复上述安装步骤即可。

如果需要重新生成 APK（例如修改 Unity 工程），使用：

```bash
make unity-build   # 仅用 Unity 构建并更新本地 APK
make install       # 将新 APK 安装到 Quest 3
make reverse       # 建立 localhost:8000 通道
```

`make unity-build` 需要本机已安装 Unity 6；它不会自动安装到头显。Unity 构建脚本固定写入
项目包名 `com.haoming.ipe.handteleop`，构建后直接运行 `make install`，启动脚本会使用同一包名。
旧的 `com.wengmister.handtrackingstreamer` 仅属于历史 APK，不应与新构建混用。

如果头显应用库里看不到程序，请在 Quest 3 的应用库筛选器中选择“未知来源（Unknown Sources）”。

## 项目结构

```text
src/fairino_fr5_vr/    协议、映射、安全门控、运行流程和设备适配器
config/teleop.yaml     Quest、FR5、IH01 的映射与安全配置
docs/                  架构、标定、操作和安全门控说明
tests/                 无硬件协议、映射和安全测试
scripts/               保留的 FR5 示教界面与诊断工具
third_party/           FAIRINO Python SDK 及其上游许可证
IPE-quest-hand-teleop/ 保留的 Quest 3 与 IH01 集成快照
```

## 使用命令

```bash
make help
make setup         # 首次安装依赖
make check         # 无硬件自检
make arm-sim       # 单独仿真，默认手动关节模式
make arm-teleop    # Quest + 联合仿真
make hand-teleop  # Quest + 实体 IH01（启动时选择手）
make ui            # 图形控制台与实时数据
make hand-control  # 保留的 IH01 实体手动控制台
```

`make arm-sim` 默认不再自动挪动，窗口中按 `q/a`、`w/s`、`e/d`、`r/f`、`t/g`、`y/h` 正反转动 J1～J6，`0` 回到初始姿态，`o/c` 控制手指开合，`Space` 暂停/继续。终端每 0.5 秒显示六轴角度和 IH01 末端位置。`make ui` 中可以启动或停止仿真、启动 Quest 联动、查看 Quest 映射目标、IH01 仿真位置、末端位置误差和数值 IK 误差。

灵巧手遥操统一使用 `make hand-teleop`，启动后进入 IH01 仿真镜像并选择右手、左手或双手；
仿真会限制在合理显示工作空间内（尤其避免落到地面以下）。窗口中的 `E` 键才会请求启用
实体 IH01 输出；`make hand-control` 则仍是实体 IH01 手动控制命令。

需要把 Quest 遥操接到实体 IH01 时，使用明确的硬件入口 `make hand-teleop`，启动时选择
右手或左手。窗口启动后默认未启用输出，按 `E` 才会启用/停用目标发送；`Space`
暂停，`R` 清故障，`Q` 退出。该入口会经过 EtherCAT 与人工确认流程；未接实体手时，按
`E` 会在界面显示未检测到实体 IH01，并保持仿真运行。进入后可用 HAND MODE 控件或
`l`、`r`、`b` 切换仿真显示手。

实体手控采用逐通道接触保护：电流达到 1000 mA，或位置堵转持续 200 ms，该通道保持当前位置；
将该通道目标松开/回退后立即释放。hand-control 默认不启用拇指–食指耦合软限位，也不叠加
额外力阈值，避免影响遥操灵活性。温度、故障码和 EtherCAT 状态仍持续监视，硬件急停与驱动器
自身保护不被软件旁路。

使用任何硬件前，请先阅读 [docs/OPERATIONS.md](docs/OPERATIONS.md)。集成运行流程默认且
有意保持为 dry-run。FR5 SDK 适配器只作为调试和投产接口存在，默认不会启用。IH01 真机
仍使用 `IPE-quest-hand-teleop/README.md` 中经过检查、独立解锁的运行流程。

## 当前验证边界

- 已支持：协议解析、数据帧组装、手腕锚定、笛卡尔目标限幅、单帧速度限制、IH01
  几何映射、超时门控、数据记录与回放，以及确定性的 dry-run 验证。组合 MuJoCo 模型
  包含 FR5 六轴、安装在法兰上的 IH01-X1-R 关节树、12 个执行器，以及供 Quest 仿真
  使用的平移数值逆解。
- FR5 外观现在只加载 FAIRINO 官方 V6 白色网格，不叠加外凸的自绘关节盖；其碰撞网格和运动学限位均以官方 URDF 为准。
- 已提供但尚未在本项目中完成现场调试：FAIRINO SDK 输出适配器。
- IH01 真机使用独立硬件流程；启动后保持未解锁状态，需要检查 EtherCAT OP、WKC 和
  故障状态，并由操作员按下 `E` 才能解锁。
- 当前不声明已经具备：碰撞规避、经过认证的功能安全、完成标定的 Quest 到机械臂坐标
  变换，或已经完成的实体遥操作试验。

第三方来源和许可证边界见 [docs/PROVENANCE.md](docs/PROVENANCE.md)。
FR5 代码与参数审计见 [docs/FR5_AUDIT_CN.md](docs/FR5_AUDIT_CN.md)。
