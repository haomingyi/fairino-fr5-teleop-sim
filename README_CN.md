# FAIRINO FR5 + Quest 3 + IH01 遥操作项目

[English](README.md) | [中文](README_CN.md)

这是一个可移植、带安全门控的遥操作项目，用于通过 Meta Quest 3 手部追踪控制一台
法奥 FAIRINO FR5 机械臂和一只 IH01 灵巧手。Quest 手腕运动用于控制机械臂，21 个
OpenXR 手部关键点用于控制 IH01 的六个主动通道。

默认工作流只针对仿真；FR5 真机输出保持锁定。IH01 实体手保留在独立的
`IPE-quest-hand-teleop/` 项目内，并有自己的 EtherCAT 检查和人工解锁流程。

## 快速开始（不连接硬件）

```bash
cd fairino-fr5-vr
make setup       # 首次安装
make arm-sim     # 单独打开 FR5 + IH01 仿真
```

Quest 已安装 APK 并通过 USB 授权后，只需：

```bash
make arm-teleop
```

该命令会检查 Quest、建立 ADB reverse，并打开联合仿真和遥操面板；不再启动电脑端 Quest 镜像窗口，
也不会强制重启头显应用。请在头显中手动启动应用并选择 **TCP Wired / localhost / 8000**。
Quest 手腕的相对平移和旋转驱动 FR5 六维末端位姿逆解，手部关键点驱动安装在末端的
IH01-X1-R。首次使用仍须实测坐标方向和比例。

直接执行命令时会提示选择右手、左手或双手映射；进入遥操面板后也可随时切换，不需要在命令后附加 `SIDE=...`。
默认坐标映射已调整为更符合站在机器人前方操作的方向：手向前对应 FR5 向前、手向右对应机器人横向、手向上对应末端上升。面板内可切换右手、左手或双手映射，并调节位移比例、手臂平移/腕部旋转速度、灵巧手行程比例和跟随速度。若安装位置或操作者站位不同，应在 [teleop.yaml](config/teleop.yaml) 中完成现场标定后再用于真机。

`arm-teleop` 默认未开启。按 `E` 开始遥操，再按 `E` 暂停，再按 `E` 继续；按空格直接回到
截图所示的 `Ready` 初始姿态（J1～J6=`[-0.0305,-0.929,-1.61,-1.54,1.56,0]`）并解除遥操，之后必须再次按 `E` 才会运动。暂停时保持当前目标，继续时以当前手腕和
当前机器人位姿重新锚定，因此不会因人手回到舒适位置而让机械臂跳变。`Esc` 或红色按钮触发急停锁存，
Quest 数据超时、面板关闭或心跳丢失也会停止目标更新。

按空格会取消遥操、清空手指目标并把仿真 FR5 回到 `ARM_HOME` 就绪姿态；它不会给真实 FR5
发送运动指令。当前末端映射使用“锚点绝对目标 + 速度限幅”，不会把同一个
Quest 位移/旋转增量重复累加，因此可减少末端漂移和腕部内外旋过冲；`config/teleop.yaml`
中的比例、速度和 ±45° 腕部旋转包络仍可按现场标定调整。

这里的映射参考点是安装在法兰上的 IH01 掌心中心（仿真 TCP），不是某一个
FR5 关节。建议这样标定：先按空格回到 Ready，让手腕保持中立，再按 `E` 建立锚点；
随后分别沿 Quest 的 X/Y/Z 轴移动约 10 cm。若方向不对，先改 `axis_matrix` 或对应的
左右手符号，方向正确后再根据 TCP 的实际位移调整 `translation_scale_mm_per_m`。
再分别测试腕部各轴约 ±20°，一次只修改一个 `orientation_axis_sign` 或旋转比例。
观察面板中的 XYZ/R/P/Yaw，仿真中稳定 TCP 误差控制在约 5～10 mm 后，再进入真机调试。

IH01 的侧向安装角属于固定法兰到手掌的机械变换：FR5 Ready 姿态保持 J6=0，当前仿真在
`flange_to_hand_sim_rpy_deg` 中加入顺时针 90° 安装偏角。更换转接板时应修改该固定变换，
不应通过改变 J6 的 Ready 角来补偿安装方向。

腕部姿态映射会处理 Quest→FR5 坐标镜像的手性，避免右手内旋被映射成外翻。现场测试时先让
手掌保持中立并按 `E` 建立锚点，然后依次只做“向上翻腕、左右侧翻、内旋/外旋”，每次只动一个
方向，并观察面板中的末端目标 `R/P/Yaw`。当前侧装姿态的默认旋转符号为
`orientation_axis_sign: [1, -1, -1]`，数组顺序是 FR5 的 `[Roll, Pitch, Yaw]`，
不是 Quest 的红绿蓝轴；其中第一个值负责掌心内旋/外旋对应的 FR5 Roll，第三个值负责 Yaw；
如果重新安装后只有某一个轴反向，只改该轴为相反符号。
先把腕部旋转比例设为 `0.4～0.6`、速度设为
`30～60 deg/s`；若某一个目标轴仍与安装方向相反，只修改
`orientation_axis_sign: [R, P, Yaw]` 中对应项为 `-1`，重启 `make arm-teleop` 后复测，
不要同时修改多个符号。

注意：FR5 的 J2/J3 在机械结构上分别承担肩部和肘部运动，但当前 Quest 应用只发送手腕
六维位姿与 21 个手部关键点，并没有发送人体肩、肘姿态。因此系统采用“手腕末端目标 →
FR5 六轴 IK”，J2/J3 会为完成末端目标而联动，不是与人体肩肘角度一一相等。若要做真正的
肩肘姿态模仿，需要在 Quest 应用中启用上半身追踪并扩展协议，再加入人体到 FR5 的姿态代价、
关节限位和自碰撞约束；不建议直接复制人体关节角。

## Quest 3 首次安装（新头显或换电脑）

`make install` 才会把 APK 安装到 Quest 3；`make reverse` 不安装程序，只建立电脑与头显之间的 `localhost:8000` 通道。首次使用请在手机 Meta Horizon App 开启开发者模式，用 USB-C 连接并解锁 Quest 3，在头显弹窗中允许 USB 调试并勾选始终允许。

```bash
cd fairino-fr5-vr/IPE-quest-hand-teleop
make status      # 检查 adb 授权，必须显示 device
make install     # 安装 hand_tracking_streamer.apk
make reverse     # 建立 TCP 端口转发，不会再次安装 APK
```

为避免关闭头显中正在运行的程序，`make install` 检测到同包名应用已安装时会跳过
`adb install`。只有重新构建 APK 后确实要替换时才使用 `FORCE_INSTALL=1 make install`；
替换运行中的 APK 可能会重启该应用。日常执行 `make hand-teleop` 不会主动关闭或启动 Quest 页面。

看到 `Success` 和 `PASS: installed ... APK` 即表示安装完成；看到 `PASS: Quest localhost:8000 -> PC localhost:8000` 即表示通道建立成功。换另一台 Quest 3 时重复上述安装步骤即可。

如果修改了 Unity 工程，需要重新生成 APK：

```bash
make unity-build   # 仅用 Unity 构建并更新本地 APK
make install       # 将新 APK 安装到 Quest 3
make reverse       # 建立 localhost:8000 通道
```

`make unity-build` 需要本机安装 Unity 6；构建完成后再执行 `make install`。当前应用包名为
`com.haoming.ipe.handteleop`，无需手动配置。

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
make hand-teleop  # 启动电脑端 Quest + 实体 IH01 接收窗口
make ui            # 图形控制台与实时数据（启动前验证一次 sudo）
make hand-control  # 保留的 IH01 实体手动控制台
```

`make arm-sim` 默认不再自动挪动，窗口中按 `q/a`、`w/s`、`e/d`、`r/f`、`t/g`、`y/h` 正反转动 J1～J6，`0` 回到初始姿态，`o/c` 控制手指开合，`Space` 暂停/继续。终端每 0.5 秒显示六轴角度和 IH01 末端位置。`make arm-teleop` 的 MuJoCo 窗口显示机器人结果，Quest 视角直接在头显中查看，独立面板负责 E 开始/暂停、空格回到 Ready、急停、映射切换和参数调节。

`arm-teleop` 不再启动 `scrcpy`，直接以头显画面为准；这样不会受到 Quest 沉浸式合成层镜像黑屏影响，
电脑端 MuJoCo 仿真和遥操面板独立运行。

联合遥操仿真地面位于 `z=0` 附近，抓取区包含两个并排的窄方形矿泉水瓶（蓝色、绿色，含瓶身、瓶颈和瓶盖）以及
黄色长方体；它们都是带质量、摩擦和重力的动态物体。遥操面板可选择物体并用 `X± / Y± / Z±` 每次移动 20 mm。
`arm-teleop` 始终保留主窗口的外部视角，并在 MuJoCo 主画面右下角固定叠加 IH01 手心摄像头画中画，无需切换视角。
MuJoCo 窗口也支持 `v/b/n` 选择蓝瓶、绿瓶、黄盒，`i/k`、`j/l`、`u/m` 分别微调 X、Y、Z。灵巧手掌部、指节和 FR5 网格参与
MuJoCo 接触求解，可用末端接近物体后闭合手指进行推/夹/抬测试。正常的手指—物体接触允许继续运动；
遥操 IK 仅在 FR5 手臂与灵巧手或抓取物发生明显穿透时拒绝目标并保持上一姿态。`make arm-sim` 会隐藏这些物体，只保留机械臂手动检查画面。

灵巧手遥操统一使用 `make hand-teleop`，启动后进入 IH01 仿真镜像并选择右手、左手或双手；
仿真会限制在合理显示工作空间内（尤其避免落到地面以下）。窗口中的 `E` 键才会请求启用
实体 IH01 输出；未扫描到实体手时 E 只显示原因，仿真不会退出。`make hand-control` 则仍是实体 IH01 手动控制命令。

需要把 Quest 遥操接到实体 IH01 时，使用明确的硬件入口 `make hand-teleop`，启动时可选择
右手、左手或双手。该命令只检查 Quest、建立 USB 通道并打开接收窗口，不会自动安装、启动或关闭 Quest
应用；请在头显应用库中手动打开应用，并选择 TCP Wired / localhost / 8000。窗口启动后默认未启用输出，按 `E` 才会启用/停用目标发送；`Space`
暂停，`R` 清故障，`Q` 退出。该入口会经过 EtherCAT 与人工确认流程；未接实体手时，按
`E` 会在界面显示未检测到/未满足条件的实体 IH01，并保持仿真运行。进入后可用 HAND MODE 控件或
`l`、`r`、`b` 切换仿真显示手。

遥操窗口默认使用固定融合几何映射（关节弯曲角 + 指尖收拢程度）。需要适配操作者手型时，
先保持手掌完全张开约 0.5 秒并按 `O`，再保持自然握拳约 0.5 秒并按 `F`。标题中的
`calibration R:6/6` 或 `L:6/6` 表示六个通道均已有效标定；少于 6 的通道会自动回退到固定映射。
按 `K` 清除当前选择手的个人标定。OK 手势映射更新后，旧版标定会自动忽略，请重新执行一次
张开/握拳标定。标定保存在本机
`IPE-quest-hand-teleop/calibration/quest_hand_personal.json`，不会提交 Git；建议先在未按 `E`
的仿真状态完成标定并检查目标条，再启用实体输出。

握拳判定以四根长手指为依据：至少三根明显弯曲且整体达到阈值后，系统会把六个 IH01 通道
统一输出满行程，即使拇指被手指遮挡也不会落后；检测到拇指明显伸展（如竖大拇指）时不会
触发该覆盖规则，OK 手势也不会触发。

主 UI 的“急停”按钮会中断整个当前进程组，停止 Quest 接收、仿真和实体输出；切换模式时
Quest 应用保持运行，只重启本地接收/仿真进程。

运行 `make ui` 时会先清除旧的 sudo 缓存，并在终端请求一次密码，验证成功后才打开图形界面；UI 会在后台
维持本次终端凭据。实体灵巧手子进程保留同一个控制终端，因此不会在窗口内再次弹出密码框。

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
  使用的六维末端位姿数值逆解。
- FR5 外观现在只加载 FAIRINO 官方 V6 白色网格，不叠加外凸的自绘关节盖；其碰撞网格和运动学限位均以官方 URDF 为准。
- 已提供但尚未在本项目中完成现场调试：FAIRINO SDK 输出适配器。
- IH01 真机使用独立硬件流程；启动后保持未解锁状态，需要检查 EtherCAT OP、WKC 和
  故障状态，并由操作员按下 `E` 才能解锁。
- 仿真碰撞和穿透保护仅适用于 MuJoCo 模型；当前不声明已经具备真实 FR5 的碰撞规避、
  经过认证的功能安全、完成标定的 Quest 到机械臂坐标变换，或已经完成的实体遥操作试验。

第三方来源和许可证边界见 [docs/PROVENANCE.md](docs/PROVENANCE.md)。
FR5 代码与参数审计见 [docs/FR5_AUDIT_CN.md](docs/FR5_AUDIT_CN.md)。
