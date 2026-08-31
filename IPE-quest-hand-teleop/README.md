# IPE Quest Hand Teleop

可移植的 Quest 3 手势遥操应用。它保留 Hand Tracking Streamer 的 OpenXR 手部数据协议，
当前随仓库提供的 APK 使用上游应用身份 `com.wengmister.handtrackingstreamer`；Quest 端、TCP 接收器、MuJoCo
和 IH01 EtherCAT 后端都在本目录内。作者：haoming。

本目录与 `../quest3-hand-tracking/` 分开维护。复制本目录到另一台 Linux 电脑后重新执行
`make bootstrap` 即可生成本地 Python 环境；不会依赖父仓库的 `build/` 或 `.venv/`。

## 目录

```text
hand_tracking_streamer/  Unity Quest 源工程与 APK 构建入口
ih01_bridge/             HTS TCP landmarks -> IH01 六通道适配器
runtime/ih01_runtime/    MuJoCo 仿真、映射和硬件桥接
simulation/ + config/    IH01 双手模型与运行配置
hardware/ethercat/       bundled SOEM/IH01 后端源码
scripts/                 环境、ADB、构建和清理工具
third_party/SOEM/        随项目携带的 SOEM 源码
```

`Library/`、`.venv/`、`build/`、APK 和日志都是可再生成的本地文件，不提交 Git。硬件模式
另要求 EtherCAT 网卡权限、48 V 电源和独立完成安全检查。

## 一次性准备与 Quest 3 安装

```bash
cd experiments/IPE-quest-hand-teleop
make bootstrap
make check
make unity-build       # Unity 6.5，生成自己的 APK
make install           # 将 APK 安装到 Quest 3
make reverse           # 仅建立 localhost:8000 TCP 转发
```

首次使用新 Quest 3：在手机 Meta Horizon App 开启开发者模式，用 USB-C 连接并解锁头显，在头显中允许 USB 调试。先运行 `make status`，必须看到设备状态为 `device`；`make install` 输出 `Success`/`PASS: installed ... APK` 才表示 APK 已装入头显，`make reverse` 输出 `PASS: Quest localhost:8000 -> PC localhost:8000` 才表示连接通道已建立。更换头显后重复这三个命令即可。

Quest 应用内选择 Left、Right 或 Both。电脑端 `SIDE` 只使用 `left`、`right`、`both` 三个
短值；不指定时，有线/无线入口会在启动时交互选择。

## 运行方式

### 有线 Quest + MuJoCo（推荐）

USB 连接 Quest、解锁并允许 USB 调试后：

```bash
make wired-sim                 # 启动时选择 left/right/both
```

该命令依次安装 APK、建立 ADB reverse、启动应用，再打开 MuJoCo 接收器；不发送 EtherCAT
命令。Quest 网络设置为 **TCP Wired / localhost / 8000**。需要分步调试时使用：
`make install`、`make reverse`、`make launch`、`make sim SIDE=left`。

仿真窗口中的 IH01 手腕会跟随 Quest 的首帧锚定后位置和姿态移动，手指继续由 21 个关键点
重定向；这只影响 MuJoCo 仿真，不改变实体 IH01 的固定安装和硬件控制路径。

### 无线 Quest + MuJoCo

无线 ADB 需要先通过 USB 切换一次。Quest 解锁并接受 USB 调试后，查到 Quest 的局域网 IP，
只在首次配置或无线 ADB 失效后执行：

```bash
make wireless-connect QUEST_IP=192.168.1.42
```

该命令执行 `adb tcpip 5555` 和 `adb connect`，因此执行时必须保持 USB 连接。确认输出
`PASS: Quest ADB Wi-Fi target ...` 后可以拔掉 USB。之后每次使用无线连接时直接执行：

```bash
adb connect 192.168.1.42:5555
adb devices -l                 # 应显示 192.168.1.42:5555 device
make wireless-sim SIDE=both HOST=0.0.0.0
```

如果 Quest 重启、切换 Wi-Fi 或 `adb devices` 不再显示 `device`，重新插 USB 并再次运行
`make wireless-connect QUEST_IP=...`。`QUEST_IP` 是头显地址，不是电脑地址；应用的 TCP
主机填写电脑局域网 IPv4，端口为 `8000`。

Quest 应用选择 **TCP Wireless**，主机填写电脑局域网 IPv4、端口 `8000`；电脑端：

```bash
make wireless-sim SIDE=both HOST=0.0.0.0
```

无线模式需放行电脑防火墙 TCP 8000。只接收数据而不显示仿真可用
`make receive SIDE=both`。

### Quest + MuJoCo + 实体 IH01

实体模式当前一次只控制一只手。由于左右手从站身份相同，入口会询问物理手的方向：

```bash
make wired-hardware
```

启动后默认为 `DISARMED`。确认 EtherCAT 为 `OP`、WKC 正常且反馈数值持续更新后按 `E`
启用目标发送；再次按 `E` 可停用。
实体模式不能使用 `SIDE=both`。窗口快捷键：`q`/`Esc` 退出，空格暂停，`r` 清故障。
面板会同时显示视觉目标和实体反馈（位置、电流、力、温度、故障码）。

面板底部提供两条可直接拖动的速度控制：`FOLLOW SPEED`（1～100 steps/frame）调整视觉
目标每次更新的最大步进，`MOTOR SPEED`（1～2000 steps/s）调整厂商电机速度阶段；标题会
实时显示当前值与最大值。电机速度也可用启动参数设置：
`IH01_TELEOP_SPEED_STEPS_S=1000 make wired-hardware SIDE=left`。

默认固定映射融合关节弯曲角和指尖向指根/掌心的收拢程度。个人标定不依赖 V2：保持手掌
完全张开约 0.5 秒后按 `O`，保持自然握拳约 0.5 秒后按 `F`。界面显示 `R:6/6` 或
`L:6/6` 代表六通道有效；无效通道自动使用固定映射。`K` 清除当前手标定。标定文件仅保存
在本机 `calibration/quest_hand_personal.json` 并被 Git 忽略。应在 `DISARMED` 仿真状态完成
标定和目标条检查，再按 `E` 启用实体输出。

### 手动控制台（不依赖 Quest）

```bash
make hand-control
```

启动时选择左手、右手或双手；双手模式会询问两个不同的从站号，也可预先设置
`IH01_LEFT_SLAVE` 与 `IH01_RIGHT_SLAVE`。窗口提供六路滑块、`o` 全开、`c` 全闭、`r`
清故障、空格发送 CW0 暂停，以及电机速度滑块（1～2000 steps/s）。清故障仅执行设备规定
的复位序列，不绕过急停或硬件保护。

hand-control 默认使用逐通道接触保护：电流达到 1000 mA 或堵转持续 200 ms 后保持该通道，
对应目标松开/回退即释放。拇指–食指耦合软限位不参与该路径，避免额外限制遥操动作。

默认会在启动时提示选择左手、右手或双手；左右手外壳和从站编号无法由 EtherCAT 自动区分，
选择只是在控制台中建立逻辑映射，请按实际接线填写从站号。需要脚本化运行时仍可用 `SIDE`
环境变量跳过提示。

## 维护与排错

```bash
make status                 # ADB 授权状态
make provenance             # 上游版本和许可边界
make build-backend          # 仅本地编译 SOEM，不打开网卡
make clean-local            # 清理可再生成文件
```

如果出现 `no authorized Quest`，先解锁头显并接受 USB 调试提示；有线连接要重新执行
`make reverse`。如果无线无法连接，确认 Quest 与电脑在同一局域网、电脑防火墙允许 TCP 8000，
并重新运行 `make wireless-connect QUEST_IP=...`。

## 上游与许可

详见 [UPSTREAM.md](UPSTREAM.md)、[LICENSE](LICENSE) 和 [CONNECTIONS.md](CONNECTIONS.md)。
上游 Unity 文件保持独立；IH01 修改集中在 `ih01_bridge/`、`runtime/` 和 `hardware/`。
