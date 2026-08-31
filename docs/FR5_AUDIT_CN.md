# FR5 代码、参数与组合仿真审计

审计日期：2026-08-28。

## 结论

原项目没有 FR5 机器人仿真。此前的 `make demo` 只计算 Quest 到 FR5/IH01 的数值目标；
`IPE-quest-hand-teleop/simulation/ih01_x1_dual.xml` 是两只 IH01 的固定模型，现有 hand-only
入口通过仿真配置将其 wrist mocap 改为跟随 Quest，实体 IH01 安装仍保持固定。

现在新增 `simulation/fr5_ih01_right.xml`：FR5 有六个旋转关节，IH01-X1-R 作为固定工具
安装在 `wrist3_link` 下。模型共有 18 个广义坐标、12 个主动执行器，并带有 Quest 手腕
平移到 FR5 数值逆解和六通道灵巧手映射。

## 已发现并修复的问题

1. XML-RPC 降级连接曾把 `GetSafetyCode()` 固定返回 0，在 CNDE 实时状态缺失时形成
   fail-open。现在必须从控制器读取急停和两路安全停止状态；读取失败返回 199 并阻止使能。
2. 完整 CNDE 路径的 SDK 安全检查只看两路 safety stop，没有纳入 EmergencyStop。项目层
   现在同时检查三者。
3. 遥操适配器曾对连续目标调用 `MoveCart`。现在改为
   `ServoMoveStart -> ServoCart -> ServoMoveEnd` 生命周期；该路径仍不开放为默认真机入口。
4. 默认配置曾主动跳过 CNDE 20005。真机配置现在要求 CNDE + XML-RPC 完整连接，不允许
   只凭 20003 进行遥操。
5. 原配置没有 IH01 与转接板的质量、质心和 TCP。现已加入显式投产门控；未测量、未审批
   时 FR5 适配器会在联网前拒绝运行。
6. 原项目没有机械臂模型。现有组合模型复用 IH01-X1-R 详细网格和关节树，并引入
   FAIRINO 官方 FR5 V6 `base/shoulder/upperarm/forearm/wrist1/2/3_link.STL` 网格；关节原点、轴和限位来自官方 URDF。FR5 网格作为外观，碰撞仍使用保守解析几何以保持仿真实时性。

## 官方数据核对

- FR5 额定负载 5 kg、最大 7 kg、臂展 922 mm、重复定位精度 ±0.02 mm。
- 六轴范围为：J1 ±175°、J2 -265°~+85°、J3 ±160°、J4 -265°~+85°、
  J5 ±175°、J6 ±175°；项目中的硬限位与官方数据一致。
- 官方产品页给出的各轴最大速度是 180°/s。项目以 180°/s 作为保守配置值。
- 官方 SDK 规定 `ServoJ/ServoCart` 需要配合 `ServoMoveStart/ServoMoveEnd`；官方示例和
  当前捆绑 Python SDK 均使用 0.008 s 的 `cmdT` 示例值。
- 官方 SDK 提供 `SetLoadWeight`、`SetLoadCoord` 和 `SetToolCoord`。这些值必须来自实际
  IH01、线缆和转接板总成，不能从额定负载或仿真模型猜测。

官方参考：

- [FAIRINO FR5 产品页](https://www.fairino.com/FR/4.html)
- [FAIRINO 官方 ROS 2 仓库与 FR5 URDF](https://github.com/FAIR-INNOVATION/frcobot_ros2/blob/main/fairino_description/urdf/fairino5_v6.urdf)
- [FAIRINO SDK 运动接口说明](https://fairino-doc-en.readthedocs.io/3.6.7/SDKManual/CPPRobotMovement.html)
- [FAIRINO SDK 工具、负载和质心设置说明](https://fairino-doc-en.readthedocs.io/3.7.8/SDKManual/C%23RobotCommonSettings.html)

## 仍需实物确认

以下参数不能从网页可靠推断，因此仍保持未批准状态：

- IH01-X1-R 本体、安装法兰、线缆随动部分的总质量；
- 总成质心在 FR5 法兰坐标系中的 `[x, y, z]`；
- 法兰到实际抓取 TCP 的六维变换；
- 转接板孔位、刚度和允许力矩；
- IH01 供电、EtherCAT 线缆走线和机械臂全姿态下的拉扯/干涉；
- FR5 与手部、环境之间的 CAD 级自碰撞和场景碰撞模型。

配置里的 `flange_to_hand_sim_*` 只用于看模型，不能直接写入控制器。取得转接板 CAD 和
称重/质心数据后，应先更新组合模型，再完成回放、仿真、低速空载和受控抓取验证。
