# UR5 + ZED Mini 垃圾分类：2026-10-22 接续节点

当前目标：在 ZED 左目原始图像中，标注、保存并检查三格工作区。机器人及夹爪执行均禁用；当前没有机器人网络连接代码。

## 开机后先做

在 PowerShell 中运行，无须激活虚拟环境：

```powershell
Set-Location D:\UR5_Waste_Sorting
.\.venv\Scripts\python.exe -m tools.startup_check
```

如果显示 `no valid serial number` / 可用设备列表为空：重新插拔相机 USB，关闭 ZED Explorer、Depth Viewer 及其他相机程序，再运行检查。设备枚举恢复后可读一帧（不弹窗，不连接机器人）：

```powershell
.\.venv\Scripts\python.exe -m tools.startup_check --frame
```

## 实时标注

```powershell
.\.venv\Scripts\python.exe workspace_zones.py
```

1. 调整并固定相机，使三格框和橙点附近抓取区都可见。相机移动后须重新验证区域及外参。
2. 按空格冻结画面。按 `1`，根据**你在现场定义的物理最左格**选区，不按图像 X 推断。沿格子内侧边界顺序点击四角，按 Enter 提交，然后按 `Y` 明确确认身份。绿色为可回收。
3. 按 `2` 标注中间格，Enter、Y。灰色、unused。
4. 按 `3` 标注物理最右格，Enter、Y。红色为不可回收。
5. 按 `4` 标注橙点附近 PICK_ZONE，Enter、Y。区域不能重叠。建议留边，避开框线及不确定边界。
6. 空格恢复实时画面，检查全部叠加与桌面相符，再按 `V` 确认本次相机位置。此确认仅针对二维区域，不代表标定、TCP或可达性已验证。
7. 按 `S` 保存到 `config/workspace_zones.json`。`L` 重新加载会清除本次相机位置确认，需要检查后再按 V。
8. Tab 切换检查模式；点击物体中心，终端显示原始像素和所属区域。`+/-` 改变显示比例；`U` 撤销草稿顶点；`R` 清空当前格；`Q` 退出。退出不会自动保存。

窗口固定使用 AUTOSIZE，程序主动缩放显示并反算鼠标位置，JSON 始终保存原图像素。若画面超出屏幕，用 `-` 缩小。原始分辨率变化时拒绝加载，不自动缩放旧区域。

```powershell
.\.venv\Scripts\python.exe workspace_zones.py --check
```

每次启动/加载，`camera_pose_revalidated` 都重置为 false，即使上次 JSON 曾保存 true。同一序列号也不能证明三脚架未移动；人工实时检查必不可少。离线图像禁止按 V 完成当前相机位置确认。

## 没接相机时练习（旧照片不代表今天的位置）

使用独立练习配置，避免覆盖实时配置：

```powershell
.\.venv\Scripts\python.exe workspace_zones.py --image data\captures\zed_20261021_135850_033340_rgb.png --config config\workspace_zones.practice.json
```

物理格子身份仍需现场确认，不能凭历史照片推断。操作提示显示在相机图像下方，不遮挡原始画面，点击提示栏不产生区域顶点。

## 分类和后续集成

`config/sorting_policy.json` 定义：plastic_bottle / aluminum_can → recyclable → LEFT_CELL；sponge / toothbrush → residual → RIGHT_CELL。置信度阈值暂设 0.80，需结合队友模型实测调节。中格和旧无分隔框不参与投放；扫描原项目没有发现可删除的旧投放配置。

`classify_detection()` 输入队友检测字典（class_name、confidence、bbox、center_pixel），输出类别、目标区域和状态。`ready_for_localization` 仅表示可进入定位，不是运动许可。

`assess_target()` 进一步检查区域完整性、物理身份、本次相机位置、图像范围、PICK_ZONE归属和相机深度。未知类/低置信度明确拒绝；缺深度等待；即使有合法相机 XYZ，仍返回 `waiting_base_calibration`。不会生成机器人 Base 坐标。

`waste_sorting/interfaces.py` 保留检测、相机定位、Base 变换、规划及夹爪的协议。当前只有 `DryRunRobot`：输出任务步骤预览、状态和日志，轨迹为 null，执行及夹爪命令始终抛出拒绝异常。没有已实现的自动深度定位、外参转换或运动规划。队友交付检测 JSON 列表后，可以 `--detections 文件路径` 叠加检测中心和处理状态；不会自动加载或下载 YOLO。

区域的 `robot_target` 存储字段预留了 Base目标、有效区域、接近高度、姿态、可达性、碰撞检查和人工验证。真实参数未知均为 null/false，二维中心只是多边形中心，不是投放 TCP。

## 无硬件验证

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -m compileall -q waste_sorting tools tests workspace_zones.py
```

只从 tests 目录发现单元测试。根目录原有 `test_zed.py` 在导入时会开相机，不应纳入无硬件测试。

2026-10-22 本次实际验证：15项单元测试全部通过；上述 compileall 语法检查通过；历史RGB图像的无窗口渲染通过，输出 `data/workspace_overlay_check.png`，已人工查看生成图像。测试首次因沙箱系统临时目录权限失败，获准在沙箱外重跑后全部通过。SDK 导入及设备枚举已执行，SDK 报设备无有效序列号、可用列表为空；未通过实时RGB/深度、交互式鼠标标注、现场身份确认或机器人测试。`--frame` 是待相机恢复后的现场命令。

## 已检查的项目现状

- 保留原文件：test_zed.py、zed_visualizer.py、zed_capture_reference_points.py、get_python_api.py。
- 保存的采集包含 RGB、depth.npy、xyz.npy 和元数据。
- 原始参考点：calibration/sessions/20261021_140123/camera_points.json / csv，8点，每点20有效帧。开机检查直接读取该 JSON 并计算平面残差，只作历史数据诊断。
- 未找到原有 fit_table_plane.py、YOLO代码/权重、机器人配对点、外参矩阵、TCP验证或真实运动接口。
- 当前目录不是 Git 仓库。

## 本次新增完整文件

- workspace_zones.py：实时/离线标注、加载及区域检查入口。
- waste_sorting/__init__.py、zones.py、camera.py：模块入口、区域几何/配置、复用已有参数的ZED输入。
- waste_sorting/decision.py、interfaces.py：分类、等待/拒绝状态和禁用执行的集成接口。
- config/sorting_policy.json、workspace_zones.json：新版映射及空白工作区模板。模板无真实多边形，不可视为已配置。
- tools/startup_check.py：环境、历史点数据与相机枚举/单帧检查。
- tests/test_workspace.py：纯模拟像素的行为验证。
- README.md：操作及交接说明。
- data/workspace_overlay_check.png：本次离线渲染验证图，无真实区域标注，不代表今天相机位姿。

代码已直接写入对应文件，以上每个文件均包含完整内容，无须手工拼接补丁。旧采集代码和原始数据没有覆盖。

## PICK_ZONE 三维表面诊断（新增）

现场已完成四区标注、身份确认、保存/加载及点击归属检查。项目中的 `config/workspace_zones.json` 现为用户保存的真实像素区域，已读取检查，未重写；序列号为 16343130。这一进度来自用户现场反馈及当前文件，不代表已完成机器人标定。

先在旧标注窗口按 Q 退出，释放相机，再运行：

```powershell
.\.venv\Scripts\python.exe pick_depth_check.py
```

1. 保持相机固定，在实时画面检查区域叠加，按 V 确认本次相机位置。本工具不会修改工作区配置。
2. 先清空 PICK_ZONE，点击内部一处桌面，按 M，保持相机和桌面不动。自动采集20个新帧并保存诊断。
3. 放入一个测试物体（可先用海绵），点击其可见表面中央，避开轮廓和高光，按 M 再测一次。不要把读数解释为物体质心或抓取TCP。
4. 可连续点击、测量。C 清空选择，+/- 缩放，Q 退出；未完成的测量丢弃。

每帧从同一次成功 grab 获取 RGB 与 XYZ。使用点击像素自身的XYZ；中心无效时不会替换成附近桌面。5×5窗口须位于PICK_ZONE内部，有效深度比例至少60%，局部Z的P90-P10不超过20mm，才算有效帧。20帧至少12帧有效，XYZ各轴时间标准差不超过10mm时报告 `camera_measurement_stable`。这些是初始诊断阈值，既不保证绝对精度，也不能证明采到的是物体或适合抓取。

自动保存 `data/localization_checks/<时间戳>/measurement.json`、原始RGB和标记图。JSON包含20帧诊断、失败计数、阈值、像素、相机XYZ中位数/时间标准差，以及本次配置快照和原文件SHA256。快照是本次人工确认后的状态，hash对应磁盘原文件；机器人Base坐标始终为空，运动始终禁用。

新增完整文件：`pick_depth_check.py`、`waste_sorting/localization.py`、`tests/test_localization.py`。修改：`waste_sorting/camera.py`（新增同步XYZ读取，原read接口保留）、`README.md`。这批改动没有覆盖工作区配置或原始标定数据。

本批实际验证：24项无硬件单元测试全部通过（包含之前15项），覆盖无效中心拒绝、邻域缺深度/深度跳变拒绝、多帧稳定性判断、RGB/XYZ同次采集和断流有限重试；compileall及命令行help通过。已读取真实区域文件并通过几何校验。新增工具的实时深度与鼠标操作尚待用户现场运行，未主动开启相机或连接机器人。

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -m compileall -q waste_sorting tests pick_depth_check.py
```

## 固定工具特征点的深度可行性检查

现场没有探针、打印机或标定板，目前仅评估夹爪固定机身上自然特征点能否提供可靠的相机深度。用户最新ZED画面右上可见OnRobot固定机身及蓝色标志；截图本身不作为原始像素坐标来源，也不能证明该点在多姿态下可重复定位。

先在标注窗口按Q关闭，再运行：

```powershell
.\.venv\Scripts\python.exe pick_depth_check.py --tool-point
```

检查实时区域叠加仍与桌面相符后按V。在实时原图中点击固定机身蓝色圆形标志的几何中心，轻按一次M后松开，保持机器人和相机静止。完成20帧后，重新点击同一物理位置，再按M重复一次。只测试深度有效性、时间稳定性及重复点击的差异，不采集机器人运动、不生成相机外参。

该显式模式允许选择PICK_ZONE外的像素，默认抓取区诊断仍严格限制PICK_ZONE；分类模块未改动。鼠标按显示比例反算原始相机像素，边缘窗口、无效中心及深度质量检查保留。输出到独立的data/tool_point_checks目录，记录type=unpaired_tool_point_diagnostic、calibration_pair=false、rigid_point_identity_confirmed=false、base_xyz_m=null。即使报告camera_measurement_stable，也不证明特征点身份、刚性、绝对准确度或外参可用。

本批修改完整文件：pick_depth_check.py、waste_sorting/localization.py、tests/test_localization.py、README.md。区域配置和机器人执行禁用设置没有修改。

## 工具点与机器人位姿的候选关联记录

已读取用户两次真实工具点诊断：20261022_134339_815708的像素[962,65]，以及20261022_134437_050659的像素[962,67]。两组均20/20有效、邻域有效比例均1.0；三维坐标差约[0.1455,1.9662,0.3317]mm，距离1.9993mm。标记图中选点位于夹爪机身标志；两像素的重复点击偏差说明帧内标准差不能代表物理点重定位精度。此结果尚未证明自然特征点可用于可靠外参标定。用户已确认现场有授权人员可确认并操作后续姿态调整；本阶段仍只做静态检查。

先保持机械臂和相机不动，退出其他ZED程序，并确认电脑网线仍直连机器人。运行：

```powershell
.\.venv\Scripts\python.exe pick_depth_check.py --tool-point --pair-robot
```

检查区域叠加后按V，重新点击同一蓝色标志几何中心，轻按一次M后松开。程序读一次采样前机器人状态，再采20帧相机深度，最后读一次采样后机器人状态。前后TCP配置变化、位姿变化、控制器重启/时间异常、时序不正确或总区间超过10秒均拒绝候选。端点法兰位置差限1mm、旋转差限0.1度、关节变化限0.1度。这些仅为候选数据质量阈值，不是安全运动许可，也不能排除两端之间曾移动后返回；全过程须由操作者保持机器人静止。

`waste_sorting/rtde_readonly.py`使用与MATLAB只读脚本相同的输出字段和RTDE v2协议。发包助手仅允许查询/输出订阅消息86、118、79、83；禁止输入recipe和数据写包，不访问脚本或Dashboard端口。每次快照超时3秒并关闭连接。没有电源、刹车、Play、运动或夹爪命令。此模式无需MATLAB同时运行。

输出到data/tool_robot_observations/<时间戳>/measurement.json及RGB/标记图。记录包含原始相机像素和20帧测量、命名的feature_id（默认onrobot_body_logo_center）、前后原始机器人状态，以及T_base_from_flange。法兰位姿由T_base_from_tcp乘以T_flange_from_tcp的逆得到，撤销控制器已配置的TCP偏移，不把150mm假设为真实抓取点。特征点在法兰下的偏移仍未知，base_xyz_m为null，calibration_pair=false。通过时observation_status为candidate_observation_recorded，仍只是待离线评估的候选观测，不是外参矩阵或机器人目标。

本批完整新增文件：waste_sorting/rtde_readonly.py、waste_sorting/observations.py、tests/test_robot_observations.py。完整修改文件：pick_depth_check.py、README.md。38项无硬件单元测试全部通过，涵盖分片网络包、禁止控制消息、断流、错误字段、非有限数据、旋转后的TCP偏移撤销、等价旋转向量、配对时间和端点变化拒绝；compileall和CLI检查通过。真实Python快照尝试连接超时；随后ipconfig显示有线网卡Media disconnected，因此新增Python接口及相机/机器人联合采集尚未完成真机验证。恢复网线后再做一次静态记录，不应把既有MATLAB成功当成新增Python接口已验证。

## 自然工具点的离线候选外参分析（实验性）

用户已完成首次真实联合采集，记录data/tool_robot_observations/20261022_135537_884841/measurement.json已实际读取重检：20/20相机帧有效，采样前后法兰位置差0.00933mm、旋转差0.00287度、区间1.304秒；camera采样位于两次机器人状态之间。此结果证明新增Python输出订阅及静态候选记录在本机成功工作，仍不是外参验证。该次Elbow约-0.51度；后续姿态/路径必须由现场授权人员评估，不据此远程发送未验证的笛卡尔或关节运动。

新增完整文件：waste_sorting/point_calibration.py（几何约束/求解/数据审计）、analyze_tool_observations.py（离线入口）、tests/test_point_calibration.py（纯模拟真值与退化测试）。修改README.md。没有安装新依赖；算法只使用现有NumPy/OpenCV，没有接入真实执行器，也不写入运行时标定配置。

每个观测的几何约束是 R_base_camera * p_camera + t_base_camera = R_base_flange * o_flange + t_base_flange。同时求未知的相机外参和固定自然特征点的法兰偏移o_flange，不把标志点认作TCP。该实现是基于这一约束的实验性线性初始化、旋转矩阵约束细化和Huber加权，不声称复现论文算法或论文精度；参考背景https://arxiv.org/abs/2004.12611。

```powershell
.\.venv\Scripts\python.exe analyze_tool_observations.py
```

程序离线读取已有记录，重算相机统计及机器人端点比较。不同相机、工作区文件指纹、特征点ID或机器人混合时拒绝；位置差小于5mm且旋转差小于2度的重复姿态分为同组，不用于制造额外样本或留出验证。这里只是去重阈值，不是建议机器人移动的距离或角度。

当前策略至少需要8个不同训练姿态和3个未用于拟合的姿态。2026-10-22 已用 `--freeze-training` 将现有8条固定到 `data/calibration_analysis/training_manifest.json`，其中保存绝对路径和SHA256；该操作拒绝覆盖已有清单。正常运行分析命令会自动使用该清单，文件改变或缺失时拒绝分析。后续所有新且不同的姿态均作为留出数据，不会随记录增加转入训练集；未指定清单时默认首8条训练、其余留出。不能只在一个姿态重复M，也不能全程保持相同方向或只绕一个轴转动；实际运动必须独立评估、由授权人员操作。相机固定、同一机身特征点固定、每条记录整个采样期间机器人完全静止。若安全范围或可见性不能满足，停止该实验路线，不以凑数据为目的运动。

线性系统秩/条件数和局部9参数雅可比秩不足时拒绝。当前保守初始化也可能拒绝某些理论上可解的特殊配置。暂定诊断门槛为训练RMS<=5mm、留出RMS<=8mm且最大误差<=15mm；这些数值不是抓取精度要求或安全门槛，未通过真实参考物验证。即使所有检查通过，只输出candidate_fit_requires_independent_physical_validation，camera_to_base_calibrated和robot_execution_enabled始终false。

报告输出data/calibration_analysis/candidate_report.json。当前真实数据复核：records_seen=8、accepted_observations=8、distinct_robot_poses=8，固定训练8条、留出0条，状态waiting_more_distinct_poses，候选矩阵为null。单独的训练诊断RMS约4.60mm，只反映训练残差，不代表绝对定位精度。46项无硬件测试全部通过，包括模拟已知外参和未知工具偏移恢复、小噪声、留出错误拒绝、重复姿态、无旋转/单旋转轴退化、非法变换拒绝，以及训练清单不覆盖、数据篡改检测和新增留出记录不转入训练。尚无真实留出验证结果，不宣称标定通过。

2026-10-22 14:29 UTC 数据更新：共11条有效、不同姿态，无重复或拒绝。清单固定前8条训练，新增3条仅作留出检查。训练RMS=4.601mm，留出误差分别6.628/5.325/8.621mm，留出RMS=6.991mm，最大8.621mm，满足既定的实验诊断门槛。状态为candidate_fit_requires_independent_physical_validation；候选外参与未知机身特征点偏移仅存入离线报告，不写运行时配置。机身特征点偏移不是抓取TCP；现有150mm TCP偏移仍未经物理验证。camera_to_base_calibrated、independent_physical_validation_passed、robot_execution_enabled均为false。下一阶段需要独立物理参考检查及TCP核实，不能据此开始自动抓取。

用户选择暂缓TCP核实，继续只读候选Base坐标预览。关闭其他ZED窗口，然后运行：

```powershell
.\.venv\Scripts\python.exe pick_depth_check.py --candidate-base
```

该模式启动前从冻结训练清单和原始观测重算候选拟合，核对工作区SHA256和报告矩阵；来源不一致或留出检查不通过时拒绝启动。不能与--tool-point或--pair-robot组合，不连接机器人。检查画面区域对齐后按V，在PICK_ZONE内点击稳定可见的物体表面；底栏显示候选Base XYZ（毫米）。M保存20帧汇总（JSON坐标单位仍为米）到data/candidate_base_checks，包含原图、标记图、候选坐标、报告/清单指纹和候选变换。相机、桌面和物体保持不动；相机或安装关系改变后不能仅按V继续沿用旧外参，应重新采集核实。

候选坐标使用单独的candidate_base_xyz_m字段，原base_xyz_m仍为null。它仅表示表面点，不是抓取TCP位姿、路径或抓取目标；TCP、独立物理检查和执行标志仍为false，现有分类决策和执行禁用逻辑不变。新增waste_sorting/candidate_preview.py及tests/test_candidate_preview.py；50项无硬件测试通过，当前11条真实证据可离线重新核算。实时GUI需现场运行上述命令确认。

现场候选预览已运行成功。像素[848,169]海绵在位记录20261022_150733_908066与移除后桌面记录20261022_150943_346536均20/20有效，已查看标记图确认移除。候选Base Z差21.891mm，对照用户尺量约20mm，差1.891mm。记录于data/candidate_base_checks/sponge_height_check_20261022.json。仅为单位置相对高度粗核对；同一像素对应同一相机射线，并非同一Base XY，不能当作全局绝对误差或TCP验证。所有执行、标定完成及独立物理验证标志保持false。

### YOLO交接接入（2026-10-22）

`Vision_Module_Final.zip` 在本机实际是目录，内含best.pt（6505955字节）和队友原版detector.py。原文件保留；原版的模型路径指向队友D:\Waste_Vision_Dev\src\best.pt，因此新增根目录detector.py适配路径、延迟加载和模型锁，保持detect_waste(image_bgr)返回(JSON字符串, 原始尺寸BGR标记图)。沿用队友512x512拉伸和CPU推理，再映射回原始像素；没有把显示缩放后的坐标传给深度采样。权重元数据训练版本为Ultralytics8.4.183，类别aluminum_can/paper_cup/plastic_bottle/sponge。按交接约定新增paper_cup -> recyclable -> LEFT_CELL；原toothbrush策略保留，但当前模型没有该类。

依赖见requirements-vision.txt及constraints，保持原NumPy2.5.4和OpenCV5.0.0.93不变。官方安装说明：https://docs.ultralytics.com/quickstart/ 。不需要ROS或MATLAB External Control程序。

新增sorting_preview.py为实时入口，waste_sorting/vision_pipeline.py为可注入推理器的分阶段处理器：采集同一次grab的RGB/XYZ -> 推理 -> 严格检查JSON原始分辨率和条目格式 -> status=OK -> 本地类别/0.80置信度策略 -> PICK_ZONE及深度邻域检查 -> 候选Base表面点 -> 禁用执行的任务预览。队友默认0.45只用于识别结果展示，本地0.80门槛不被绕过。JSON中的waste_category不能直接决定投放区，类别与本地映射冲突时拒绝。塑料瓶/铝罐/纸杯进入LEFT_CELL，海绵进入RIGHT_CELL，MIDDLE_CELL不用。

像素三维定位直接使用ZED匹配点云xyz[v,u]，不另填或猜测内参；候选变换为p_base=R_base_camera*p_camera+t_base_camera，启动时复核已保存的原始证据。当前深度为单帧采样，不宣称20帧稳定性。检测框中心只对应可见表面，不保证在物体顶部或适合抓取；直立约定不能替代抓取中心和朝向估计。grasp_pose、trajectory、base_xyz_m均为null，execution_status=disabled，没有URScript、夹爪命令或机器人连接。未来接近/抓取/投放为task_outline占位步骤。

关闭其他ZED程序后运行：

```powershell
.\.venv\Scripts\python.exe sorting_preview.py
```

检查区域框后按V；放一个物体到PICK_ZONE，画面显示识别框、类别、分类区与通过检查的候选Base坐标。S保存当前原图、标记图、detector.json和preview.json到data/sorting_previews；Q退出，+/-缩放。按V只确认本次画面对齐，不完成外参或TCP验证。相机发生移动应重新核实标定。单相机单所有者顺序处理，整个结果绑定frame_id；未来移入视觉线程时应将RGB、XYZ和frame_id作为同一个不可拆分数据包传递，不可将旧检测结果套用新深度，也不能直接由预览队列驱动运动。

已有图片可离线推理，不打开相机、不创建虚假的深度或本次相机验证：

```powershell
.\.venv\Scripts\python.exe sorting_preview.py --image data/candidate_base_checks/20261022_150733_908066/rgb.png
```

离线模式保存检测与分类结果，通过识别过滤的条目定位仍停在waiting_camera_revalidation。tests/test_vision_pipeline.py覆盖四类路由、非OK/低置信度/未知类别/类别冲突拒绝、原始尺寸映射、空检测、PICK区域、无效中心深度、逐帧深度不复用及执行禁用。57项纯离线测试通过。

实际安装Ultralytics8.4.183、torch2.14.1+cpu、torchvision0.29.1后pip check通过，NumPy/OpenCV版本未变；同一进程已验证YOLO与pyzed.sl可同时导入，没有打开相机。真实best.pt SHA256为7b1237f82f5b84098520a89503a168cb1dd5f4a404217cce136193bbb400ce0b。对保存的海绵图运行离线入口成功，输出data/sorting_previews/20261022_153001_614529：海绵置信度0.552036、框[790,127,893,208]、中心[841,167]，因低于本地0.80门槛被拒绝；另有图像上方尺子附近aluminum_can误检0.269669，被队友0.45门槛拒绝。已查看标记图，未因演示需要降低门槛，不宣称识别100%或真实三维全流程已通过。下一步用户关闭现有ZED窗口，运行实时入口，在PICK区域摆放单个物体并S保存现场结果。

现场实时识别已运行，PICK内海绵记录20261022_154448_661658置信度0.562886，被默认0.80门槛拒绝。为继续只读定位对接，新增显式参数`--preview-min-confidence 0.45`，仅本次禁用执行的预览按队友接口0.45门槛处理；不修改config/sorting_policy.json的0.80值，assess_target仍用全局默认策略。status非OK、未知类别、类别冲突、PICK区域、深度和相机验证门槛继续生效，机器人执行始终禁用。启动输出、画面底栏及保存结果记录preview_min_confidence和preview_only范围；不把放宽后的预览当作可抓取判定。新增隔离测试，58项离线测试通过。

```powershell
.\.venv\Scripts\python.exe sorting_preview.py --preview-min-confidence 0.45
```

现场实时预览首条完整通过记录：data/sorting_previews/20261022_155211_797126/preview.json，frame489，sponge置信度0.671538，中心[829,113]位于PICK_ZONE；已查看标记图确认红点在海绵表面。类别residual、目标RIGHT_CELL，相机XYZ约[0.275549,-0.365836,1.072184]m，候选Base XYZ约[319.956,-215.196,541.289]mm；深度邻域有效比例1.0、Z跨度1.902mm，处理耗时约71.856ms。状态candidate_surface_preview_only，仅单帧候选表面点；execution_status仍disabled，TCP/独立物理验证/标定完成标志均false。其余三个模型类别尚需现场逐类预览检查。

后续四类单物体实时预览均已记录，汇总data/sorting_previews/four_class_live_check_20261022.json保存源记录路径/SHA256、置信度、目标区和候选坐标，四例使用同一工作区指纹及候选外参报告。海绵67.2% -> RIGHT_CELL，塑料瓶72.3%/铝罐87.8%/纸杯92.7% -> LEFT_CELL。纸杯候选Base坐标[335.818,-112.429,591.086]mm，铝罐[345.218,-98.486,645.156]mm。已查看四类标记图；纸杯红点投影在杯口内部，无法据图确认点云对应杯沿、内壁或杯底，不能当作抓取点。四例仅证明0.45门槛的预览链路可运行，未完成准确率评估、绝对定位、抓取TCP或真实动作验证；默认0.80仍会拒绝海绵和塑料瓶。下一项硬件工作先查看OnRobot Setup当前设备识别状态，再安排夹爪连接诊断。

### 夹爪现场控制方式更正

用户询问现场老师后，最终更正为：Tool Output数字0开启=关爪，取消/关闭数字0=开爪，仅使用0号通道。此前“0开1关”和“0开启为开爪”的说法均已被本条更正取代。I/O控制权保持User；截图为24V。OnRobot5.15.0通用插件识别不是当前现场手动控制的前提，停止以插件自动发现作为唯一排查路线。此前选择OnRobot后Error变为Empty，仍未识别设备；该变化不代表通信成功。配置停留No connection可停止之前工具口探测，后续按现场已有I/O控制方法开发。

新增config/gripper_io.json记录现场提供的映射，waste_sorting/gripper.py为纯意图预览器；新版视觉记录附带open/close的通道和电平，open_level=false、close_level=true。未实现网络发送，command始终拒绝执行，启动/退出均不写输出。老师提供的手动方法不等于电脑写输出已验证；software_execution_verified仍false，TCP和外参验证及运动执行标志不变。尚未由软件验证夹紧反馈、夹持力或到位时间，不将Tool Input1或模拟电压读数直接解释为抓取成功。

### 独立单次夹爪I/O测试

新增gripper_io_check.py、waste_sorting/gripper_test.py和tests/test_gripper_io_test.py；rtde_readonly.py增加独立工具I/O输出配方，原相机/机器人位姿配方保持不变。测试程序不被sorting_preview或DryRunRobot调用。默认open/close只保存离线预览，显式--send才允许一次0号输出写入。只读status命令订阅RTDE30004，无输出写入：

```powershell
.\.venv\Scripts\python.exe gripper_io_check.py status
.\.venv\Scripts\python.exe gripper_io_check.py open
```

现场授权人员确认夹爪空载、手及物品离开夹指范围、工具I/O由User控制，示教器程序停止后，才手动运行单次实际测试：

```powershell
.\.venv\Scripts\python.exe gripper_io_check.py open --send
# 观察实际开爪并检查输出后，再单独进行关爪测试：
.\.venv\Scripts\python.exe gripper_io_check.py close --send
```

发送前检查CB3 3.15版本、runtime_state=1（STOPPED）、程序运行位关闭、安全模式NORMAL/REDUCED、各关节速度绝对值<=0.01rad/s和现有24V工具供电。检查失败不发送、不解除安全停止、不修改供电或控制权。输出已是请求电平时不发送。发送严格固定sec secondary程序，仅包含set_tool_digital_out(0, False/True)，经30002发送一次，无关节/TCP运动命令；只读RTDE的actual_digital_output_bits第16位读取工具输出0，第17位读取工具输出1。最多三次读回核对，不重发写命令、不在退出或错误时重置夹爪、不改变工具输出1。读回匹配仅证明输出电平，不能证明夹爪到位或物体夹住；关节前后端点比较也不能证明期间始终未动。写入或读回异常时状态可能未知，应现场查看，不盲目重跑。全部记录保存data/gripper_io_tests；保持视觉及自动分类执行禁用、软件开合验证标志false。

67项纯离线测试通过。已运行open默认离线预览，固定脚本是set_tool_digital_out(0, False)，没有发送。实际只读status已成功（20261022_170717_365227/result.json）：控制器3.15.5.0、程序停止、六关节速度0、安全模式1、供电24V，工具输出0=true、输出1=true。这些是当次输出电平读数，不是实际爪位确认。实际--send尚未运行，待用户在现场进行第一次开爪验证。

协议依据：[UR工具数字输出](https://www.universal-robots.com/manuals/EN/HTML/SW5_20/Content/prod-scriptmanual/G5/set_tool_digital_outnb.htm)、[UR Secondary Program](https://www.universal-robots.com/articles/ur/programming/secondary-program/)、[RTDE字段与工具输出位定义](https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html)。函数文档页面为e-Series版本参考，当前CB3控制器上的实际单次命令仍待现场验证；不把文档查询当作硬件测试通过。

后续实际开爪验证已完成：用户运行open --send，记录20261022_171051_986666/result.json，状态output_level_readback_matches，输出0从true变false，输出1保持true，关节端点最大差0.002063度。用户明确反馈“打开了”，另存open_confirmation_20261022.json作为人工观察证据，不修改原始发送日志。该结果验证本台CB3的单次开爪命令；关爪、带物体夹持及自动分类执行尚未验证/启用。

后续单次空载关爪也已验证：用户运行close --send，20261022_171336_133454/result.json读回匹配，0从false变true，1保持true，关节端点最大差0.002746度；用户反馈“关上了”，另存close_confirmation_20261022.json。现在仅单次空载开/关得到现场确认，带载夹持、到位/夹紧反馈、自动序列和机械臂运动未验证。历史发送日志中的gripper_motion_verified=false是当时软件无法观察爪位的标记，人工确认保存在独立记录，不回写原始证据。

新增只读姿态记录工具record_robot_pose.py与waste_sorting/taught_poses.py。由现场授权人员选定并手动到达姿态后，待程序停止、机器人静止，再运行` .\.venv\Scripts\python.exe record_robot_pose.py HOME`。采集工具I/O状态、实际关节/配置TCP位姿、工具I/O状态三次OUTPUT-only RTDE快照，检查时间顺序、同一机器人、关节端点速度、停止状态及三快照关节差<=0.1度；保存data/taught_poses，不发运动、I/O或Dashboard控制。HOME只是人工用途标签，姿态安全与返回路径不能由记录自动证明；path_validated、automatic_replay_enabled及TCP验证均false。其他标签PICK_APPROACH/LEFT_DROP_APPROACH/RIGHT_DROP_APPROACH仅预留给现场示教记录，不由候选相机坐标自动生成。

现场已保存HOME记录data/taught_poses/20261022_172057_929968/pose.json；实际关节角约[259.1616,-79.3443,26.9139,76.6793,79.2847,-185.2449]度，配置TCP的Base位置约[-61.1275,506.4695,638.8701]mm。此位置为现场记录标签，不是自动安全认证。工具偏移及回放、路径验证仍保持未验证；禁止将关节角随意归一化后替代原始记录。下一步只读记录由现场授权人员选择的PICK_APPROACH，不根据未验证的候选Base坐标生成运动。

先采第二个安全姿态，再检查其是否构成新姿态和是否仍选中同一物理特征。不要把一次成功记录或模拟测试当作允许自动运动的依据。

## 后续现场阶段（本次不执行）

### MATLAB 支持包与只读机器人状态

用户已在MATLAB R2024b安装UR系列支持包，`urRTDEClient`和`urROSNode`都可被which找到。同目录.p优先于.m的显示无需删除文件。电脑直连有线地址147.250.35.74/24，机器人147.250.35.40；此前4次ping均回复，未发送机器人控制指令。网络设置及验证记录位于data/network_checks。

本机支持包的`urRTDEClient`初始化会设置输入寄存器recipe；其当前实现的`readJointConfiguration`读取target_q，`readCartesianPose`通过模型正运动学计算位姿，不能直接作为控制器actual_TCP_pose或实测标定点。本项目新增`matlab/read_ur5_state_readonly.m`直接订阅控制器输出timestamp、actual_q、actual_TCP_pose和tcp_offset，不建立输入recipe，不发送URScript、I/O或运动命令。

```matlab
addpath('D:\UR5_Waste_Sorting\matlab');
read_ur5_state_readonly('selftest'); % 离线解码与拒绝控制消息测试
state = read_ur5_state_readonly('147.250.35.40'); % 一次只读订阅
```

只读订阅使用RTDE端口30004与协议v2，不需要运行MATLAB External Control程序节点或启动ROS。连接仅订阅本连接的输出，获取一帧后关闭；异常时拒绝保存状态。RTDE数据同步开始不代表启动机器人程序。控制器返回的实际TCP位姿仍基于当前配置的TCP；当前150mm偏移的实际工具几何尚未验证，不能将此读数当作桌面标记的Base配对点。

脚本输出实际关节角（度）、Base下TCP位置/旋转向量（米/弧度）以及当前法兰到TCP偏移，保存到data/robot_state_readonly。该文件是完整独立MATLAB函数，未更改支持包或Python执行禁用设置。已实际运行MATLAB checkcode与离线selftest，通过；首次沙箱启动失败，沙箱外重跑成功。

真实读取尝试未成功：MATLAB无法建立TCP连接。随后实际复查Windows网卡，接口15“以太网”为Disconnected、0bps，配置地址147.250.35.74/24仍保留；此次ping也失败。尚未收到或保存真实机器人状态，不能视为RTDE协议兼容性验证通过。恢复网线连接后再运行上述只读命令，不需要按Play。已为脚本添加5秒ConnectTimeout。

后续现场验证已成功：用户恢复连接后运行脚本，获得控制器3.15.5.0的实际关节角、actual_TCP_pose和tcp_offset，并保存data/robot_state_readonly/tp3909112f_b0fa_4744_b7ab_f5c6b35d010e.json。已读取该真实文件，Base下TCP位置约[-297.474,-256.415,720.589]mm，当前生效偏移[0,0,150]mm。用户示教器Feature=Base页面的XYZ和关节角与读取结果吻合；旋转向量换成等价表示后约[0.422181,2.263102,-2.186275]rad，与屏幕三位小数相符。该交叉核对通过不代表工具几何或外参标定通过，机器人运动控制仍未启用。

先实际完成并验证这四个二维区域。之后再核实相机是否移动、TCP工具中心、可安全到达的采样点，采集匹配的机器人 Base 点，求外参并用独立点验证。真机路径、接近高度、投放姿态及夹爪需要现场授权人员确认；示教器 TCP Position 的逆解问题尚未诊断修复。


## 2026-10-23 恢复现场与四个人工姿态记录完成

恢复网线后用户提供4次ping回复且无丢包。只读RTDE状态记录data/gripper_io_tests/20261023_102008_374791/result.json：控制器3.15.5.0、程序STOPPED、安全模式NORMAL、采样关节速度为零、工具24V；输出0=false、输出1=false，与前一日不同，本次未修改输出，不由电平推断实际爪位。用户确认现场授权人员在场且夹爪空载。

现场人员手动选定姿态后，用户运行只读记录工具，新增：
- LEFT_DROP_APPROACH：data/taught_poses/20261023_102722_810012/pose.json，三快照最大关节差0.003415度。
- RIGHT_DROP_APPROACH：data/taught_poses/20261023_102943_361690/pose.json，三快照最大关节差0.002753度。

连同此前HOME、PICK_APPROACH，四种标签均已有记录；已用build_pose_record离线复核四份原始快照，记录检查均通过，未改写原始文件。左右含义沿用物理分类区，不按Base坐标正负或图像横坐标重定义。接近姿态不等于最终释放或抓取姿态；静止端点不证明路径安全，不将端点直接连接为movej/movel回放。TCP、候选外参的独立物理验证及路径验证仍未通过，自动回放与视觉分拣执行保持禁用。本次恢复检查中69项离线测试通过，未发送机械臂运动或夹爪动作。


### 海绵人工示教记录更正（2026-10-23）

用户明确纠正：data/taught_poses/20261023_103927_499153/pose.json记录时夹爪未闭合、夹指压着桌面；该记录不可作为有效接近或抓取目标复用。先前静止检查通过只说明采样时关节稳定，不证明无接触。原始文件保留不改写。

用户随后人工调整并确认张爪、两指位于海绵两侧、不接触海绵和桌面，另存data/taught_poses/20261023_105542_685052/pose.json，原标签仍为PICK_APPROACH；离线复核三次快照最大关节差0.003442度。该间隙为用户现场观察，软件未测量确认，不构成抓取、路径或姿态安全验证。新姿态关节配置与旧记录明显不同，不将两条记录连接为自动路径。

独立说明data/taught_poses/sponge_posture_review_20261023.json保存两份源文件路径、SHA256及现场反馈。此说明是审阅证据，不是执行拦截器；自动回放与视觉分拣执行仍禁用。当前尚未完成此次海绵闭爪、抬升和带载夹持验证。


海绵人工夹持现场反馈：用户在上述调整后的待夹持位置确认“夹住了”，随后人工短距离抬升，反馈“离开了，未滑动”。独立观察记录：data\taught_poses\sponge_manual_grasp_observation_20261023_105826_576046.json。仅一次人工夹持和离桌后停住的观察；抬升距离、持续时间和抬升后姿态未记录，未完成释放、运输路径或重复可靠性验证。未启用自动回放或分拣执行。


海绵原地人工试验后续：用户确认放回桌面、夹指保持间隙，张爪后反馈海绵留在桌面并与两指完全分离。释放观察记录：data\taught_poses\sponge_manual_release_observation_20261023_110231_491819.json。至此完成一次人工原地夹持、短距离离桌、放回与释放；未进行分类区运输，未验证重复成功率、夹持力、TCP或自动成功检测。观察依据为用户现场反馈，助手未发送运动或I/O写入，执行禁用状态不变。


海绵释放后，用户保持张爪并人工抬离海绵，记录上方接近姿态data/taught_poses/20261023_110341_722684/pose.json（标签仍为PICK_APPROACH），三快照最大关节差0.004120度。用途说明保存在同目录operator_role.json；区别于20261023_105542_685052的两指在海绵两侧待闭爪位置。间隙为现场反馈，未测量。多个同名标签不可自动按最新记录选作回放目标；各姿态之间路径仍未验证。


海绵人工示教后续：用户空载张爪、手动到物理RIGHT_CELL上方，记录data/taught_poses/20261023_110858_878854/pose.json，标签RIGHT_DROP_APPROACH，离线复核最大关节差0.002746度。用途说明见同目录operator_role.json。原关节角含腕关节约-295.95度和177.65度，保留原值，不做整圈归一化，不由端点推断路径安全；尚未记录最终释放位置或完成带载向右侧运输。


用户确认RIGHT_CELL是桌面划分区域而非垃圾桶，判断当前空载张爪位置可作为海绵释放候选，不继续下降；新增data/taught_poses/20261023_111229_526704/pose.json（原标签RIGHT_DROP_APPROACH），用途说明同目录operator_role.json。三快照最大关节差0.003442度。与110858记录的配置TCP位置差约0.063mm，最大原始关节差约0.0048度，实际是近乎同一姿态，未示教独立下降段。候选释放用途来自现场判断，尚未带海绵到此释放、未测量离桌间隙，不代表运输路径或自动放置通过。


海绵第二次人工夹持未成功保持：用户确认原地夹上且未碰桌面，随后抬升时反馈滑落、海绵落到桌面；之后张爪，用户确认夹指仍与桌面有间隙。记录：data\taught_poses\sponge_manual_grasp_failure_20261023_113832_618571.json。尚未向RIGHT_CELL完成带载运输或释放。原因未确定，未测量夹持力或接触几何，也没有此次实际夹取姿态记录，不用旧姿态替代。前一次短抬升未滑动的观察保留，但两次结果不一致，不能宣称带载夹持稳定；暂停重复夹取，先检查实际接触位置。I/O映射不变，自动回放及分拣执行仍禁用。


海绵人工向RIGHT_CELL搬运后续：用户说明先前滑落时只夹住一角；调整后明确反馈夹住中部，随后确认短距离抬升、到右侧上方、放下与张爪分离步骤完成。现场反馈记录：data\taught_poses\sponge_manual_right_transfer_observation_20261023_114425_325572.json。本次作为一次由现场人员手动操作的搬运和释放记录；部分反馈仅简短确认，没有实测间隙、连续滑动监测、夹持力、实际抓取/释放姿态或路径轨迹。夹角滑落为此前失败的可能因素，未证明唯一原因。保留失败记录，不宣称重复可靠性或自动分拣完成；相机外参独立验证、TCP及自动回放路径仍未通过，助手未发送运动或夹爪写入。


### 离线姿态用途清单工具

新增review_taught_poses.py与waste_sorting/pose_catalog.py。运行 `.\.venv\Scripts\python.exe review_taught_poses.py` 仅扫描本地姿态、核对三次快照与记录摘要、已有用途说明的源路径及SHA256，生成data/pose_review/catalog.json和catalog.md，不连接机器人，不选取最新同名记录作为目标，不生成运动路径。实际扫描为9条姿态记录，其中1条曾压桌面、明确不可复用，证据错误0条。原始记录均保留。清单是审阅工具，不是运动系统中的执行拦截器。

新增4项离线测试覆盖同名标签及碰桌标记保留、源文件改动、关节摘要篡改、用途说明错绑源文件；全套指定tests目录的73项离线测试通过。后续真实抓取中部调整后的成功抓取位置和实际带载释放位置没有即时姿态快照，不能把较早的候选姿态冒充这次成功试验端点。自动回放、TCP和路径验证状态不变。


### 独立抓取与释放用途标签

record_robot_pose.py新增PICK_GRASP（两指对准物体两侧的待夹持位置）、LEFT_DROP_RELEASE及RIGHT_DROP_RELEASE（相应物理分类区的候选释放位置）。原HOME、PICK_APPROACH、LEFT_DROP_APPROACH、RIGHT_DROP_APPROACH继续支持；旧文件不改名、不改写，也不根据新标签含义重新解释旧记录。每个新增采集记录含label_purpose和label_scope，明确只是人工用途而非物理验证。CLI显示用途说明，全部动作、TCP、路径和回放验证标志仍保持false。仍为三次只读快照，不按标签发送动作，不要求或推断夹爪状态。

离线清单支持新用途，并保留旧记录的独立现场说明。新增测试验证新标签走原只读采样流程且不解除执行禁用，以及旧记录不被新标签重解释。75项指定tests目录离线测试通过；CLI帮助与现存9条记录清单生成通过，1条碰桌不可复用、证据错误0条。本次软件修改未连接机器人或相机。

后续补录命令为 `.\.venv\Scripts\python.exe record_robot_pose.py PICK_GRASP` 和 `.\.venv\Scripts\python.exe record_robot_pose.py RIGHT_DROP_RELEASE`，仅由现场人员手动到位、程序停止且机器人静止后运行。若以空载状态补录释放候选，不能将其描述为此前成功试验的实际带载释放快照。


独立RIGHT_DROP_RELEASE已补录：data/taught_poses/20261023_115143_836946/pose.json，离线复核快照和摘要通过，最大关节差0.002759度。此为空载张爪候选释放位置，不追溯冒充此前带载试验端点。用户要求后续不再漏记，新增MANUAL_TRIAL_CHECKLIST.md：关键位置必须先停止、采集、核对成功，再闭爪、转移或释放；调整后重新记录，错过快照明确标注缺失。下一项待补为PICK_GRASP，闭爪前先记录。


独立PICK_GRASP已记录：data/taught_poses/20261023_120021_416834/pose.json。用户在闭爪前采集，标签表示两指对准物体两侧的待夹持用途；离线复核三次快照和关节/TCP摘要通过，最大关节差0.002759度。此时尚未收到本次闭爪/抬升结果，不能标记抓取成功。原始关节角保留，TCP、路径与自动回放未验证。


当前补录试验：PICK_GRASP记录120021后用户反馈夹住，随后确认短距离抬升步骤，另存data/taught_poses/20261023_120324_017981/pose.json，最大关节差0.004098度。其实际用途是夹持海绵后的抬升位置，虽然原标签PICK_APPROACH的通用说明写抓取前；以同目录operator_role.json的具体现场用途区分，原始记录不改写。未测量沿途间隙，尚未记录本次右侧接近与带载释放位置，必须到位后先采集核对再继续。


当前海绵补录试验右侧上方接近位置已保存：data/taught_poses/20261023_120638_546475/pose.json，RIGHT_DROP_APPROACH，快照与摘要离线复核通过，最大关节差0.002746度。具体用途及关联前置记录见同目录operator_role.json；尚未采集本次实际带载放置位置，下降到位后须先记录RIGHT_DROP_RELEASE、核对成功，再由现场人员张爪。路径未记录，自动回放禁用。


当前海绵试验张爪前放置姿态已采集：data/taught_poses/20261023_120809_351330/pose.json，RIGHT_DROP_RELEASE，三快照及摘要复核通过，最大关节差0.002063度。此次在人工搬运流程中张爪前采集，区别于115143空载候选。四个本次端点及SHA256已关联于data/taught_poses/sponge_trial_20261023_120021_endpoints.json；只是证据清单而非轨迹，禁止直接连点回放。当前释放观察仍待用户确认，软件未测量桌面承托和间隙。


当前四端点海绵试验释放结果：用户明确确认海绵留在RIGHT_CELL内并与两指完全分离。确认记录：data\taught_poses\sponge_trial_120021_release_confirmation_20261023_120955_860416.json，绑定端点清单路径和SHA256，原始端点及清单不改写。至此本次人工抓取/搬运/放置/释放有四个已保存的关键端点及用户释放观察；没有完整路径或自动成功检测。释放后抬离尚待确认，自动执行继续禁用。


本次四端点海绵人工试验收尾：用户确认张爪后向上抬离完成，收尾记录data\taught_poses\sponge_trial_120021_closed_20261023_121054_643004.json，绑定释放确认SHA256。本次人工试验结束；抬离后未再采集姿态，不能宣称完整路径已记录。保留夹角滑落失败与全部原始文件，自动回放及视觉分拣执行仍禁用。后续可先做离线试验汇总，不再为收尾重复要求现场抓取。


已完成本次海绵试验离线证据链复核：data/manual_trial_review/sponge_trial_120021_summary.md及同名JSON，逐层校验收尾→释放确认→四端点清单→原始快照的SHA256、标签顺序与关节/TCP摘要，全部一致。汇总明确TCP物理对应、候选外参独立验证、实际抓取点与路径仍待解决。本次未连接硬件。


TCP几何粗核对：用户报告从法兰安装面沿轴向量到末端黑色夹指内侧中部高度为150mm，并确认两接触区中点在法兰中心轴延长线上。记录：data\tcp_checks\geometry_coarse_check_20261023_123000_071589.json。这些反馈支持现有[0,0,150mm]名义平移，但未量化误差、横向偏移，未验证工具方向或不同开口状态下的抓取中点位置。测量时实际开口尚待明确；tcp_verified及独立物理验证仍false，未修改控制器配置。


TCP粗量状态补充：用户明确150mm是在夹爪张开状态测得；空载闭合后测得200mm，并确认同一法兰安装面、同一轴向及同一黑色夹指内侧中部，不是指尖最下端。比较记录：data\tcp_checks\open_closed_geometry_20261023_123621_502594.json。报告差值50mm提示抓取参考点随开合状态变化，测量误差未量化；不能把150mm当所有开口下通用抓取TCP，也不能直接改成空载闭合的200mm。下一步待核实实际夹海绵时参考点，暂不按厚度插值。原配置和姿态文件保持不变，tcp_verified及自动执行仍false。


TCP核对阶段：用户提醒海绵仍在RIGHT_CELL，不要求把它搬回PICK_ZONE。新增独立标签TCP_CHECK_GRASP，表示不限物理区域的TCP几何核对用待夹持姿态；闭爪前采集，不能解释成TCP验证已通过。CLI及操作清单已更新，新标签纳入已有只读采样/执行禁用测试；75项离线测试通过，CLI帮助确认支持新标签。本次不连接机器人，不修改旧记录及控制器配置。下一步由现场人员原位对准海绵两侧中部、确认与桌面有间隙后，用TCP_CHECK_GRASP先记录，核对成功再闭爪。


RIGHT_CELL海绵原位TCP核对用待夹持位置已在闭爪前保存：data/taught_poses/20261023_124213_912891/pose.json，标签TCP_CHECK_GRASP；三次快照与摘要复核通过，最大关节差0.002746度，用途及区域见同目录operator_role.json。此时尚待实际闭爪观察及海绵夹持开口下几何测量，未将张开150mm或空载闭合200mm直接用于修改TCP。自动执行仍禁用。


海绵夹持状态TCP粗量：在TCP_CHECK_GRASP记录124213后，用户报告夹住，按同一安装面/轴向/夹指内侧中部要求尺量195mm。记录：data\tcp_checks\sponge_gripped_geometry_20261023_124503_486720.json。与当前150mm名义轴向长度差45mm；结合张开150mm、空载闭合200mm，作为开口相关几何的粗量证据，尚未量化精度、夹持宽度和横向偏移，不改写控制器TCP或旧姿态，不宣称TCP已验证。下一步在保持同一夹持状态和机械臂静止的条件下，独立重新放尺核对重复读数，不为测量抬升或转动机器人。


海绵夹持轴向复测：用户再次明确195mm，与第一次一致；复测反馈绑定前次测量SHA256，见data\tcp_checks\sponge_gripped_repeat_20261023_124731_606133.json。不再重复尺量。195mm仅为当前海绵夹持状态的轴向估计，不证明X/Y零偏、方向或绝对精度；保持控制器原配置不改，tcp_verified仍false。后续先由现场人员确认海绵有桌面承托再原地释放，随后讨论独立实体参考的验证方式。


TCP尺量阶段收尾：用户确认张爪后海绵完全分离，记录data\tcp_checks\measurement_release_20261023_124819_851976.json。张开150mm、空载闭合200mm、海绵夹持195mm（重复一致）均为粗量，未更改控制器TCP。下一步需要确定独立固定参考及虚拟抓取中点的可重复定位方法，暂不进行旋转或自动回放。


已按用户请求核对Phase 1原聊天及代码，蓝色logo联合外参/法兰偏移的已有成果应保留复用，不重做；它估计的是固定机身点，不是开合相关抓取中点。旧拟合已撤销配置TCP偏移还原法兰，物理TCP粗量变化不自动推翻旧拟合。详情data/tcp_checks/phase1_logo_tcp_distinction.md。先暂停重复找型号、量尺和实体动作，依据已有证据区分可复用成果及剩余独立验证。


### 复用Phase 1的离线TCP候选几何对照

新增review_tcp_geometry.py，仅使用历史姿态、量尺记录、冻结外参证据进行离线对照。先核对尺量与姿态SHA256/快照摘要，调用load_candidate复核当前工作区指纹及原始8训练+3留出证据，并重现logo偏移；原始报告和配置不改写。输出data/tcp_checks/historical_geometry_comparison.json及.md，保留来源哈希和假设。海绵假设点暂取法兰下[0,0,0.195]m，其中X/Y=0仍属未量测假设；不输出抓取姿态或轨迹。

历史124213姿态下，配置150mm点到195mm假设点距离45mm，Base差向量约[44.802,1.736,-3.848]mm；不是给Base Z加45mm。相机坐标仅为历史姿态的候选预测，不能代表实时测量或独立物理验证。新增3项离线测试验证法兰旋转下的偏移方向、相机变换/logo与抓取点区分、非法变换拒绝，78项离线测试通过。下一步可准备只读视觉叠加核对，但必须先确认固定ZED是否移动；视觉叠加也不代替独立物理精度验证。TCP与自动回放/分拣执行继续未启用。


### 只读TCP假设点叠加

用户确认固定ZED Mini自Phase 1标定以来没有移动或调整角度，现场反馈单独保存在data/tcp_checks的camera_unchanged_report记录中；每次启动仍须重新检查区域并按V，不持久化本次确认。

新增tcp_overlay_preview.py及waste_sorting/tcp_overlay.py。复核冻结候选外参和logo偏移，使用ZED当前HD720、校正LEFT图像对应内参；不猜测内参、不加载YOLO。界面LIVE时不显示旧预测，按V确认区域后，M依次读取工具输出状态、机器人法兰相关状态、采集新图像、再次读取工具状态；沿用静止/时间顺序/10秒跨度/关节差检查，并确认图像采集在快照时间括号内。失败不保留叠加。仅输出订阅，无运动、工具I/O写入或控制器配置变更。

M后冻结该张图与三个候选投影：1黄色logo拟合点、2青色当前配置TCP点、3品红195mm海绵夹持假设点。195mm假设X/Y为0，当前张爪时不代表真实中点；无深度采样，也不从二维重合推断三维精度或独立标定通过。S保存冻结原图/标记图/sample.json（状态快照、时间、内参及候选来源），R清除冻结结果并回LIVE，Q退出。相机仅由一个程序拥有。

关闭其他ZED程序，保持机械臂静止、程序停止后运行：

```cmd
.\.venv\Scripts\python.exe tcp_overlay_preview.py
```

新增4项离线测试覆盖投影/画面外/相机后方、张爪状态下195mm仍属未验证假设、运动拒绝、图像越出时间括号拒绝。共82项指定tests目录离线测试通过，入口语法检查通过。真实相机及机器人只读联测尚待现场运行；助手未启动相机或连接机器人。自动回放及分拣执行继续禁用。


只读TCP叠加首次现场运行成功：data/tcp_overlay_checks/20261023_125953_962610/sample.json，相机16343130，三次快照最大关节差0.002759度，时间括号与投影数值离线重检一致。已查看原始RGB，预测logo投影在蓝色logo附近，仅定性观察，没有测量像素误差或深度误差。当前输出0=false、现场图为张爪，不能据此验证195mm海绵夹持假设。独立审阅记录在同目录review.json。后续若原位夹持条件仍满足，由现场人员闭爪、确认中部夹持后，保持不移动，再采集新的冻结叠加；必须R清旧画面，不能对旧帧解释当前状态。


第二条只读TCP叠加已保存：data/tcp_overlay_checks/20261023_130433_892427/sample.json。输出0前后均true，输出1均false；与此前张爪样本最大原始关节差约0.003436度，本次采样静止差约0.003470度。原始图可见夹指构型变化，预测点基本不变是法兰姿态和固定偏移模型不变所致，不是实时跟踪或195mm验证。此样本尚需现场明确实际夹住海绵与否；不凭输出电平认定抓取成功，也不从截图宣称二维/三维误差。审阅见同目录review.json。


TCP叠加130433样本后续：用户明确确认实际夹住海绵，补充证据在该目录grasp_confirmation记录中，绑定sample.json哈希。只确认实际夹持，不代表品红195mm投影点与实际抓取中点对应，尚待二维对应观察，不能宣称TCP通过。


195mm海绵候选点二维对应观察：用户确认130433冻结图品红3号十字位于两指实际接触海绵的中部之间。结合两次195mm尺量，保存海绵专用名义平移候选data\tcp_checks\sponge_translation_candidate_20261023_130812_000524.json。仅定性二维对应，无像素残差或深度误差；X/Y零偏未量化，方向未验证，不代表三维TCP精度或独立物理验证通过。原logo/相机候选保留，控制器150mm未改，自动回放和分拣执行仍禁用。下一步先完成原地释放收尾，不要求再次夹取。


本轮TCP候选叠加检查收尾：用户明确反馈张爪后海绵分离，记录data\tcp_checks\overlay_check_release_20261023_130852_714922.json，关联195mm候选文件SHA256。没有抬离或相机退出反馈，不推断这两项已完成。该轮证据足以支持继续离线候选软件工作，不再重复夹取；控制器TCP未改，三维独立验证仍待完成。


### 单物体20帧表面坐标稳定性诊断

新增waste_sorting/temporal_preview.py，sorting_preview.py新增可选--observe-stability。原默认单帧流程和0.80全局分类策略不变；0.45仍只用于显式预览门槛。启用后独立累计20个有效处理样本，保留每帧处理序号、ZED IMAGE纳秒时间戳、主机时间、框中心/边框、相机点和候选Base点。按S时随preview.json保存当前窗口，不复用旧深度，不生成平均抓取目标。

只接受恰好一个candidate_surface_preview_only条目。空检测、多检测、拒绝条目、无效几何、重复/倒退传感器时间戳或处理序号中断会清空；类别/分类区/尺寸变化、框中心相对窗口首样本位移>8像素、bbox IoU<0.5、主机间隔>1秒或窗口跨度>10秒会重新累计，按V重新确认也清空窗口。同类物体被替换但几何相似仍可能无法识别，physical_object_identity_verified保持false。

20个样本后按相机XYZ各轴标准差<=5mm且各轴极差<=20mm报告stable_surface_observation_only，否则报告unstable_surface_observation。这些是暂定诊断阈值，不是抓取许可或精度标准；20个有效处理样本不能证明ZED期间没有丢帧，也不保证框中心是适合抓取的物体表面。旧单帧记录不能追溯称为通过20帧检查。--image与此选项互斥。

关闭其他ZED程序、准备PICK_ZONE内单个静止物体后运行：

```cmd
.\.venv\Scripts\python.exe sorting_preview.py --preview-min-confidence 0.45 --observe-stability
```

检查区域并V确认，观察Temporal 20/20状态后S保存，Q退出。该入口不连接机器人；TCP195mm候选仅保留在独立检查证据中，不被此次诊断自动应用或标为已验证。新增6项离线测试，全套88项通过；CLI帮助检查通过，真实相机联测待现场完成。


海绵20帧诊断首次现场记录：data/sorting_previews/20261023_131447_362569/preview.json，帧257–276共20个有效处理样本，窗口2.827秒，保存帧海绵置信度92.4%，分类RIGHT_CELL。离线重算相机XYZ标准差约[0.01949,0.01505,0.06889]mm，各轴极差[0.06866,0.05300,0.24271]mm，与记录一致，满足暂定诊断阈值；审阅见同目录temporal_review.json。仅证明单位置短时间表面坐标抖动较小，不证明绝对精度、期间无丢帧、TCP或抓取点正确。用户截图为后续帧402的9/20累计，与已保存帧276不同，不能据此认定该保存窗口未达20。自动执行仍禁用。下一项可换PICK_ZONE内第二位置复查稳定性，不将其称为准确率或绝对定位验证。


海绵第二位置稳定性已复核：data/sorting_previews/20261023_131844_792003/preview.json，帧1496–1515共20样本，窗口2.860秒；相机XYZ标准差约[0.649,0.648,0.117]mm，极差[1.494,1.572,0.417]mm，满足暂定诊断阈值。保存帧置信度50.4%，仅通过本次0.45预览门槛，默认0.80会拒绝。两位置汇总data/sorting_previews/sponge_two_position_temporal_review_20261023.json。识别框中心像素有变化，可影响XYZ抖动；两组位置不同，不用坐标差推断定位误差，不宣称精度或抓取验证通过。


塑料瓶20帧诊断已复核：data/sorting_previews/20261023_132223_349833/preview.json，20帧、约2.797秒，分类recyclable→LEFT_CELL，保存帧置信度64.8%；相机XYZ标准差约[0.504,0.118,0.464]mm、极差[1.123,0.339,1.328]mm，通过暂定诊断阈值。审阅见同目录temporal_review.json。仅可见表面点的短窗稳定性，不证明实际抓取点或绝对精度，自动执行禁用。


铝罐20帧诊断已复核：data/sorting_previews/20261023_132345_297757/preview.json，约2.806秒，保存帧置信度87.2%，分类recyclable→LEFT_CELL。相机XYZ标准差[np.float64(0.215), np.float64(0.39), np.float64(0.791)]mm、极差[np.float64(0.503), np.float64(0.871), np.float64(1.849)]mm，通过暂定诊断阈值。审阅见同目录temporal_review.json；实际抓取点及绝对精度仍未验证，自动执行禁用。


纸杯20帧诊断已复核：data/sorting_previews/20261023_132625_639336/preview.json，保存帧置信度91.6%，相机Z标准差0.046mm、极差0.180mm，分类LEFT_CELL，满足暂定诊断阈值。四类五窗口汇总data/sorting_previews/four_class_temporal_check_20261023.json及.md，已核对共同工作区及候选报告哈希。低抖动不代表绝对精度；保存窗口为选取结果，不含重置/失败统计，不宣称准确率或成功率。纸杯杯内点不视为杯沿抓取点，自动执行仍禁用。


### 同一摆放下海绵视觉/人工参考一致性核对工具

新增review_sponge_reference.py，只做离线比较，不连接硬件。输入一条新的单海绵稳定视觉记录、同一摆放下的人工对准姿态、已有195mm海绵候选，以及粗量厚度。必须另行得到用户明确确认：摆放未变、海绵平放且采样点在顶部、夹持参考在海绵高度中部；命令行确认参数不得代替实际观察。重现冻结候选外参并核对视觉报告/工作区哈希、TCP候选源哈希和姿态快照摘要。

模型将视觉顶部点沿Base -Z减去粗量厚度一半，与法兰变换乘以195mm候选点比较，输出带来源哈希的差向量/模长。顶部到中部沿Base竖直方向，法兰到抓取点偏移随法兰旋转，两者不可混用。两指接触中点与框中心的XY也可能不同，误差还混合了厚度、人工对准、外参、深度与TCP偏差，因此仅一致性诊断，无自动合格阈值，不标记独立物理验证/TCP通过，不输出执行抓取姿态或路径。

91项指定tests目录离线测试通过，CLI帮助通过。现场尚未采集这一轮同摆放配对数据，不能将此前不同摆放的旧视觉与旧姿态冒充匹配记录。下一步先换回海绵、平放PICK_ZONE，保存新的20帧视觉样本；之后保持海绵不动，再由现场人员对准实际夹持部位并先记录。已知约20mm厚度来自旧尺量，需确认本轮仍用同一块海绵且未改变放置方向。


同摆放海绵对照视觉基准已核对：data/sorting_previews/20261023_135050_208425/preview.json，单海绵20帧诊断通过，保存帧置信度89.3%，相机XYZ标准差[np.float64(0.476), np.float64(0.532), np.float64(0.196)]mm。基准引用和SHA256登记于data/sponge_reference_checks/pair_20261023_135050_visual_baseline.json，目前人工对准姿态及同摆放/顶部/高度中部的现场确认仍待补，未计算配对差异。保持海绵原位，不自动关联旧姿态。


海绵同摆放检查闭爪前人工对准姿态已记录：data/taught_poses/20261023_135746_993683/pose.json，TCP_CHECK_GRASP，最大三快照关节差0.004098度。与135050视觉基准关联的用途说明在同目录operator_role.json；尚待用户明确确认人工对准未碰动海绵，且195mm对应夹持状态，仍须闭爪观察及闭爪后姿态，当前不计算对照差异。


135050视觉—135746闭爪前姿态配对：用户明确反馈海绵没有被动过，补充确认记录data\sponge_reference_checks\pair_135050_unchanged_before_closure_20261023_140056_479276.json。仅确认闭爪前摆放未变，未将此回答扩大成厚度、闭爪后位置或高度中部接触确认；仍不计算差异。


同摆放对照闭爪后姿态已保存：data/taught_poses/20261023_140257_149497/pose.json，最大采样关节差0.005491度，较135746闭爪前最大原始关节差0.002076度。用途及视觉记录关联见同目录operator_role.json。用户已反馈夹好，但闭爪未推移、接触高度中部、同块约20mm厚海绵三项明确确认仍待补，不自动用默认值运行一致性比较。


同摆放对照已计算：用户明确确认闭爪未推移/转动海绵、两指在高度中部接触、同块约20mm厚海绵平放。使用135050视觉与140257闭爪后姿态，输出data/sponge_reference_checks/20261023_140514_583986/comparison.json，人工195mm假设参考减视觉顶部下移10mm参考的差约[-34.192,22.583,42.832]mm，模长59.276mm。独立矩阵算式复核一致；这是模型间差异，不是已分离的TCP/外参误差。候选视觉几何下横向分量也显著，不能只改Z解释。此次暴露[0,0,195mm]轴向假设不能据现有证据应用；此前二维目测和尺量不足以确认完整TCP，原证据保留，审阅见同目录review.json。控制器保持150mm未改，不生成修正运动。下一步先完成原地释放，再检查安装轴线/夹指接触参考及投影假设，不能重做同样叠加便宣称独立验证通过。


59.276mm差异已离线分解，见data/sponge_reference_checks/20261023_140514_583986/geometry_diagnosis.json及.md：沿法兰轴差42.322mm，横向差41.503mm。本次法兰Z轴与Base水平面仅约1.91度，所以增加工具轴向长度不能解决约42.8mm的Base高度差。不能仅据二维投影位于两指之间就认定X/Y=0。条件反算法兰坐标只供诊断，不作设置；原配置不改，自动执行禁用。用户只回复继续，尚未明确反馈上一步释放结果，不记录已释放。下一步可在原姿态做只读配对图像，检查logo与实际接触区域。


### 暂停TCP排查后：已保存图片离线类别评估

按用户要求暂缓TCP及59mm差异排查，保留原证据和执行禁用状态。新增evaluate_saved_vision.py、waste_sorting/vision_evaluation.py及5项测试，总计96项离线测试通过。工具不连接相机或机器人，沿用队友512拉伸/CPU权重与原接口；先核对原图/源预览SHA256，复跑每张原图，在0.45/0.80分别过滤并检查框中心是否仅位于PICK_ZONE，按类别列表与目视标签作比较，不计算mAP或抓取准确率。

库存15图：8海绵、瓶/罐/杯各2、空待抓区1。标签由助手原图目视审阅而来，尚待用户复核；区外LEFT_CELL海绵不算待抓区目标。发现154448与154818海绵图SHA256相同，去重后14图（海绵7、其余各2、空1）。去重报告data/vision_evaluation/runs/20261023_142014_672562/report.md及JSON：0.45下类别列表一致14/14，0.80下7/14；按类0.80保留海绵2/7、瓶0/2、罐2/2、杯2/2，空待抓区1/1输出正确为空。只是已选小样本描述，存在选择偏差、场景相关性、未知训练重叠和负样本不足，不能推广为总体准确率，不能据此修改默认0.80。

评估清单data/vision_evaluation/manifest.json、目视复核索引label_review.png、汇总README.md已保存。模型/依赖/全局策略未改。本次全程离线；下一阶段优先补固定采集顺序、保留失败的独立样本及空区/未知物体负样本，而非继续挑成功画面或立即重训。


新增模型无关评估图片采集capture_vision_dataset.py及waste_sorting/dataset_capture.py，详见data/vision_evaluation/CAPTURE_GUIDE.md。先V确认，再B倒计时3秒，默认5次、至少1秒间隔，自动保存原图与时间戳/哈希，Q/异常保留部分与失败。三场景empty_pick/outside_known/unknown_in_pick，场景不自动成为标签。评估加载器支持经目视复核后引用原始采集会话；新增4项测试，完整100项离线测试通过，CLI帮助通过，真实采集待用户运行。未连接相机或机器人，原模型/策略/依赖不变。


模型无关负样本采集首组完成：data/vision_evaluation/captures/20261023_143045_536052/session.json，5次全部保存，跨度4.358秒，时间递增及SHA256核对通过。已逐张查看原图，PICK_ZONE为空，目视标签另存empty_pick_manifest_20261023_143045.json，不改原会话。离线报告data/vision_evaluation/runs/20261023_143329_208707/report.md：0.45与0.80下5张均无待抓区已知类输出。仅1组相关背景场景，不推广误检率。下一组outside_known保持PICK_ZONE空，将已知类物体放在物理LEFT_CELL等区外。


区外已知物体组已完成：data/vision_evaluation/captures/20261023_143620_141770/session.json，5张全保存、时间戳及SHA256核对通过。逐张原图可见塑料瓶在物理LEFT_CELL，PICK_ZONE空；另建outside_known_manifest_20261023_143620.json，不改原会话。离线报告data/vision_evaluation/runs/20261023_143852_859889/report.md，两门槛均无待抓区已知类输出；检测/过滤明细见报告与会话review.json。仅1个摆放场景，不能推广总体误检率。下一组unknown_in_pick用四类之外普通物体放在待抓区，记录有无冒认已知类。


区外物体身份补充：用户明确说明画面左侧红色容器是误放的铝罐，因此模型对其aluminum_can输出不能认定为误认或未知物体误检。确认记录保存在data/vision_evaluation/captures/20261023_143620_141770的object_identity_clarification文件中。区外实际同时有塑料瓶和铝罐，PICK_ZONE依然空，既有ROI-only评估结果不变，无需重采或重跑；保留原会话/报告和哈希引用。未知物体测试尚未进行，不推断其结果。


用户明确要求跳过unknown_in_pick未知物体测试，不再将该采集作为下一步。未知物体鲁棒性保持未测试，已有四类及空待抓区/区外过滤结果保留；不据跳过推断通过或失败。范围决定见data/vision_evaluation/scope_decision_20261023_144724_180594.json。


新增统一分拣任务只读预览waste_sorting/task_review.py与离线回放preview_sorting_tasks.py，接入sorting_preview.py实时画面及保存JSON。重新应用全局0.80门槛，不受0.45预览覆盖；检查原始检测状态、区域/深度、类别路由、当前帧稳定窗口，核对时间戳/身份和重算坐标抖动。所有物理阻塞仍硬性保留，不读取输入true标志自动许可动作；无执行目标、轨迹或任务自动下发。新增5项测试，总105项通过。默认五条已保存样本离线回放完成，报告data/task_reviews/20261023_145527_190365/report.md，3条视觉检查通过、低置信度海绵/塑料瓶2条被全局门槛拦下。使用说明TASK_PREVIEW_GUIDE.md。未连接相机或机器人，TCP与未知物体测试仍按用户决定暂缓。


用户提供2F85张开/闭合STEP并反馈老师/设备资料确认匹配，已读入并渲染，详见data/cad_review/2f85_20261023/README.md。CAD为Robotiq 2F-85外形，和此前现场OnRobot标识外形存在矛盾；型号来源确认保留，尚待当前硬件是否更换等澄清。原始CAD单位毫米，两状态模型坐标位置不同，未作机器人法兰配准，包络尺寸不能直接当TCP。原配置、已验证I/O映射和运动禁用状态均不变。


用户明确先前2F-85文件传错，与当前夹爪不同；后续提供RG6_v2.STEP并报型号RG6，已导入并渲染原始CAD边线，见data/cad_review/rg6_20261023/README.md及RG6_v2_views.png。本体外形与现场OnRobot照片相符，包络约263.595×164.280×76.007mm、174实体，但不是TCP；法兰配准及开口状态未确定。现场向下延伸的黑色末端夹指与CAD末端轮廓有差异，尚需确认加装件。它可能贡献此前漏掉的横向偏移，但不据此宣称已解59mm差异。现有TCP配置与I/O方法均不改，后续先核对末端件CAD，无机器人动作。

用户随后明确“以cad得出的TCP为准”。已按该决定采用RG6 STEP作为名义几何依据，新增tools/derive_rg6_nominal_tcp.py并实际复算通过。选定CAD参考系中名义点[0,0.1,230.538115]mm；定义为两个标准指垫平面面积中心的中点，平面间距103.168118mm。源哈希/实体归属/安装圆半径/坐标变换检查通过。详见data/cad_review/rg6_20261023/CAD_TCP_REFERENCE.md及cad_nominal_tcp.json。该点不是已配准UR六维TCP：安装环平面与凸台端面有2mm差异，机器人法兰到CAD变换仍为空；不要求重复量尺，不用先前195mm候选覆盖CAD结果。运行时代码、控制器设置、工具I/O与自动运动禁用状态未改变。

补充机器人侧Quick Changer STEP后完成接触面配对候选与OCC实体相交检查，记录data/cad_review/quick_changer_20261023/ASSEMBLY_PROGRESS.md。候选参考系下轴向距离255.338115mm；0°/180°两方向未检出实体交叠，但真实法兰方向未配准。用户反馈示教器无安装配置且要求按数值推进，现采用255.34mm作为只读投影轴向假设（假设沿法兰+Z且X=Y=0），记录working_assumption.json。新增tcp_overlay_preview.py --axial-hypothesis-mm 255.34，屏幕及保存样本明确标注假设、未验证和执行禁用；默认195mm旧路径保留。相关10项测试、CLI帮助检查通过。未连接硬件、未改示教器TCP或I/O，不再要求用户寻找该安装配置页。

用户明确要求将全局置信度门槛降到60%，已修改config/sorting_policy.json为0.60；默认预览跟随策略，去掉帮助文本中硬编码0.80。测试覆盖0.60接受、0.59拒绝及低门槛预览不覆盖全局规则，109项测试通过（使用工作区内临时目录，避开系统临时目录权限问题）。最新塑料瓶65.36%及海绵85.56%样本按新策略离线复核，视觉检查均通过；报告data/task_reviews/20261023_164335_384851/report.md，门槛变更记录data/vision_evaluation/policy_change_20261023_60_percent.json。原始采集记录未改写，物理TCP、外参、轨迹阻塞及机器人执行禁用保留。重启实时程序用sorting_preview.py --observe-stability，不再带0.45预览覆盖参数。


2026-10-26：用户确认TCP=0继续人工示教，海绵/塑料瓶/铝罐/纸杯各完成一组6个端点，合计24个源哈希与快照核对通过，原关节弧度不做整圈归一化。四类固定摆位记录见data/manual_sessions/20261026_four_object_summary.md，新索引20261026_four_object_endpoint_index.json。离线规划输出data/planning_inputs/20261026_four_objects_tcp_zero，便携包planning_inputs_four_objects_tcp_zero_20261026.zip，30份源证据副本，独立解压校验24点通过；14项相关测试通过。旧两物体包和状态机交接包保留。铝罐撤离阶段发生过保护停机，之后正常状态补录成功；根因尚未记录，事件随新版规划候选携带incident_review_required，不因后续正常而删除。四组人工流程不等于已验证连续轨迹/自动分拣/重复性；固定摆位注册、模型碰撞、实际起始状态及实机执行仍待后续接入，自动执行和回放未启用。
