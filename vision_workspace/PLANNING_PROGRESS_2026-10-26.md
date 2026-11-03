# TCP=0 人工示教后的软件对接进度

用户已确认继续以TCP=0人工示教；当前约定保存在config/teaching_convention.json。海绵右侧、塑料瓶左侧各6个明确来源的关节端点，全部实际记录为零TCP。保留原始关节整圈数，不切换到临时均值TCP，不复用两物体的抓取高度。

新增waste_sorting/endpoint_catalog.py、prepare_joint_references.py，将已保存的端点索引整理为带来源哈希的离线关节目标参考。逐项检查源快照、角色、分类区、配置版本、全零TCP和关节值；不按同名最新记录自动选择，篡改或不一致即拒绝。13项相关测试通过，实际12个端点导出核对通过。

输出目录data/planning_inputs/20261026_tcp_zero：catalog.json、两物体requests.json、中文README、英文INTEGRATION_EN.md、原始证据副本、portable_sources.json和标准库校验脚本verify_inputs.py。校验器可在另一台电脑上用python3 verify_inputs.py运行，不依赖ROS、相机SDK或设备连接。

这些是“规划器的候选输入”，不是已经规划的轨迹。第一段起始关节状态为空，后续段起点仅是历史参考；实际执行前仍需要实时起始状态和环境/夹爪/带载物体模型。固定摆放的物理注册尚未验证，不能仅凭视觉类别就执行固定抓取点。其它类别没有兜底动作计划。

用户反馈已有成员正在搭建ROS/MoveIt环境，本机Windows环境不安装或迁移ROS。下一阶段等待该成员提供Ubuntu、ROS2、MoveIt及UR驱动版本、UR5模型启动方式、关节名/前缀及URSim/mock模式，先做plan-only离线规划。状态机原交接包不改动，通过独立适配层接入。没有连接机器人、写I/O、解除执行禁用或自动回放。
