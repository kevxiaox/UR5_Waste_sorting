# 统一分拣任务预览

当前门槛：按用户明确要求，全局分类置信度由0.80调为0.60；恰好0.60接受，低于0.60拒绝。区域、深度、稳定窗口与物理执行阻塞保持有效。默认预览跟随该配置，实时运行用`sorting_preview.py --observe-stability`。以下旧样本在0.80下被拦截的描述属于历史结果，原始记录不改写。

更新：稳定窗口检查独立于类别置信度检查；低置信度但已稳定的样本只报告低置信度，不再误报“等待稳定窗口”。低置信度且窗口陈旧时仍报告两个问题，执行禁用保持不变。塑料瓶样本data/sorting_previews/20261023_163430_078294置信度65.36%，20帧坐标稳定；重放结果仍拦截，核对记录见该目录review.json。相关6项任务复核测试通过。正在运行的旧窗口需重启才会采用显示修正。

主入口仍为sorting_preview.py，保留已有相机、YOLO、区域、同帧深度、候选坐标与可选20帧诊断。新增task_review显示和保存统一决策：始终重新应用全局0.60分类门槛，不让更低的预览门槛覆盖策略；核对类别/分类区一致性、区域会话确认、有效深度、当前帧对应的单物体稳定窗口。输入中即便有人把TCP或标定标志改成true，也不会自动许可执行。

无相机的离线回放：

```cmd
.\.venv\Scripts\python.exe preview_sorting_tasks.py
```

也可用--saved显式给出一个或多个preview.json，不按“最新同名记录”自动选择。输出data/task_reviews/<时间>/report.json及.md，记录来源SHA256和当前策略SHA256；不改写历史记录。

实时显示：

```cmd
.\.venv\Scripts\python.exe sorting_preview.py --observe-stability
```

保存结果新增task_review，顶部显示Task状态。vision_review_only_physical_validation_pending仅代表视觉侧条件满足；blocked_by_vision_checks表示类别、门槛、区域、深度或当前窗口存在阻塞；blocked_multiple_candidates不自动选一个。未启用连续帧观察时，会明确等待稳定窗口。

review不是执行队列、规划器或URScript。selected_target、base_xyz_m、grasp_pose、trajectory继续为空，自动分拣禁用。海绵约59mm一致性差异与195mm未验证候选保留；用户选择暂缓TCP与未知物体测试，不在此隐式解除这些限制。

离线已回放5条已保存现场记录：高置信度海绵、铝罐、纸杯通过视觉检查；低置信度海绵与塑料瓶被默认门槛拒绝。仅为软件流程回放，不是自动抓取验证。
