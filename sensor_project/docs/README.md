# 项目说明

本项目包含 v2 传感器的九组解析刚体压头 compression 工况，全部完成到接触后 3 mm。工况包括六种球头位置/角度组合和三种平板法向角（−45°、0°、+45°），摩擦系数均为 0.5，六个磁铁空腔通过各自中心 RP 耦合。

## 文件与使用顺序

1. 在 `config/sensor.json` 查看坐标原点和单位；硅胶源几何位于 `geometry/v2_silicone.step`。
2. `config/magnets.json` 定义六个磁铁的位置、轴向、尺寸和剩磁；`config/hall_sensors.json` 定义四个 Hall 的位置和局部坐标轴。
3. `config/indenter_cases.json` 定义九组压头和共享加载参数。
4. 在 `abaqus_output/analytic_rigid_cavities_mu05_cpus4/<case_id>/` 打开 `<case_id>_complete.cae` 查看模型，或检查 `Job-<case_id>.inp`。GitHub 副本不包含 ODB，不能直接查看变形场和接触场。
5. `results/compression_summary.csv` 记录九组完成状态、耗时、警告数和本地 ODB 路径；`comparison_summary.csv` 是无空腔对照统计。对照模型和当前模型的 CPU 数不同，不能用耗时比例推断空腔的纯计算开销。
6. 双击 `scripts/check_abaqus_status.cmd` 查看项目目录中的求解状态，不会提交作业。

## 文档导航

| 文档 | 内容 |
|---|---|
| `current_final_parameters_and_outputs.md` | 实际模型、九组结果、交付文件和限制 |
| `analytical_rigid_contact.md` | 解析半球、平板、接触面和 RP 设置 |
| `indenter_parameterization_plan.md` | 工况 JSON 字段和局部加载方向 |
| `abaqus_output_requirements.md` | 当前输出和后续扩展建议 |
| `restart_acceleration_plan.md` | 重启动现状与后续剪切流程 |
| `HALL_WORKFLOW.md` | 磁铁位姿导出和 Hall 计算命令 |
| `final_simulation_checklist.md` | 建模、提交和结果验收检查 |
| `contact_friction_stabilization_plan.md` | 接触、摩擦和收敛检查 |
| `surrogate_plan.md` | 形变降维与 real2sim 数据使用 |

## 路径约定

脚本通过自身位置定位项目根目录，不写死盘符或用户名。准备脚本的 `OutputRoot`、提交脚本的 `RunRoot`、状态脚本的 `Root` 若为相对路径，均相对于项目根目录解析。运行时生成的绝对路径只用于定位本次文件，不会写入配置。

Python 后处理默认读取项目内的 `config/magnets.json` 和 `config/hall_sensors.json`。显式指定的 ODB、CSV 和配置路径按启动命令时的当前工作目录解析。建议从项目根目录运行后处理命令。单工况建模需从包含 `case.json` 的工况目录运行，以选择工况并将输出留在该目录；`../../../scripts/build_case.py` 是项目内相对路径。

## 重新建模

默认结果目录已有完成工况，不要覆盖。需要新试验时，先在项目根目录准备独立输出目录：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/prepare_compression_cases.ps1 -OutputRoot abaqus_output/new_compression -CaseId P25_Np00_Y0 -FrictionCoefficient 0.5
```

进入准备的工况目录后执行 `abaqus cae noGUI=../../../scripts/build_case.py`，生成 CAE/INP。datacheck 通过后再提交。`submit_compression_baselines.ps1 -RunRoot abaqus_output/new_compression` 可提交准备好的 compression 工况（每例 4 CPU），默认跳过已成功完成的 job。

当前结果没有 shear，也没有可直接用于后续 shear 的完整求解 restart 数据。Hall 工具可用于后处理，但目前没有九组完整 Hall 轨迹。传感器物理正确性不能只由 `COMPLETED` 判定，还需检查接触、反力、网格收敛和实验对比。

## 输出文件说明

GitHub 交付副本保留 CAE、INP、状态/诊断文本和汇总 CSV；不上传 ODB、STT、MDL、SIM 等大型求解数据库。本地完整 ODB 可在各工况目录中查看，但需通过单独存储介质或文件传输提供给需要查看场结果的人。
