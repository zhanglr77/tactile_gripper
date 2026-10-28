# v2 有限元压缩仿真项目

这是 v2 磁触觉传感器的 Abaqus/Standard 有限元项目，可整体复制到其他电脑使用。先阅读[项目说明](docs/README.md)。项目脚本根据自身位置定位配置和几何文件，不依赖固定盘符或用户名。

## 本次压缩工况

共九组，均完成从接触到 3 mm 的 compression：六组直径 8 mm 的解析半球压头，以及 −45°、0°、+45° 三组解析平板。模型包含六个磁铁空腔及磁铁中心 RP 耦合，摩擦系数为 0.5，每个 job 使用 4 CPU。各工况 `.sta` 文件均记录 `THE ANALYSIS HAS COMPLETED SUCCESSFULLY`，错误数为 0。警告仍需结合 `.msg` 逐条检查；求解完成不代表网格收敛或物理结果已验证。

工况、耗时和警告数见 `results/compression_summary.csv`。本次只完成 compression，没有 shear 结果；现有结果也没有可直接用于 shear 的求解 restart 数据。Hall 后处理工具和 0° 平板的位姿示例已提供，但尚未批量计算九组完整 Hall 轨迹。

## 目录

- `config/`：传感器、硅胶磁铁、Hall 位置与压头工况 JSON。
- `geometry/`：硅胶 STEP 源几何。
- `scripts/`：建模、工况准备、状态检查、磁铁位姿导出和 Hall 磁场计算。
- `abaqus_output/analytic_rigid_cavities_mu05_cpus4/`：九组 CAE、INP 和求解文本记录。
- `results/`：压缩汇总、对比数据和位姿/Hall 示例。
- `docs/`：中文建模、求解和结果使用说明。

GitHub 交付副本不包含 ODB、STT、MDL、SIM 等大型求解数据库（本地合计约 7.15 GiB）。因此远端副本可用于查看 CAE 模型、检查输入文件或重新运行，但不能直接在 Visualization 中打开原始场结果。需要后处理时，请另行保存或传递本地 ODB。

环境：Abaqus 2026；Hall 后处理使用 Python 3 和 NumPy。`scripts/check_abaqus_status.cmd` 只检查状态，不会提交作业。
