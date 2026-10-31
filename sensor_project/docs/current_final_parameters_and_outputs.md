# 本次参数与结果

## 模型与参数

- 传感器：v2，原点为全局原点；几何长度使用 mm，力使用 N，应力使用 MPa。
- 硅胶：`geometry/v2_silicone.step`，Neo-Hookean，C10=0.0265 MPa，D1=0.39 MPa^-1。
- 网格：硅胶种子尺寸 1 mm，四面体 C3D10H。该网格已完成本轮求解，尚不代表通过网格收敛验证。
- 磁铁：六个直径 3 mm、高 2 mm 的空腔，位置和轴向来自 `config/magnets.json`；各空腔通过运动学耦合连接到对应磁铁中心 RP。
- 接触：有限滑移、Hard normal contact、允许分离；各向同性 penalty 摩擦，μ=0.5，弹性滑移容差比例 0.005。
- 压头：六组直径 8 mm 的解析半球；三组宽 25 mm 的解析平板，表面法向角为 −45°、0°、+45°。压头由各自 RP 驱动。
- 加载：先闭合 3 mm 初始间隙，再从首次接触位置压缩到 0.5、1、1.5、2、2.5、3 mm，共六个 compression 步；压头转动受限。本轮没有 shear 步骤。
- 求解：Abaqus/Standard，几何非线性开启。每个 job 使用 4 CPU，内存上限 10%。

## 已完成工况

九个 job 均在 `.sta` 文件中记录 `THE ANALYSIS HAS COMPLETED SUCCESSFULLY`，压缩到接触后 3 mm，错误数为 0。耗时和警告数量来自 `results/compression_summary.csv`。

| 工况        | 类型/法向角        | Y/mm | 耗时/分钟 | 警告数 |
| ----------- | ------------------ | ---: | --------: | -----: |
| S8_Nm45_Ym4 | 8 mm 半球，−45°  |  −4 |     38.72 |      3 |
| S8_Nm45_Yp4 | 8 mm 半球，−45°  |    4 |     39.08 |      3 |
| S8_Np00_Ym4 | 8 mm 半球，0°     |  −4 |     38.83 |      4 |
| S8_Np00_Yp4 | 8 mm 半球，0°     |    4 |     40.22 |      4 |
| S8_Np45_Ym4 | 8 mm 半球，+45°   |  −4 |     38.13 |      3 |
| S8_Np45_Yp4 | 8 mm 半球，+45°   |    4 |     38.82 |      4 |
| P25_Nm45_Y0 | 25 mm 平板，−45° |    0 |     20.58 |      5 |
| P25_Np00_Y0 | 25 mm 平板，0°    |    0 |     49.08 |     21 |
| P25_Np45_Y0 | 25 mm 平板，+45°  |    0 |     20.17 |      3 |

求解成功不表示所有警告均已确认，也不表示网格、接触物理或磁场结果已验证。`.msg` 文件保留用于逐条核查警告。

## 交付内容与限制

- `abaqus_output/analytic_rigid_cavities_mu05_cpus4/`：每例保留 CAE、INP、case/submission JSON 及可用的求解诊断文本。
- `results/compression_summary.csv`：九例状态、耗时、警告、ODB 本地路径及 restart 标记。
- `results/comparison_summary.csv`：无空腔对照的精简数据；对照使用 1 CPU、带空腔工况使用 4 CPU，因此耗时不能用于隔离空腔开销。
- `results/P25_Np00_Y0_mag_poses.csv`：0° 平板 106 帧磁铁位姿；`*_endpoints.csv` 为初末帧 Hall 计算示例，不是完整 Hall 时间序列。
- 本轮只完成 compression，没有 shear，也没有可直接继续求解的完整 restart 数据。
- GitHub 交付副本不包含本地约 7.15 GiB 的 ODB/STT/MDL/SIM 数据库。CAE 可用于检查模型，INP 可供复核或重新提交；若要在 Visualization 查看变形场，必须另行传递 ODB。
