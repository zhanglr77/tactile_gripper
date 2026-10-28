# 磁铁位姿与 Hall 计算

Hall 位置、局部坐标轴来自 `config/hall_sensors.json`；圆柱磁铁尺寸、初始位姿和 Br 来自 `config/magnets.json`。Br 默认 850 mT，可按实验校准。Hall 轴为局部 X=全局 −Y、局部 Y=全局 +X、局部 Z=全局 +Z。

从项目根目录导出一个工况的磁铁位姿，例如：

```powershell
abaqus python scripts/export_magnet_poses.py abaqus_output/analytic_rigid_cavities_mu05_cpus4/P25_Np00_Y0/Job-P25_Np00_Y0.odb results/P25_Np00_Y0_mag_poses.csv --config config/magnets.json
```

工具从磁铁 RP 读取 U 和 UR。位置为初始中心加 U，UR 单位为弧度，不能当作欧拉角逐轴直接叠加。若缺失节点集或场输出，工具会报错；需先核对 ODB。

然后使用带 NumPy 的 Python 3：

```powershell
python scripts/calculate_hall.py results/P25_Np00_Y0_mag_poses.csv results/P25_Np00_Y0_hall.csv --magnets config/magnets.json --hall config/hall_sensors.json
```

`magnet_field.py` 随脚本打包，使用有限圆柱均匀磁化表面磁荷积分模型，不依赖项目外目录。各磁铁贡献在全局叠加，再转换到 Hall 局部轴。输出磁场单位 mT，保留两位小数，并记录相对首帧的变化。

已验证 0° 平板导出链路：`results/P25_Np00_Y0_mag_poses.csv` 包含 106 帧位姿；`P25_Np00_Y0_pose_endpoints.csv` 和 `P25_Np00_Y0_hall_endpoints.csv` 仅包含初始帧及最终压缩帧，用于检查流程。尚未批量计算九组完整 Hall 轨迹，端点示例不能当作完整时间序列。

磁场模型、Br 和真实传感器坐标仍需要实验验证；仿真收敛不等于 real2sim 一致。命令中的 `python` 也可替换为已安装且包含 NumPy 的 `abaqus python`。
