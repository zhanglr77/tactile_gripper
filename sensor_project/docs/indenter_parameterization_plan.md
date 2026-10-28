# 压头参数化与工况 JSON

工况配置为 `config/indenter_cases.json`。每例必须有唯一 `case_id`、`indenter` 和 `surface_position`。

- `indenter.type=sphere`：`diameter_mm` 定义球直径，生成解析半球。
- `indenter.type=rectangular_flat`：`width_mm` 是轮廓宽度，`length_mm` 为挤出显示深度，数学接触面沿挤出方向无限延伸。
- `surface_position.normal_angle_deg`：XZ 平面中从全局 +Z 朝 +X 的硅胶外法向角度。
- `surface_position.y_mm`：接触点所在全局 Y 排。

生成器在硅胶外表面求对应位置和法向 `n`。压头先沿 `−n` 闭合初始间隙，再从首次接触位置继续挤压。当前只支持 XZ 平面法向的配置，不是通用任意曲面参数化。

共享参数 `shared_loading` 包括初始间隙 3 mm、压缩深度 3 mm、六个 0.5 mm 间隔检查点和摩擦系数 0.5。`shear_cases` 描述后续剪切意图，本次生成器不会生成或提交 shear。

准备脚本将配置写入各工况 `case.json`。`include_magnet_cavities=true` 是本次正式设置，生成六个空腔及其 RP 耦合。`-NoMagnetCavities` 仅用于诊断；该模式不输出磁铁 RP 位姿。`-FrictionCoefficient` 可单独覆盖本次准备目录的摩擦，原有配置和完成结果不应被覆盖。

当前打包配置包含九个已完成工况。新增压头前先检查 CAE 的位置、法向、覆盖范围和运动轴，再通过 datacheck。
## 当前配置扩展

配置已在七例完成结果基础上增加 P25_Nm45_Y0 和 P25_Np45_Y0（Y=0），现共九例，全部已成功完成 3 mm compression。
