# 解析刚体与接触

本项目所有压头采用 Abaqus Analytical Rigid Surface，硅胶仍为可变形网格。压头不生成实体单元，由 RP 控制位移和转动。

球头由圆弧旋转生成解析半球，半径为 `diameter_mm/2`，下极朝向硅胶。轮廓极点采用 0.001 mm 轴向偏移处理，实际形状和接触覆盖范围应以 CAE/INP 为准。

平板由直线轮廓挤出生成；轮廓宽度有限，沿挤出方向无限延伸。`length_mm` 只控制 CAE 显示深度，不能理解为有限矩形工具的接触边界。当前包含 −45°、0°、+45° 三个平板工况。

解析刚体面是单侧面，法向须朝向被接触的硅胶，必须作为接触主面，并关联刚体 RP；硅胶作为从面。接触采用有限滑移。生成器调用 `SurfaceToSurfaceContactStd`，INP 包含 `*RIGID BODY, ANALYTICAL SURFACE=...`。

## 核查依据

Abaqus 2026 官方文档条目（文档本身未打包）：

- `SIMACAEMODRefMap/simamod-c-rigidsurf.htm`：解析刚体面定义、单侧性及圆弧限制。
- `SIMACAECAERefMap/simacae-c-prtconcrigid.htm`：CAE 刚体部件。
- `SIMACAEITNRefMap/simaitn-c-contactpair.htm`：接触对。
- `SIMAINPRefResources/hemipunch_anl.inp`：解析压头示例。

本次九例均通过 datacheck 和 3 mm compression。检查日志中关于解析主面接触追踪方式的提示，以及实际接触压力、穿透和边缘变形；datacheck 不替代完整求解与物理验收。