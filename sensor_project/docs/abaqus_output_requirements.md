# Abaqus 输出说明

## 本次实际保留

每个工况目录包含 `case.json`、`submission.json`、CAE、INP、ODB、`.sta`、`.msg`、`.dat`、`run.log` 及 datacheck 的小型日志。完成状态以 `.sta` 中 `ANALYSIS HAS COMPLETED SUCCESSFULLY` 为准。

当前场输出包含全模型节点 U、RF，单元 S、LE，接触 CSTRESS、CDISP、CSTATUS；硅胶内部和外部节点位移均可从 ODB 读取。初始节点坐标加 U 得到变形坐标，不要只导出可视化表面。

历史输出包含压头 RP 位移/反力、底部 RP 反力、六个磁铁 RP 的 U1–U3 和 UR1–UR3。磁铁位姿导出工具会要求 ODB 中相应 RP 的 U/UR 场输出存在；若不满足则明确报错，不生成替代旋转。

本次没有剪切结果；没有完整求解 restart 文件。datacheck 产生的中间二进制文件不是完整求解的 restart 状态，不随交付保留。

## 后续分析建议

保留全体硅胶节点的编号、初始坐标、拓扑和各帧 U，供三维变形分析或 POD/PCA 使用。需要接触合力和完整能量验收时，进一步核对实际输出，按需补充 CFORCE、CNAREA、CTANDIR、ALLIE、ALLSE、ALLWK、ALLFD 及稳定化相关能量。

求解完成后仍需检查接触穿透、反力平衡、局部应变、网格畸变、警告内容和输出完整性。当前脚本的输出设置不能当作所有研究用途的完整验收结果。