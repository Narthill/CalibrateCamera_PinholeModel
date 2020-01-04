# CalibrateCamera_PinholeModel
> 基于 opencv 2.4.10
## 使用
编译完成后
```
-p  <标定图片dirpath>
    <图片数量>
    <图片format>
    <角点列数>
    <角点行数>
```
## 主要流程
1. 导入棋盘chart
2. findChessboardCorners寻找棋盘角点
3. calibrateCamera标定, 获取内参
4. projectPoints计算原2d点和重投影2d点的误差