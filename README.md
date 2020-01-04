# CalibrateCamera_PinholeModel
> 基于 opencv 2.4.10
## 使用
```
-p  <标定图片dirpath>
    <图片数量>
    <图片format>
    <角点列数>
    <角点行数>
```
## 主要流程
1. 导入棋盘chart<br>
![原图](https://github.com/Narthill/CalibrateCamera_PinholeModel/raw/master/5.jpg)
2. findChessboardCorners寻找棋盘角点<br>
![角点捕获](https://github.com/Narthill/CalibrateCamera_PinholeModel/raw/master/%E8%A7%92%E7%82%B9%E6%8D%95%E8%8E%B7.PNG)
3. calibrateCamera标定, 获取内参
4. projectPoints计算原2d点和重投影2d点的误差
5. undistort 畸变矫正<br>
![角点捕获](https://github.com/Narthill/CalibrateCamera_PinholeModel/raw/master/%E7%9F%AB%E6%AD%A3%E5%90%8E.PNG)