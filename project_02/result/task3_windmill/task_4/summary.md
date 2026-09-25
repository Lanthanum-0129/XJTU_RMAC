# task_4 跟踪小结

- 视频参数：1440x1080，30 FPS，读入 1800 帧
- 预处理后端：GPU (OpenCV CUDA)（--accel gpu），平均预处理耗时 2.87677 ms/帧
- 调色板：orange/yellow
- 检出帧 924，丢失帧 876，最长连续丢失 119 帧
- 目标形态：靶标形（同心圆+十字，内部密度≥50）与单圆形均为有效目标，同一身份从亮起跟踪至熄灭；候选由 Hough 圆环 + 环覆盖率≥0.75 验证，横幅、科技核心方形、实心发光块被覆盖率/密度判据拒绝；运动模糊时启用轮廓外接圆补充
- 中心：R 字母逐帧时序跟踪（增加跳变抑制），R 缺失时被选目标自身历史 Kasa 圆拟合兜底；角度相对当帧中心计算
- 锁定：引入匀速运动预测与综合代价关联（位置/角度/半径）；首锁连续 3 帧确认；丢失容忍 60 帧后按运动趋势与距离评分重选并递增 ID
- 事件表：

| 帧 | 事件 |
|---|---|
| frame 0 | center initialized by R-letter |
| frame 2 | first lock ID=0 (type=1), confirmed over 3 frames |
| frame 63 | reselect ID=1 after 61 lost; basis=motion & proximity |
| frame 83 | recovery of ID=1 after 2 lost |
| frame 96 | recovery of ID=1 after 2 lost |
| frame 108 | recovery of ID=1 after 1 lost |
| frame 112 | recovery of ID=1 after 3 lost |
| frame 135 | recovery of ID=1 after 1 lost |
| frame 224 | recovery of ID=1 after 59 lost |
| frame 226 | recovery of ID=1 after 1 lost |
| frame 230 | recovery of ID=1 after 2 lost |
| frame 233 | recovery of ID=1 after 1 lost |
| frame 245 | recovery of ID=1 after 7 lost |
| frame 255 | recovery of ID=1 after 2 lost |
| frame 259 | recovery of ID=1 after 1 lost |
| frame 266 | recovery of ID=1 after 2 lost |
| frame 272 | recovery of ID=1 after 1 lost |
| frame 277 | recovery of ID=1 after 1 lost |
| frame 279 | recovery of ID=1 after 1 lost |
| frame 288 | recovery of ID=1 after 6 lost |
| frame 308 | recovery of ID=1 after 2 lost |
| frame 319 | recovery of ID=1 after 3 lost |
| frame 328 | recovery of ID=1 after 3 lost |
| frame 330 | recovery of ID=1 after 1 lost |
| frame 365 | recovery of ID=1 after 34 lost |
| frame 371 | recovery of ID=1 after 1 lost |
| frame 378 | recovery of ID=1 after 2 lost |
| frame 381 | recovery of ID=1 after 2 lost |
| frame 384 | recovery of ID=1 after 1 lost |
| frame 388 | recovery of ID=1 after 3 lost |
| frame 392 | recovery of ID=1 after 2 lost |
| frame 396 | recovery of ID=1 after 3 lost |
| frame 402 | recovery of ID=1 after 4 lost |
| frame 404 | recovery of ID=1 after 1 lost |
| frame 410 | recovery of ID=1 after 5 lost |
| frame 413 | recovery of ID=1 after 2 lost |
| frame 417 | recovery of ID=1 after 3 lost |
| frame 423 | recovery of ID=1 after 4 lost |
| frame 456 | recovery of ID=1 after 32 lost |
| frame 465 | recovery of ID=1 after 1 lost |
| frame 478 | recovery of ID=1 after 1 lost |
| frame 487 | recovery of ID=1 after 5 lost |
| frame 492 | recovery of ID=1 after 4 lost |
| frame 497 | recovery of ID=1 after 1 lost |
| frame 526 | recovery of ID=1 after 1 lost |
| frame 609 | recovery of ID=1 after 79 lost |
| frame 615 | recovery of ID=1 after 1 lost |
| frame 654 | recovery of ID=1 after 20 lost |
| frame 689 | recovery of ID=1 after 17 lost |
| frame 728 | recovery of ID=1 after 28 lost |
| frame 730 | recovery of ID=1 after 1 lost |
| frame 732 | recovery of ID=1 after 1 lost |
| frame 734 | recovery of ID=1 after 1 lost |
| frame 737 | recovery of ID=1 after 1 lost |
| frame 857 | recovery of ID=1 after 119 lost |
| frame 862 | recovery of ID=1 after 3 lost |
| frame 911 | recovery of ID=1 after 48 lost |
| frame 948 | recovery of ID=1 after 1 lost |
| frame 966 | recovery of ID=1 after 1 lost |
| frame 978 | recovery of ID=1 after 4 lost |
| frame 1013 | recovery of ID=1 after 1 lost |
| frame 1024 | recovery of ID=1 after 1 lost |
| frame 1027 | recovery of ID=1 after 1 lost |
| frame 1045 | recovery of ID=1 after 1 lost |
| frame 1053 | recovery of ID=1 after 1 lost |
| frame 1079 | recovery of ID=1 after 1 lost |
| frame 1091 | recovery of ID=1 after 1 lost |
| frame 1118 | recovery of ID=1 after 1 lost |
| frame 1134 | recovery of ID=1 after 1 lost |
| frame 1219 | recovery of ID=1 after 6 lost |
| frame 1222 | recovery of ID=1 after 2 lost |
| frame 1227 | recovery of ID=1 after 4 lost |
| frame 1231 | recovery of ID=1 after 3 lost |
| frame 1233 | recovery of ID=1 after 1 lost |
| frame 1238 | recovery of ID=1 after 3 lost |
| frame 1245 | recovery of ID=1 after 6 lost |
| frame 1250 | recovery of ID=1 after 4 lost |
| frame 1256 | recovery of ID=1 after 3 lost |
| frame 1259 | recovery of ID=1 after 2 lost |
| frame 1263 | recovery of ID=1 after 2 lost |
| frame 1267 | recovery of ID=1 after 3 lost |
| frame 1270 | recovery of ID=1 after 2 lost |
| frame 1273 | recovery of ID=1 after 1 lost |
| frame 1276 | recovery of ID=1 after 2 lost |
| frame 1284 | recovery of ID=1 after 4 lost |
| frame 1287 | recovery of ID=1 after 2 lost |
| frame 1294 | recovery of ID=1 after 6 lost |
| frame 1297 | recovery of ID=1 after 2 lost |
| frame 1299 | recovery of ID=1 after 1 lost |
| frame 1301 | recovery of ID=1 after 1 lost |
| frame 1305 | recovery of ID=1 after 2 lost |
| frame 1313 | recovery of ID=1 after 7 lost |
| frame 1318 | recovery of ID=1 after 4 lost |
| frame 1322 | recovery of ID=1 after 1 lost |
| frame 1325 | recovery of ID=1 after 2 lost |
| frame 1359 | recovery of ID=1 after 33 lost |
| frame 1364 | recovery of ID=1 after 3 lost |
| frame 1382 | recovery of ID=1 after 17 lost |
| frame 1389 | recovery of ID=1 after 6 lost |
| frame 1392 | recovery of ID=1 after 2 lost |
| frame 1423 | recovery of ID=1 after 1 lost |
| frame 1441 | recovery of ID=1 after 2 lost |
| frame 1461 | recovery of ID=1 after 1 lost |
| frame 1464 | recovery of ID=1 after 1 lost |
| frame 1537 | recovery of ID=1 after 1 lost |
| frame 1548 | recovery of ID=1 after 6 lost |
| frame 1551 | recovery of ID=1 after 1 lost |
| frame 1559 | recovery of ID=1 after 5 lost |
| frame 1562 | recovery of ID=1 after 2 lost |
| frame 1575 | recovery of ID=1 after 12 lost |
| frame 1578 | recovery of ID=1 after 2 lost |
| frame 1582 | recovery of ID=1 after 3 lost |
| frame 1591 | recovery of ID=1 after 7 lost |
| frame 1595 | recovery of ID=1 after 3 lost |
| frame 1597 | recovery of ID=1 after 1 lost |
| frame 1600 | recovery of ID=1 after 1 lost |
| frame 1602 | recovery of ID=1 after 1 lost |
| frame 1606 | recovery of ID=1 after 3 lost |
| frame 1609 | recovery of ID=1 after 2 lost |
| frame 1612 | recovery of ID=1 after 2 lost |
| frame 1620 | recovery of ID=1 after 1 lost |
| frame 1623 | recovery of ID=1 after 2 lost |
| frame 1627 | recovery of ID=1 after 3 lost |
| frame 1633 | recovery of ID=1 after 1 lost |
| frame 1637 | recovery of ID=1 after 2 lost |
| frame 1643 | recovery of ID=1 after 1 lost |
| frame 1649 | recovery of ID=1 after 1 lost |
| frame 1682 | recovery of ID=1 after 25 lost |
| frame 1688 | recovery of ID=1 after 5 lost |
| frame 1697 | recovery of ID=1 after 7 lost |
| frame 1702 | recovery of ID=1 after 4 lost |
| frame 1704 | recovery of ID=1 after 1 lost |
| frame 1707 | recovery of ID=1 after 2 lost |
| frame 1713 | recovery of ID=1 after 3 lost |
| frame 1715 | recovery of ID=1 after 1 lost |
| frame 1719 | recovery of ID=1 after 2 lost |
| frame 1723 | recovery of ID=1 after 3 lost |
| frame 1735 | recovery of ID=1 after 2 lost |
| frame 1738 | recovery of ID=1 after 1 lost |
| frame 1742 | recovery of ID=1 after 2 lost |
| frame 1744 | recovery of ID=1 after 1 lost |
| frame 1746 | recovery of ID=1 after 1 lost |

- 视频：recognition_overlay.mp4、binary_process.mp4（本目录）
