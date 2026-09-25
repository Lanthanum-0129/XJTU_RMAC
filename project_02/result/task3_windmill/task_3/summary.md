# task_3 跟踪小结

- 视频参数：1440x1080，30 FPS，读入 796 帧
- 预处理后端：GPU (OpenCV CUDA)（--accel gpu），平均预处理耗时 2.93449 ms/帧
- 调色板：orange/yellow
- 检出帧 434，丢失帧 362，最长连续丢失 60 帧
- 目标形态：靶标形（同心圆+十字，内部密度≥50）与单圆形均为有效目标，同一身份从亮起跟踪至熄灭；候选由 Hough 圆环 + 环覆盖率≥0.75 验证，横幅、科技核心方形、实心发光块被覆盖率/密度判据拒绝；运动模糊时启用轮廓外接圆补充
- 中心：R 字母逐帧时序跟踪（增加跳变抑制），R 缺失时被选目标自身历史 Kasa 圆拟合兜底；角度相对当帧中心计算
- 锁定：引入匀速运动预测与综合代价关联（位置/角度/半径）；首锁连续 3 帧确认；丢失容忍 60 帧后按运动趋势与距离评分重选并递增 ID
- 事件表：

| 帧 | 事件 |
|---|---|
| frame 10 | center initialized by R-letter |
| frame 198 | first lock ID=0 (type=1), confirmed over 3 frames |
| frame 259 | reselect ID=1 after 61 lost; basis=motion & proximity |
| frame 271 | recovery of ID=1 after 1 lost |
| frame 273 | recovery of ID=1 after 1 lost |
| frame 278 | recovery of ID=1 after 1 lost |
| frame 316 | recovery of ID=1 after 1 lost |
| frame 334 | recovery of ID=1 after 1 lost |
| frame 343 | recovery of ID=1 after 1 lost |
| frame 349 | recovery of ID=1 after 2 lost |
| frame 351 | recovery of ID=1 after 1 lost |
| frame 360 | recovery of ID=1 after 1 lost |
| frame 364 | recovery of ID=1 after 1 lost |
| frame 371 | recovery of ID=1 after 2 lost |
| frame 375 | recovery of ID=1 after 1 lost |
| frame 381 | recovery of ID=1 after 1 lost |
| frame 383 | recovery of ID=1 after 1 lost |
| frame 385 | recovery of ID=1 after 1 lost |
| frame 391 | recovery of ID=1 after 5 lost |
| frame 394 | recovery of ID=1 after 1 lost |
| frame 401 | recovery of ID=1 after 1 lost |
| frame 403 | recovery of ID=1 after 1 lost |
| frame 405 | recovery of ID=1 after 1 lost |
| frame 407 | recovery of ID=1 after 1 lost |
| frame 412 | recovery of ID=1 after 2 lost |
| frame 421 | recovery of ID=1 after 1 lost |
| frame 425 | recovery of ID=1 after 1 lost |
| frame 427 | recovery of ID=1 after 1 lost |
| frame 429 | recovery of ID=1 after 1 lost |
| frame 431 | recovery of ID=1 after 1 lost |
| frame 433 | recovery of ID=1 after 1 lost |
| frame 435 | recovery of ID=1 after 1 lost |
| frame 438 | recovery of ID=1 after 2 lost |
| frame 441 | recovery of ID=1 after 2 lost |
| frame 443 | recovery of ID=1 after 1 lost |
| frame 449 | recovery of ID=1 after 3 lost |
| frame 454 | recovery of ID=1 after 4 lost |
| frame 459 | recovery of ID=1 after 2 lost |
| frame 470 | recovery of ID=1 after 4 lost |
| frame 472 | recovery of ID=1 after 1 lost |
| frame 482 | recovery of ID=1 after 9 lost |
| frame 488 | recovery of ID=1 after 5 lost |
| frame 493 | recovery of ID=1 after 2 lost |
| frame 496 | recovery of ID=1 after 2 lost |
| frame 500 | recovery of ID=1 after 1 lost |
| frame 503 | recovery of ID=1 after 1 lost |
| frame 507 | recovery of ID=1 after 1 lost |
| frame 512 | recovery of ID=1 after 1 lost |
| frame 520 | recovery of ID=1 after 1 lost |
| frame 530 | recovery of ID=1 after 3 lost |
| frame 533 | recovery of ID=1 after 2 lost |
| frame 535 | recovery of ID=1 after 1 lost |
| frame 547 | recovery of ID=1 after 2 lost |
| frame 549 | recovery of ID=1 after 1 lost |
| frame 563 | recovery of ID=1 after 1 lost |
| frame 567 | recovery of ID=1 after 2 lost |
| frame 573 | recovery of ID=1 after 1 lost |
| frame 581 | recovery of ID=1 after 1 lost |
| frame 583 | recovery of ID=1 after 1 lost |
| frame 593 | recovery of ID=1 after 1 lost |
| frame 600 | recovery of ID=1 after 1 lost |

- 视频：recognition_overlay.mp4、binary_process.mp4（本目录）
