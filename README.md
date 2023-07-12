# Awesome 3D LiDAR Datasets [![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

This repository is the collection of datasets, involving the 3D LiDAR. In this repository, the overall dataset table is represented the number and kind of LiDARs, objective of Datasets, and scale information. Objectives were not divided in detail, but were divided into Object Detection (OD), Segmentation (Seg), Odometry (Odom), Place Recognition (PR) and Localization. In addition, if data exceeding 1 km is included, it is displayed as large scale. Furthermore, Single w. Multiple LiDAR represents that the sequences consist of multiple LiDAR; however, they does not acheived simultaneously.

## Recent Update
### Update: 2023-07-12
- Firstly divide the collection of 3D LiDAR datasets.

|Dataset|Year                         |Single vs Multi|Spinning and Solid                           |Objective   |Scale|
|-------|-----------------------------|---------------|---------------------------------------------|------------|-----|
|[MIT DARPA](http://grandchallenge.mit.edu/wiki/index.php?title=PublicData)|2010                         |Single         |1 VLP                                        |Odom        |Large|
|[Ford Campus](https://robots.engin.umich.edu/SoftwareData/InfoFord)|2011                         |Single         |1 VLP                                        |Odom        |Large|
|[KITTI](https://www.cvlibs.net/datasets/kitti/)  |2013                         |Single         |1 VLP                                        |Odom        |Large|
|[NCLT](http://robots.engin.umich.edu/nclt/)   |2017                         |Single         |1 VLP                                        |Odom        |Both |
|[Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset)|2019                         |Multi          |2 VLP                                        |Odom        |Large|
|[Toronto-3D](https://github.com/WeikaiTan/Toronto-3D)|2020                         |Multi          |1 Teledyne, 1 Optech, 1 Maverick             |Seg         |Large|
|[Apollo-DaoxiangLake Dataset](https://developer.apollo.auto/daoxianglake.html)|2020                         |Single         |1 VLP                                        |Localization|Large|
|[Apollo-SouthBay Dataset](https://developer.apollo.auto/southbay.html)|2020                         |Single         |1 VLP                                        |Localization|Large|
|[MulRan](https://sites.google.com/view/mulran-pr/dataset) |2020                         |Single         |1 Ouster                                     |PR, Odom    |Large|
|[The Oxford Radar RobotCar Dataset](https://oxford-robotics-institute.github.io/radar-robotcar-dataset/)|2020                         |Multi          |2 VLP                                        |Odom, PR    |Large|
|[Newer College Dataset](https://ori-drs.github.io/newer-college-dataset/)|2020                         |Single         |1 Ouster                                     |Odom        |Small|
|[nuScences](https://www.nuscenes.org/)|2020                         |Single         |1 VLP                                        |OD          |Large|
|[Ford AV Dataset](https://avdata.ford.com/)|2020                         |Multi          |4 VLP                                        |Odom        |Large|
|[LIBRE](https://sites.google.com/g.sp.m.is.nagoya-u.ac.jp/libre-dataset)  |2020                         |Single w. Multiple LiDAR|12 Spinning (each)                           |Odom        |Large|
|[EU Long-term Dataset](https://sites.google.com/g.sp.m.is.nagoya-u.ac.jp/libre-dataset)|2021                         |Single         |1 VLP                                        |Odom        |Large|
|[NTU VIRAL Dataset](https://ntu-aris.github.io/ntu_viral_dataset/)|2021                         |Multi          |2 Ouster                                     |Odom        |Small|
|[M2DGR](https://github.com/SJTU-ViSYS/M2DGR)  |2021                         |Single         |1 VLP                                        |Odom        |Large|
|[Pandaset](https://pandaset.org/)|2021                         |Multi          |2 Hesai(Spinning, Solid)                     |Seg         |Large|
|[UrbanNav Dataset](https://github.com/IPNL-POLYU/UrbanNavDataset)|2021                         |Multi          |2 Velodyne, 1 Leishen                        |Odom        |Large|
|[Hilti 2021 SLAM dataset](https://hilti-challenge.com/dataset-2021.html)|2021                         |Multi          |1 Ouster, 1 Livox                            |Odom        |Small|
|[S3LI Dataset](https://www.dlr.de/rm/en/s3li_dataset/#gallery/37227)|2022                         |Single         |1 Black-filed Cube LiDAR                     |Odom        |Large|
|[STHEREO](https://sites.google.com/view/rpmsthereo/)|2022                         |Single         |1 Ouster                                     |Odom        |Large|
|[ORFD](https://github.com/chaytonmin/Off-Road-Freespace-Detection)   |2022                         |Single         |1 Hesai                                      |Seg         |Large|
|[Tiers](https://github.com/TIERS/tiers-lidars-dataset)  |2022                         |Multi          |2 Livox, 2 Ouster, 1 VLP, 1 RGBD             |Odom        |Both|
|[Hllti 2022 SLAM dataset](https://hilti-challenge.com/dataset-2022.html)|2022                         |Single         |1 Hesai                                      |Odom        |Small|
|[USTC FLICAR](https://ustc-flicar.github.io/)|2023                         |Multi          |2 VLP, 1 Ouster, 1 Livox                     |Odom        |Small|
|[WOMD Dataset](https://waymo.com/open/data/motion/)|2023                         |Single         |1 Spinning (Unknown)                         |OD          |Large|
|[Wild Places](https://csiro-robotics.github.io/Wild-Places/)|2023                         |Single         |1 VLP                                        |PR          |Large|
|[Hilti 2023 SLAM Dataset](https://hilti-challenge.com/dataset-2023.html)|2023                         |Single w. Multiple LiDAR|1 Hesai, 1 Robosense (each)                  |Odom        |Small|
|[City Dataset](https://github.com/minwoo0611/MA-LIO)|2023                         |Multi|1 Ouster, 2 Livox                  |Odom        |Large|
|[Ground-Challenge](https://github.com/sjtuyinjie/Ground-Challenge)|2023                         |Single|1 VLP                  |Odom        |Small|