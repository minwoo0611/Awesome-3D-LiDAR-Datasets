# Awesome 3D LiDAR Datasets [![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

This repository is the collection of datasets, involving the 3D LiDAR. The information is presented in a comprehensive table, outlining the type and number of LiDARs, the purpose of each dataset, and scale details. The objectives are broadly categorized into Object Detection (OD), Segmentation (Seg), Odometry (Odom), Place Recognition (PR), and Localization (Loc). If a dataset includes data exceeding 1 km, it is classified as large scale. Datasets that use multiple LiDAR sequences, even if not executed concurrently, are labeled as 'Single w. Multiple LiDAR'.

## Most Recent Update
### Update: 2023-07-13
- The table includes the specific LiDAR products utilized in each dataset, involving the its channels or name.
### Update: 2023-07-12
- Initial segregation of the 3D LiDAR dataset collection.

The table below summarizes the details of each dataset:

|Dataset|Year|Single vs Multi|Spinning LiDAR|Solid State LiDAR|Objective|Scale|
|---|---|---|---|---|---|---|
|[MIT DARPA](http://grandchallenge.mit.edu/wiki/index.php?title=PublicData)|2010|Single|1x HDL-64E|No|Odom|Large|
|[Ford Campus](https://robots.engin.umich.edu/SoftwareData/InfoFord)|2011|Single|1x HDL-64E|No|Odom|Large|
|[KITTI](https://www.cvlibs.net/datasets/kitti/)|2013|Single|1x HDL-64E|No|Odom|Large|
|[NCLT](http://robots.engin.umich.edu/nclt/)|2017|Single|1x HDL-32E|No|Odom|Both|
|[Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset)|2019|Multi|2x VLP-16C|No|Odom|Large|
|[Toronto-3D](https://github.com/WeikaiTan/Toronto-3D)|2020|Multi|1x Teledyne Optech Maverick (32 Channels)|No|Seg|Large|
|[Apollo-DaoxiangLake Dataset](https://developer.apollo.auto/daoxianglake.html)|2020|Single|1x HDL-64E|No|Loc|Large|
|[Apollo-SouthBay Dataset](https://developer.apollo.auto/southbay.html)|2020|Single|1x HDL-64E|No|Loc|Large|
|[MulRan](https://sites.google.com/view/mulran-pr/dataset)|2020|Single|1x OS1-64|No|PR, Odom|Large|
|[The Oxford Radar RobotCar Dataset](https://oxford-robotics-institute.github.io/radar-robotcar-dataset/)|2020|Multi|2x HDL-32E|No|Odom, PR|Large|
|[Newer College Dataset](https://ori-drs.github.io/newer-college-dataset/)|2020|Single|1x OS1-64|No|Odom|Small|
|[nuScences](https://www.nuscenes.org/)|2020|Single|1x HDL-32E|No|OD|Large|
|[Ford AV Dataset](https://avdata.ford.com/)|2020|Multi|4x HDL-32E|No|Odom|Large|
|[LIBRE](https://sites.google.com/g.sp.m.is.nagoya-u.ac.jp/libre-dataset)|2020|Single w. Multiple LiDAR|12x Spinning (each)|No|Odom|Large|
|[EU Long-term Dataset](https://epan-utbm.github.io/utbm_robocar_dataset/)|2021|Multi|2x HDL-32E|No|Odom|Large|
|[NTU VIRAL Dataset](https://ntu-aris.github.io/ntu_viral_dataset/)|2021|Multi|2x OS1-16|No|Odom|Small|
|[M2DGR](https://github.com/SJTU-ViSYS/M2DGR)|2021|Single|1x VLP-32C|No|Odom|Large|
|[Pandaset](https://pandaset.org/)|2021|Multi|1x Pandar64|1x PandarGT|Seg|Large|
|[UrbanNav Dataset](https://github.com/IPNL-POLYU/UrbanNavDataset)|2021|Multi|1x HDL-32E, 1x VLP-16C, 1x Lslidar C16|No|Odom|Large|
|[Livox Simu-Dataset](https://www.livoxtech.com/simu-dataset)|2021|Multi|No|5x Livox Horizon, 1x Livox Tele|OD, Seg|Large|
|[Hilti 2021 SLAM dataset](https://hilti-challenge.com/dataset-2021.html)|2021|Multi|1x OS0-64|1x Livox MID70|Odom|Small|
|[S3LI Dataset](https://www.dlr.de/rm/en/s3li_dataset/#gallery/37227)|2022|Single|No|1x Black-filed Cube LiDAR|Odom|Large|
|[STHEREO](https://sites.google.com/view/rpmsthereo/)|2022|Single|1x OS1-128|No|Odom|Large|
|[ORFD](https://github.com/chaytonmin/Off-Road-Freespace-Detection)|2022|Single|1x Hesai Pandar40P|No|Seg|Large|
|[Tiers](https://github.com/TIERS/tiers-lidars-dataset)|2022|Multi|1x VLP-16C, 1x OS1-64, 1x OS0-128|1x Livox Avia, 1x Livox Horizon, 1x RealSense L515|Odom|Both|
|[Hllti 2022 SLAM dataset](https://hilti-challenge.com/dataset-2022.html)|2022|Single|1x Hesai PandarXT-32|No|Odom|Small|
|[USTC FLICAR](https://ustc-flicar.github.io/)|2023|Multi|1x HDL-32E, 1x VLP-32C, 1x OS0-128|1x Livox Avia|Odom|Small|
|[WOMD Dataset](https://waymo.com/open/data/motion/)|2023|Single|1x Spinning (Unknown)|No|OD|Large|
|[Wild Places](https://csiro-robotics.github.io/Wild-Places/)|2023|Single|1x VLP-16C|No|PR|Large|
|[FusionPortable](https://fusionportable.github.io/dataset/fusionportable/)|2022|Single|1x OS1-128|No|Odom|Small|
|[Hilti 2023 SLAM Dataset](https://hilti-challenge.com/dataset-2023.html)|2023|Single w. Multiple LiDAR|1x PandarXT-32, 1x Robosense BPearl (each)|No|Odom|Small|
|[City Dataset](https://github.com/minwoo0611/MA-LIO)|2023|Multi|1x OS2-128|1x Livox Tele, 1x Livox Avia|Odom|Large
|[Ground-Challenge](https://github.com/sjtuyinjie/Ground-Challenge)|2023|Single|1 $\times$ VLP-16C|No|Odom|Small|
|[RACECAR](https://github.com/linklab-uva/RACECAR_DATA)|2023|Multi|No|3x Luminar Hydra|Loc, OD|Large|
|[ConSLAM](https://github.com/mac137/ConSLAM)|2023|Single|1x VLP-16C|No|SLAM|Small|
|[Pohang Canal Dataset](https://sites.google.com/view/pohang-canal-dataset/home?authuser=0)|2023|Multi|1x OS1-64, 2x OS1-32|No|Odom|Large|
|[Boreas](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjR4MSfo-qBAxXgh1YBHeQIB7AQFnoECAgQAQ&url=https%3A%2F%2Fwww.boreas.utias.utoronto.ca%2F&usg=AOvVaw2zQ4gkfyDfI3aPIxXUL56v&opi=89978449)|2023|Single|1x VLP-128|No|Odom,PR,OD|Large|
|[HeLiPR](https://sites.google.com/view/heliprdataset)|2023|Multi|1x OS1-64, 1x VLP-16|1x Livox Avia, 1x Aeva Aeries II|Odom,PR|Large|
|[A Multi-LiDAR Multi-UAV Dataset](https://tiers.github.io/multi_lidar_multi_uav_dataset/)|2023|Multi|1x OS1-64|1x Livox Mid, 1x Livox 360|Odom|Small|
