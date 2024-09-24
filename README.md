# Awesome 3D LiDAR Datasets [![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

This repository is the collection of datasets, involving the 3D LiDAR. The information is presented in a comprehensive table, outlining the type and number of LiDARs, the purpose of each dataset, and scale details. The objectives are broadly categorized into Object Detection (OD), Segmentation (Seg), Odometry (Odom), Place Recognition (PR), Depth Estimation (Depth) and Localization (Loc). If a dataset includes data exceeding 1 km, it is classified as large scale. Datasets that use multiple LiDAR sequences, even if not executed concurrently, are labeled as 'Single w. Multiple LiDAR'.

## Most Recent Update
### Update: 2024-09-24
- Adds the full description of some datasets (continuing)
### Update: 2023-09-23
- Updates recent datasets (HeLiMOS, GEODE and HK MEMS dataset)
### Update: 2024-04-19
- Updates recent datasets (FusionPortableV2)
### Update: 2024-02-23
- Updates recent 2024 datasets (MARS-LVIG dataset, M2DGR-plus, LiDAR-degeneracy-dataset and BotanicGarden)
### Update: 2023-07-13
- The table includes the specific LiDAR products utilized in each dataset, involving the its channels or name.
### Update: 2023-07-12
- Initial segregation of the 3D LiDAR dataset collection.

## Summary Table
The table below summarizes the details of each dataset:

|Dataset|Year|Single vs Multi|Spinning LiDAR|Solid State LiDAR|Objective|Scale|
|---|---|---|---|---|---|---|
|[Ford Campus](https://robots.engin.umich.edu/SoftwareData/InfoFord)|2011|Single|1x HDL-64E|No|Odom|Large|
|[KITTI](https://www.cvlibs.net/datasets/kitti/)|2013|Single|1x HDL-64E|No|Odom|Large|
|[NCLT](http://robots.engin.umich.edu/nclt/)|2017|Single|1x HDL-32E|No|Odom|Both|
|[Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset)|2019|Multi|2x VLP-16C|No|Odom|Large|
|[Toronto-3D](https://github.com/WeikaiTan/Toronto-3D)|2020|Multi|1x Teledyne Optech Maverick (32 Channels)|No|Seg|Large|
|[Apollo-SouthBay Dataset](https://developer.apollo.auto/southbay.html)|2019|Single|1x HDL-64E|No|Loc|Large|
|[Apollo-DaoxiangLake Dataset](https://developer.apollo.auto/daoxianglake.html)|2020|Single|1x HDL-64E|No|Loc|Large|
|[MulRan](https://sites.google.com/view/mulran-pr/dataset)|2020|Single|1x OS1-64|No|PR, Odom|Large|
|[The Oxford Radar RobotCar Dataset](https://oxford-robotics-institute.github.io/radar-robotcar-dataset/)|2020|Multi|2x HDL-32E|No|Odom, PR|Large|
|[Newer College Dataset](https://ori-drs.github.io/newer-college-dataset/)|2020|Single|1x OS1-64|No|Odom|Small|
|[nuScences](https://www.nuscenes.org/)|2020|Single|1x HDL-32E|No|OD|Large|
|[Ford AV Dataset](https://avdata.ford.com/)|2020|Multi|4x HDL-32E|No|Odom|Large|
|[LIBRE](https://sites.google.com/g.sp.m.is.nagoya-u.ac.jp/libre-dataset)|2020|Single w. Multiple LiDAR|12x Spinning (each)|No|Odom|Large|
|[DurLAR](https://github.com/l1997i/DurLAR)|2021|Single|2x OS1-128|No|Depth|Large|
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
|[FusionPortable](https://fusionportable.github.io/dataset/fusionportable/)|2022|Single|1x OS1-128|No|Odom|Small|
|[Hllti 2022 SLAM dataset](https://hilti-challenge.com/dataset-2022.html)|2022|Single|1x Hesai PandarXT-32|No|Odom|Small|
|[USTC FLICAR](https://ustc-flicar.github.io/)|2023|Multi|1x HDL-32E, 1x VLP-32C, 1x OS0-128|1x Livox Avia|Odom|Small|
|[WOMD Dataset](https://waymo.com/open/data/motion/)|2023|Single|1x Spinning (Unknown)|No|OD|Large|
|[Wild Places](https://csiro-robotics.github.io/Wild-Places/)|2023|Single|1x VLP-16C|No|PR|Large|
|[Hilti 2023 SLAM Dataset](https://hilti-challenge.com/dataset-2023.html)|2023|Single w. Multiple LiDAR|1x PandarXT-32, 1x Robosense BPearl (each)|No|Odom|Small|
|[City Dataset](https://github.com/minwoo0611/MA-LIO)|2023|Multi|1x OS2-128|1x Livox Tele, 1x Livox Avia|Odom|Large
|[Ground-Challenge](https://github.com/sjtuyinjie/Ground-Challenge)|2023|Single|1 $\times$ VLP-16C|No|Odom|Small|
|[RACECAR](https://github.com/linklab-uva/RACECAR_DATA)|2023|Multi|No|3x Luminar Hydra|Loc, OD|Large|
|[ConSLAM](https://github.com/mac137/ConSLAM)|2023|Single|1x VLP-16C|No|SLAM|Small|
|[Pohang Canal Dataset](https://sites.google.com/view/pohang-canal-dataset/home?authuser=0)|2023|Multi|1x OS1-64, 2x OS1-32|No|Odom|Large|
|[Boreas](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjR4MSfo-qBAxXgh1YBHeQIB7AQFnoECAgQAQ&url=https%3A%2F%2Fwww.boreas.utias.utoronto.ca%2F&usg=AOvVaw2zQ4gkfyDfI3aPIxXUL56v&opi=89978449)|2023|Single|1x VLP-128|No|Odom,PR,OD|Large|
|[HeLiPR](https://sites.google.com/view/heliprdataset)|2023|Multi|1x OS1-64, 1x VLP-16|1x Livox Avia, 1x Aeva Aeries II|Odom,PR|Large|
|[A Multi-LiDAR Multi-UAV Dataset](https://tiers.github.io/multi_lidar_multi_uav_dataset/)|2023|Multi|1x OS1-64|1x Livox Mid, 1x Livox 360|Odom|Small|
|[ParisLuco3D](https://npm3d.fr/parisluco3d)|2023|Single|1x HDL-32E|No|Seg, OD|Large|
|[MARS-LVIG dataset](https://journals.sagepub.com/doi/full/10.1177/02783649241227968)|2024|Single|No|1x Livox Avia|Odom, Loc|Large|
|[M2DGR-plus](https://github.com/SJTU-ViSYS/M2DGR-plus?tab=readme-ov-file)|2024|Single|1x RS LiDAR 16C|No|Odom|Small|
|[LiDAR-Degeneray-Datasets](https://github.com/ntnu-arl/lidar_degeneracy_datasets)|2024|Single|1x OS0-128|No|Odom|Small|
|[BotanicGarden](https://github.com/robot-pesg/BotanicGarden)|2024|Multi|1x VLP-16|1x Livox Avia|Odom, Loc, PR|Large|
|[3DRef](https://3dref.github.io/)|2024|Multi|1x OS0-128, 1x Hesai QT64|1x Livox Avia|Seg|Small|
|[FusionPortableV2](https://fusionportable.github.io/dataset/fusionportable_v2/#various-platforms-and-scenarios)|2024|Single|1x OS1-128|No|Odom|Large|
|[HeLiMOS](https://sites.google.com/view/helimos)|2024|Multi|1x OS2-128, 1x VLP-16C|1x Livox Avia, 1x Aeva Aeries II|Seg|Large|
|[GEODE Dataset](https://github.com/PengYu-Team/GEODE_dataset)|2024|Multi|1x VLP-16C, 1x OS1-64|1x Livox Avia|Odom|Large|
|[HK MEMS Dataset](https://github.com/RuanJY/HK_MEMS_Dataset)|2024|Multi|1x OS1-32|1x Robosense M1 LiDAR, 1x Realsense L515|Odom|Large|

## Datasets
### [Ford Campus](https://robots.engin.umich.edu/SoftwareData/InfoFord)
- **Year**: 2011
- **Sensor**: Velodyne HDL-64E, Point Grey Ladybug3 omnidirectional camera, Reigl LMS-Q120 LiDAR, Applanix POS-LV 420 INS with Trimble GPS, Xsens MTi-G
- **Objective**: Odometry
- **Environment**: Campus and Downtown in Dearbon
- **System**: Vehicle 
- **Publication**: IJRR
- **Abstract**: In this paper we describe a data set collected by an autonomous ground vehicle testbed, based upon a modified Ford F-250 pickup truck. The vehicle is outfitted with a professional (Applanix POS-LV) and consumer (Xsens MTi-G) inertial measurement unit, a Velodyne three-dimensional lidar scanner, two push-broom forward-looking Riegl lidars, and a Point Grey Ladybug3 omnidirectional camera system. Here we present the time-registered data from these sensors mounted on the vehicle, collected while driving the vehicle around the Ford Research Campus and downtown Dearborn, MI, during November--December 2009. The vehicle path trajectory in these data sets contains several large- and small-scale loop closures, which should be useful for testing various state-of-the-art computer vision and simultaneous localization and mapping algorithms

### [KITTI](https://www.cvlibs.net/datasets/kitti/)
- **Year**: 2013
- **Sensor**: Velodyne HDL-64E, Point Grey Flea2 video cameras, OXTS RT 3003 localization system (GPS/IMU/RTK)
- **Objective**: Odometry
- **Environment**: Downtown
- **System**: Vehicle (Ford F-250)
- **Publication**: CVPR
- **Abstract**: Today, visual recognition systems are still rarely employed in robotics applications. Perhaps one of the main reasons for this is the lack of demanding benchmarks that mimic such scenarios. In this paper, we take advantage of our autonomous driving platform to develop novel challenging benchmarks for the tasks of stereo, optical flow, visual odometry/SLAM and 3D object detection. Our recording platform is equipped with four high resolution video cameras, a Velodyne laser scanner and a state-of-the-art localization system. Our benchmarks comprise 389 stereo and optical flow image pairs, stereo visual odometry sequences of 39.2 km length, and more than 200k 3D object annotations captured in cluttered scenarios (up to 15 cars and 30 pedestrians are visible per image). Results from state-of-the-art algorithms reveal that methods ranking high on established datasets such as Middlebury perform below average when being moved outside the laboratory to the real world. Our goal is to reduce this bias by providing challenging benchmarks with novel difficulties to the computer vision community. Our benchmarks are available online at: www.cvlibs.net/datasets/kitti



### [NCLT](http://robots.engin.umich.edu/nclt/)
- **Year**: 2017
- **Sensor**: Velodyne HDL-32E, Hokuyo UTM-30LX-EW, Ladybug3 omnidirectional camera, IMU, FOG, GPS, RTK GPS
- **Objective**: Odometry
- **Environment**: Campus
- **System**: Segway
- **Publication**: IJRR
- **Abstract**: This paper documents a large scale, long-term autonomy dataset for robotics research collected on the University of Michigan’s North Campus. The dataset consists of omnidirectional imagery, 3D lidar, planar lidar, GPS, and proprioceptive sensors for odometry collected using a Segway robot. The dataset was collected to facilitate research focusing on long-term autonomous operation in changing environments. The dataset is comprised of 27 sessions spaced approximately biweekly over the course of 15 months. The sessions repeatedly explore the campus, both indoors and outdoors, on varying trajectories, and at different times of the day across all four seasons. This allows the dataset to capture many challenging elements including: moving obstacles (e.g., pedestrians, bicyclists, and cars), changing lighting, varying viewpoint, seasonal and weather changes (e.g., falling leaves and snow), and long-term structural changes caused by construction projects. To further facilitate research, we also provide ground-truth pose for all sessions in a single frame of reference. 

### [Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset)
- **Year**: 2019
- **Sensor**: 2x Velodyne VLP-16C, FLIR FL3-U3-20E4-C, U-Blox EVK-7P GPS, SOKKIA GRX2 GPS, KVH DSP-1760 FOG, Xsens MTi-300 IMU, RLS LM13 Encoder, Withrobot myPressure Altimeter
- **Objective**: Odometry
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: IJRR
- **Abstract**: The high diversity of urban environments, at both the inter and intra levels, poses challenges for robotics research. Such challenges include discrepancies in urban features between cities and the deterioration of sensor measurements within a city. With such diversity in consideration, this paper aims to provide Light Detection and Ranging (LiDAR) and image data acquired in complex urban environments. In contrast to existing datasets, the presented dataset encapsulates various complex urban features and addresses the major issues of complex urban areas, such as unreliable and sporadic Global Positioning System (GPS) data, multi-lane roads, complex building structures, and the abundance of highly dynamic objects. This paper provides two types of LiDAR sensor data (2D and 3D) as well as navigation sensor data with commercial-level accuracy and high-level accuracy. In addition, two levels of sensor data are provided for the purpose of assisting in the complete validation of algorithms using consumer-grade sensors. A forward-facing stereo camera was utilized to capture visual images of the environment and the position information of the vehicle that was estimated through simultaneous localization mapping (SLAM) are offered as a baseline. This paper presents 3D map data generated by the SLAM algorithm in the LASer (LAS) format for a wide array of research purposes, and a file player and a data viewer have been made available via the Github webpage to allow researchers to conveniently utilize the data in a Robot Operating System (ROS) environment. The provided file player is capable of sequentially publishing large quantities of data, similar to the rosbag player. The dataset in its entirety can be found at http://irap.kaist.ac.kr/dataset.

### [Toronto-3D](https://github.com/WeikaiTan/Toronto-3D)
- **Year**: 2020
- **Sensor**: Teledyne Optech Maverick (32 Channels), Ladybug5 panoramic camera, GNSS
- **Objective**: Segmentation
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: CVPR Workshop
- **Abstract**: Semantic segmentation of large-scale outdoor point clouds is essential for urban scene understanding in various applications, especially autonomous driving and urban high-definition (HD) mapping. With rapid developments of mobile laser scanning (MLS) systems, massive point clouds are available for scene understanding, but publicly accessible large-scale labeled datasets, which are essential for developing learning-based methods, are still limited. This paper introduces Toronto-3D, a large-scale urban outdoor point cloud dataset acquired by a MLS system in Toronto, Canada for semantic segmentation. This dataset covers approximately 1 km of point clouds and consists of about 78.3 million points with 8 labeled object classes. Baseline experiments for semantic segmentation were conducted and the results confirmed the capability of this dataset to train deep learning models effectively. Toronto-3D is released to encourage new research, and the labels will be improved and updated with feedback from the research community.

### [Apollo-DaoxiangLake Dataset](https://developer.apollo.auto/daoxianglake.html)

- **Year**: 2020
- **Sensor**: Velodyne HDL-64E, Camera, IMU
- **Objective**: Localization
- **Environment**: Lake Park
- **System**: Vehicle
- **Publication**: ECCV
- **Abstract**: The Apollo-DaoxiangLake dataset contains 8 trials of repetitive data within 14 weeks over the same road. In particular, the dataset includes different times of the day, for example, noon, afternoon, sunset, and seasonal changes, e.g., sunny, snowy days. The dataset contains 3D LiDAR point clouds (with motion compensation), images, IMU datas and post-processing ground truth poses in a local coordinate system. The camera images in the datasets are shown to demonstrate the coverage diversity of our datasets.

### [Apollo-SouthBay Dataset](https://developer.apollo.auto/southbay.html)
- **Year**: 2019
- **Sensor**: Velodyne HDL-64E, IMU
- **Objective**: Localization
- **Environment**: Park, Highway, Downtown
- **System**: Vehicle
- **Publication**: CVPR
- **Abstract**: The Apollo-SouthBay dataset contains six routes, BaylandsToSeafood, ColumbiaPark, Highway237, MathildaAVE, SanJoseDowntown, and SunnyvaleBigLoop covering different scenarios including but not limited to residential areas, urban downtown areas and highways. What's more, the dataset contains 3D LiDAR scans(as well as point clouds generated by accumulating scans with motion compensation), post-processed ground truth poses and online estimated poses from the GNSS/IMU integrated solution. In Figure 1, the six routes in southern SanFrancisco Bay area are shown for reference. In Figure 2, the camera images of four selected scenarios in the datasets are shown to demonstrate the coverage diversity of our datasets.

### [MulRan](https://sites.google.com/view/mulran-pr/dataset)
- **Year**: 2020
- **Sensor**: Ouster OS1-64, Navtech CIR204-H, IMU, FOG an GPS (Same as Complex Urban Dataset)
- **Objective**: Place Recognition, Odometry
- **Environment**: Campus, Urban
- **System**: Vehicle
- **Publication**: ICRA
- **Abstract**: This paper introduces a multimodal range dataset namely for radio detection and ranging (radar) and light detection and ranging (LiDAR) specifically targeting the urban environment. By extending our workshop paper [1] to a larger scale, this dataset focuses on the range sensor-based place recognition and provides 6D baseline trajectories of a vehicle for place recognition ground truth. Provided radar data support both raw-level and image-format data, including a set of time-stamped 1D intensity arrays and 360◦ polar images, respectively. In doing so, we provide flexibility between raw data and image data depending on the purpose of the research. Unlike existing datasets, our focus is at capturing both temporal and structural diversities for range-based place recognition research. For evaluation, we applied and validated that our previous location descriptor and its search algorithm [2] are highly effective for radar place recognition method. Furthermore, the result shows that radar-based place recognition outperforms LiDAR-based one exploiting its longer-range measurements. The dataset is available from https://sites.google.com/view/mulran-pr

### [The Oxford Radar RobotCar Dataset](https://oxford-robotics-institute.github.io/radar-robotcar-dataset/)

- **Year**: 2020
- **Sensor**: Navtech CTS350-X Millimetre-Wave FMCW radar, 2 x Velodyne HDL-32E 3D LIDAR, Point Grey Bumblebee XB3 (BBX3-13S2C38) trinocular stereo camera, 3 x Point Grey Grasshopper2 (GS2-FW-14S5C-C) monocular camera, 2xSICKLMS-151 2D LIDAR, NovAtel SPAN-CPT ALIGN inertial and GPS navigation system
- **Objective**: Odometry, Place Recognition
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: ICRA
- **Abstract**: In this paper we present The Oxford Radar RobotCar Dataset, a new dataset for researching scene understanding using Millimetre-Wave FMCW scanning radar data. The target application is autonomous vehicles where this modality is robust to environmental conditions such as fog, rain, snow, or lens flare, which typically challenge other sensor modalities such as vision and LIDAR. The data were gathered in January 2019 over thirty-two traversals of a central Oxford route spanning a total of 280km of urban driving. It encompasses a variety of weather, traffic, and lighting conditions. This 4.7TB dataset consists of over 240,000 scans from a Navtech CTS350-X radar and 2.4 million scans from two Velodyne HDL-32E 3D LIDARs; along with six cameras, two 2D LIDARs, and a GPS/INS receiver. In addition we release ground truth optimised radar odometry to provide an additional impetus to research in this domain. The full dataset is available for download at: ori.ox.ac.uk/datasets/radar-robotcar-dataset

### [Newer College Dataset](https://ori-drs.github.io/newer-college-dataset/)
- **Year**: 2020
- **Sensor**: Ouster OS1-64, Intel Realsense-D435i, Bosch BMI055 (Camera IMU), ICM-20948 (LiDAR IMU)
- **Objective**: Odometry
- **Environment**: College
- **System**: Handheld
- **Publication**: IROS
- **Abstract**: In this paper, we present a large dataset with a variety of mobile mapping sensors collected using a handheld device carried at typical walking speeds for nearly 2.2 km around New College, Oxford as well as a series of supplementary datasets with much more aggressive motion and lighting contrast. The datasets include data from two commercially available devices- a stereoscopic-inertial camera and a multibeam 3D LiDAR, which also provides inertial measurements. Additionally, we used a tripod-mounted survey grade LiDAR scanner to capture a detailed millimeter-accurate 3D map of the test location (containing ∼290 million points). Using the map, we generated a 6 Degrees of Freedom (DoF) ground truth pose for each LiDAR scan (with approximately 3 cm accuracy) to enable better benchmarking of LiDAR and vision localisation, mapping and reconstruction systems. This ground truth is the particular novel contribution of this dataset and we believe that it will enable systematic evaluation which many similar datasets have lacked. The large dataset combines both built environments, open spaces and vegetated areas so as to test localisation and mapping systems such as vision-based navigation, visual and LiDAR SLAM, 3D LiDAR reconstruction and appearance-based place recognition, while the supplementary datasets contain very dynamic motions to introduce more challenges for visual-inertial odometry systems. The datasets are available at: ori.ox.ac.uk/datasets/newer-college-dataset

### [nuScences](https://www.nuscenes.org/)
- **Year**: 2020
- **Sensor**: 1x HDL-32E, 6x Camera RGB, 5x Radar, GPS, IMU
- **Objective**: Object Detection
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: CVPR
- **Abstract**: Robust detection and tracking of objects is crucial for the deployment of autonomous vehicle technology. Image based benchmark datasets have driven development in computer vision tasks such as object detection, tracking and segmentation of agents in the environment. Most autonomous vehicles, however, carry a combination of cameras and range sensors such as lidar and radar. As machine learning based methods for detection and tracking become more prevalent, there is a need to train and evaluate such methods on datasets containing range sensor data along with images. In this work we present nuTonomy scenes (nuScenes), the first dataset to carry the full autonomous vehicle sensor suite: 6 cameras, 5 radars and 1 lidar, all with full 360 degree field of view. nuScenes comprises 1000 scenes, each 20s long and fully annotated with 3D bounding boxes for 23 classes and 8 attributes. It has 7x as many annotations and 100x as many images as the pioneering KITTI dataset. We define novel 3D detection and tracking metrics. We also provide careful dataset analysis as well as baselines for lidar and image based detection and tracking. Data, development kit and more information are available online1.

### [Ford AV Dataset](https://avdata.ford.com/)
- **Year**: 2020
- **Sensor**: 4x HDL-32E, 6x Point Grey 1.3 MP Cameras, 1x Point Grey 5 MP Camera, Applanix POS-LV GNSS system
- **Objective**: Odometry
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: IJRR
- **Abstract**: This paper presents a challenging multi-agent seasonal dataset collected by a fleet of Ford autonomous vehicles at different days and times during 2017-18. The vehicles traversed an average route of 66 km in Michigan that included a mix of driving scenarios such as the Detroit Airport, freeways, city-centers, university campus and suburban neighbourhoods, etc. Each vehicle used in this data collection is a Ford Fusion outfitted with an Applanix POS-LV GNSS system, four HDL-32E Velodyne 3D-lidar scanners, 6 Point Grey 1.3 MP Cameras arranged on the rooftop for 360-degree coverage and 1 Pointgrey 5 MP camera mounted behind the windshield for the forward field of view. We present the seasonal variation in weather, lighting, construction and traffic conditions experienced in dynamic urban environments. This dataset can help design robust algorithms for autonomous vehicles and multi-agent systems. Each log in the dataset is time-stamped and contains raw data from all the sensors, calibration values, pose trajectory, ground truth pose, and 3D maps. All data is available in Rosbag format that can be visualized, modified and applied using the open-source Robot Operating System (ROS). We also provide the output of state-of-the-art reflectivity-based localization for bench-marking purposes. The dataset can be freely downloaded at avdata.ford.com.

### [LIBRE](https://sites.google.com/g.sp.m.is.nagoya-u.ac.jp/libre-dataset)
- **Year**: 2020
- **Sensor**: 12x Spinning LiDAR (each), Camera, IMU, GNSS, CAN, 360° 4K camera, Event camera, Infrared camera, 3D point cloud map, Vector map
- **Objective**: Odometry
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: IV
- **Abstract**: In this work, we present LIBRE: LiDAR Benchmarking and Reference, a first-of-its-kind dataset featuring 10 different LiDAR sensors, covering a range of manufacturers, models, and laser configurations. Data captured independently from each sensor includes three different environments and configurations: static targets, where objects were placed at known distances and measured from a fixed position within a controlled environment; adverse weather, where static obstacles were measured from a moving vehicle, captured in a weather chamber where LiDARs were exposed to different conditions (fog, rain, strong light); and finally, dynamic traffic, where dynamic objects were captured from a vehicle driven on public urban roads, multiple times at different times of the day, and including supporting sensors such as cameras, infrared imaging, and odometry devices. LIBRE will contribute to the research community to (1) provide a means for a fair comparison of currently available LiDARs, and (2) facilitate the improvement of existing self-driving vehicles and robotics-related software, in terms of development and tuning of LiDAR-based perception algorithms.

### [DurLAR](https://github.com/l1997i/DurLAR)
- **Year**: 2021
- **Sensor**: 1x Ouster OS1-128, 1x Carnegie Robotics MultiSense S21 stereo camera, 1x OxTS RT3000v3 GNSS/INS, 1x Yocto Light V3 lux meter
- **Objective**: Depth Estimation
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: 3DV
- **Abstract**: We present DurLAR, a high-fidelity 128-channel 3D LiDAR dataset with panoramic ambient (near infrared) and reflectivity imagery, as well as a sample benchmark task using depth estimation for autonomous driving applications. Our driving platform is equipped with a high resolution 128 channel LiDAR, a 2MPix stereo camera, a lux meter and a GNSS/INS system. Ambient and reflectivity images are made available along with the LiDAR point clouds to facilitate multi-modal use of concurrent ambient and reflectivity scene information. Leveraging DurLAR, with a resolution exceeding that of prior benchmarks, we consider the task of monocular depth estimation and use this increased availability of higher resolution, yet sparse ground truth scene depth information to propose a novel joint supervised/selfsupervised loss formulation. We compare performance over both our new DurLAR dataset, the established KITTI benchmark and the Cityscapes dataset. Our evaluation shows our joint use supervised and self-supervised loss terms, enabled via the superior ground truth resolution and availability within DurLAR improves the quantitative and qualitative performance of leading contemporary monocular depth estimation approaches (RMSE = 3.639, SqRel = 0.936).

### [EU Long-term Dataset](https://epan-utbm.github.io/utbm_robocar_dataset/)
- **Year**: 2021
- **Sensor**: 2x HDL-32E, 2x Pixelink PL-B742F, 1x ibeo LUX 4L, 1x Continental ARS 308, 1x SICK LMS100-10000, 1x Magellan ProFlex 500, 1x Xsens MTi-28A53G25
- **Objective**: Odometry
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: IROS
- **Abstract**: The field of autonomous driving has grown tremendously over the past few years, along with the rapid progress in sensor technology. One of the major purposes of using sensors is to provide environment perception for vehicle understanding, learning and reasoning, and ultimately interacting with the environment. In this paper, we first introduce a multisensor platform allowing vehicle to perceive its surroundings and locate itself in a more efficient and accurate way. The platform integrates eleven heterogeneous sensors including various cameras and lidars, a radar, an IMU (Inertial Measurement Unit), and a GPS-RTK (Global Positioning System / Real-Time Kinematic), while exploits a ROS (Robot Operating System) based software to process the sensory data. Then, we present a new dataset (https://epan-utbm.github.io/utbm_robocar_dataset/) for autonomous driving captured many new research challenges (e.g. highly dynamic environment), and especially for long-term autonomy (e.g. creating and maintaining maps), collected with our instrumented vehicle, publicly available to the community.

### [NTU VIRAL Dataset](https://ntu-aris.github.io/ntu_viral_dataset/)

- **Year**: 2021
- **Sensor**: 2x OS1-16, 1x Xsens MTi-28A53G25, 1x Humatic P440, 1x Leica MS60 TotalStation
- **Objective**: Odometry
- **Environment**: Urban
- **System**: drone
- **Publication**: IJRR
- **Abstract**: In recent years, autonomous robots have become ubiquitous in research and daily life. Among many factors, public datasets play an important role in the progress of this field, as they waive the tall order of initial investment in hardware and manpower. However, for research on autonomous aerial systems, there appears to be a relative lack of public datasets on par with those used for autonomous driving and ground robots. Thus, to fill in this gap, we conduct a data collection exercise on an aerial platform equipped with an extensive and unique set of sensors: two 3D lidars, two hardware-synchronized global-shutter cameras, multiple Inertial Measurement Units (IMUs), and especially, multiple Ultra-wideband (UWB) ranging units. The comprehensive sensor suite resembles that of an autonomous driving car, but features distinct and challenging characteristics of aerial operations. We record multiple datasets in several challenging indoor and outdoor conditions. Calibration results and ground truth from a high-accuracy laser tracker are also included in each package. All resources can be accessed via our webpage https://ntu-aris.github.io/ntu_viral_dataset/.

### [M2DGR](https://github.com/SJTU-ViSYS/M2DGR)
- **Year**: 2021
- **Sensor**: 1x VLP-32C, 1x Handsfree A9, 1x Ublox M8T, 1x FLIR, 1x Pointgrey CM3-U3-13Y3C, 1x Gaode PLUG 617, 1x Inivation DVXplorer, 1x Realsense d435i, 1x Vicon Vero 2.2, 1x Leica Nova MS60, 1x Xsens Mti 680G
- **Objective**: Odometry
- **Environment**: Urban
- **System**: UGV
- **Publication**: RA-L
- **Abstract**: We introduce M2DGR: a novel large-scale dataset collected by a ground robot with a full sensor-suite including six f ish-eye and one sky-pointing RGB cameras, an infrared camera, an event camera, a Visual-Inertial Sensor (VI-sensor), an inertial measurement unit (IMU), a LiDAR, a consumer-grade Global Navigation Satellite System (GNSS) receiver and a GNSS-IMU navigation system with real-time kinematic (RTK) signals. All those sensors were well-calibrated and synchronized, and their data were recorded simultaneously. The ground truth trajectories were obtained by the motion capture device, a laser 3D tracker, and an RTK receiver. The dataset comprises 36 sequences (about 1TB) captured in diverse scenarios including both indoor and outdoor environments. We evaluate state-of-the-art SLAM algorithms on M2DGR. Results show that existing solutions perform poorly in some scenarios. For the benefit of the research community, we make the dataset and tools public. The webpage of our project is https://github.com/SJTU-ViSYS/M2DGR.

### [Pandaset](https://pandaset.org/)
- **Year**: 2021
- **Sensor**: 1x Hesai Pandar64, 1x Hesai PandarGT, 5x Leopard LI-USB30-AR023ZWDRB, 1x Leopard LI-USB30-AR023ZWDRB, 1x NovAtel PwrPak7
- **Objective**: Segmentation
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: ITSC
- **Abstract**: The accelerating development of autonomous driving technology has placed greater demands on obtaining large amounts of high-quality data. Representative, labeled, real world data serves as the fuel for training deep learning networks, critical for improving self-driving perception algorithms. In this paper, we introduce PandaSet, the first dataset produced by a complete, high-precision autonomous vehicle sensor kit with a no-cost commercial license. The dataset was collected using one 360◦ mechanical spinning LiDAR, one forwardfacing, long-range LiDAR, and 6 cameras. The dataset contains more than 100 scenes, each of which is 8 seconds long, and provides 28 types of labels for object classification and 37 types of labels for semantic segmentation. We provide baselines for LiDAR-only 3D object detection, LiDAR-camera fusion 3D object detection and LiDAR point cloud segmentation. For more details about PandaSet and the development kit, see https://scale.com/open-datasets/pandaset.

### [UrbanNav Dataset]((https://github.com/IPNL-POLYU/UrbanNavDataset))
- **Year**: 2021
- **Sensor**: 1x HDL-32E, 1x VLP-16, 1x Xsens Mti 10, 3x u-blox ZED-F9P, 1x EVK-M8T, 1x NovAtel Flexpak6, 1x ZED2 Stereo, 1x SPAN-CPT
- **Objective**: Odometry
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: ION
- **Abstract**: Urban canyon is typical in megacities like Hong Kong and Tokyo. Accurate positioning in urban canyons remains a challenging problem for the applications with navigation requirements, such as the navigation for pedestrian, autonomous driving vehicles and unmanned aerial vehicles. The GNSS positioning can be significantly degraded in urban canyons, due to the signal blockage by tall buildings. The visual positioning and LiDAR positioning can be considerably affected by numerous dynamic objects. To facilitate the research and development of robust, accurate and precise positioning using multiple sensors in urban canyons, we build a multi-sensoroy dataset collected in diverse challenging urban scenarios in Hong Kong and Tokyo, that provides full-suite sensor data, which includes GNSS, INS, LiDAR and cameras. We call this open-sourced dataset, UrbanNav. In 2019, we formed a joint working group under the joint efforts from International Association of Geodesy (IAG) and ION. This working group is currently under Sub-Commission 4.1: Emerging Positioning Technologies and GNSS Augmentations of IAG. After consolidating the suggestions and comments from intenational navigation researchers, the objectives of this work group are: 1. Open-sourcing positioning sensor data, including GNSS, INS, LiDAR and cameras collected in Asian urban canyons; 2. Raising the awareness of the urgent navigation requirement in highly-urbanized areas, especially in Asian-Pacific regions; 3. Providing an integrated online platform for data sharing to facilitate the development of navigation solutions of the research community; and 4. Benchmarking positioning algorithms based on the open-sourcing data. Currently, two pilot dataset can be downloaded by the link in the following. https://www.polyu-ipn-lab.com/download We also provide a GitHub page to answer possible issues that users may encounter. Meanwhile, we also provide example usage of the dataset for applications of LiDAR simultaneous localization and mapping (SLAM) and visual-inertial navigation system (VINS), etc. https://github.com/weisongwen/UrbanNavDataset In this conference paper, we will introduce the detail sensors setup, data format, the calibration of the intrinsic/extrinsic parameters, and the ground truth generation. We believe this opensource dataset can facilitate to identify the challenges of different sensors in urban canyons. Finally we will address the future maintenance directions of the UrbanNav dataset including the following: 1. Building a website to let the researchers upload their paper and result that evaluated based on the open-source data in terms of the proposed criteria. 2. Identifying the experts in the field to design the assessment criteria for different positioning algorithms. 3. Reporting the performance of the state-of-the-art positioning and integration algorithms in the urban canyons every 2 years.
