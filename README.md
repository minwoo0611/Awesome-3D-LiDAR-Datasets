# Awesome 3D LiDAR Datasets [![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

This repository is the collection of datasets, involving the 3D LiDAR. The information is presented in a comprehensive table, outlining the type and number of LiDARs, the purpose of each dataset, and scale details. The objectives are broadly categorized into Object Detection (OD), Segmentation (Seg), Odometry (Odom), Place Recognition (PR), Depth Estimation (Depth) and Localization (Loc). If a dataset includes data exceeding 1 km, it is classified as large scale. Datasets that use multiple LiDAR sequences, even if not executed concurrently, are labeled as 'Single w. Multiple LiDAR'.

## Most Recent Update
### Update: 2024-11-12
- Adds the full description of some datasets (continuing)
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
|[nuScenes](https://www.nuscenes.org/)|2020|Single|1x HDL-32E|No|OD|Large|
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
|[ORFD](https://github.com/chaytonmin/Off-Road-Freespace-Detection)|2022|Single|1x Hesai Pandora40P|No|Seg|Large|
|[Tiers](https://github.com/TIERS/tiers-lidars-dataset)|2022|Multi|1x VLP-16C, 1x OS1-64, 1x OS0-128|1x Livox Avia, 1x Livox Horizon, 1x RealSense L515|Odom|Both|
|[FusionPortable](https://fusionportable.github.io/dataset/fusionportable/)|2022|Single|1x OS1-128|No|Odom|Small|
|[Hllti 2022 SLAM dataset](https://hilti-challenge.com/dataset-2022.html)|2022|Single|1x Hesai PandarXT-32|No|Odom|Small|
|[USTC FLICAR](https://ustc-flicar.github.io/)|2023|Multi|1x HDL-32E, 1x VLP-32C, 1x OS0-128|1x Livox Avia|Odom|Small|
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
|[ENWIDE](https://projects.asl.ethz.ch/datasets/doku.php?id=enwide)|2024|Single|1x OS0-128|No|Odom|Small|
|[LiDAR-Degeneray-Datasets](https://github.com/ntnu-arl/lidar_degeneracy_datasets)|2024|Single|1x OS0-128|No|Odom|Small|
|[BotanicGarden](https://github.com/robot-pesg/BotanicGarden)|2024|Multi|1x VLP-16|1x Livox Avia|Odom, Loc, PR|Large|
|[WOMD Dataset](https://waymo.com/open/data/motion/)|2024|Multi|1x mid range, 4x short range|No|OD|Large|
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

---

### [KITTI](https://www.cvlibs.net/datasets/kitti/)
- **Year**: 2013
- **Sensor**: Velodyne HDL-64E, Point Grey Flea2 video cameras, OXTS RT 3003 localization system (GPS/IMU/RTK)
- **Objective**: Odometry
- **Environment**: Downtown
- **System**: Vehicle (Ford F-250)
- **Publication**: CVPR
- **Abstract**: Today, visual recognition systems are still rarely employed in robotics applications. Perhaps one of the main reasons for this is the lack of demanding benchmarks that mimic such scenarios. In this paper, we take advantage of our autonomous driving platform to develop novel challenging benchmarks for the tasks of stereo, optical flow, visual odometry/SLAM and 3D object detection. Our recording platform is equipped with four high resolution video cameras, a Velodyne laser scanner and a state-of-the-art localization system. Our benchmarks comprise 389 stereo and optical flow image pairs, stereo visual odometry sequences of 39.2 km length, and more than 200k 3D object annotations captured in cluttered scenarios (up to 15 cars and 30 pedestrians are visible per image). Results from state-of-the-art algorithms reveal that methods ranking high on established datasets such as Middlebury perform below average when being moved outside the laboratory to the real world. Our goal is to reduce this bias by providing challenging benchmarks with novel difficulties to the computer vision community. Our benchmarks are available online at: www.cvlibs.net/datasets/kitti



---

### [NCLT](http://robots.engin.umich.edu/nclt/)
- **Year**: 2017
- **Sensor**: Velodyne HDL-32E, Hokuyo UTM-30LX-EW, Ladybug3 omnidirectional camera, IMU, FOG, GPS, RTK GPS
- **Objective**: Odometry
- **Environment**: Campus
- **System**: Segway
- **Publication**: IJRR
- **Abstract**: This paper documents a large scale, long-term autonomy dataset for robotics research collected on the University of Michigan’s North Campus. The dataset consists of omnidirectional imagery, 3D lidar, planar lidar, GPS, and proprioceptive sensors for odometry collected using a Segway robot. The dataset was collected to facilitate research focusing on long-term autonomous operation in changing environments. The dataset is comprised of 27 sessions spaced approximately biweekly over the course of 15 months. The sessions repeatedly explore the campus, both indoors and outdoors, on varying trajectories, and at different times of the day across all four seasons. This allows the dataset to capture many challenging elements including: moving obstacles (e.g., pedestrians, bicyclists, and cars), changing lighting, varying viewpoint, seasonal and weather changes (e.g., falling leaves and snow), and long-term structural changes caused by construction projects. To further facilitate research, we also provide ground-truth pose for all sessions in a single frame of reference. 

---

### [Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset)
- **Year**: 2019
- **Sensor**: 2x Velodyne VLP-16C, FLIR FL3-U3-20E4-C, U-Blox EVK-7P GPS, SOKKIA GRX2 GPS, KVH DSP-1760 FOG, Xsens MTi-300 IMU, RLS LM13 Encoder, Withrobot myPressure Altimeter
- **Objective**: Odometry
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: IJRR
- **Abstract**: The high diversity of urban environments, at both the inter and intra levels, poses challenges for robotics research. Such challenges include discrepancies in urban features between cities and the deterioration of sensor measurements within a city. With such diversity in consideration, this paper aims to provide Light Detection and Ranging (LiDAR) and image data acquired in complex urban environments. In contrast to existing datasets, the presented dataset encapsulates various complex urban features and addresses the major issues of complex urban areas, such as unreliable and sporadic Global Positioning System (GPS) data, multi-lane roads, complex building structures, and the abundance of highly dynamic objects. This paper provides two types of LiDAR sensor data (2D and 3D) as well as navigation sensor data with commercial-level accuracy and high-level accuracy. In addition, two levels of sensor data are provided for the purpose of assisting in the complete validation of algorithms using consumer-grade sensors. A forward-facing stereo camera was utilized to capture visual images of the environment and the position information of the vehicle that was estimated through simultaneous localization mapping (SLAM) are offered as a baseline. This paper presents 3D map data generated by the SLAM algorithm in the LASer (LAS) format for a wide array of research purposes, and a file player and a data viewer have been made available via the Github webpage to allow researchers to conveniently utilize the data in a Robot Operating System (ROS) environment. The provided file player is capable of sequentially publishing large quantities of data, similar to the rosbag player. The dataset in its entirety can be found at http://irap.kaist.ac.kr/dataset.

---

### [Toronto-3D](https://github.com/WeikaiTan/Toronto-3D)
- **Year**: 2020
- **Sensor**: Teledyne Optech Maverick (32 Channels), Ladybug5 panoramic camera, GNSS
- **Objective**: Segmentation
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: CVPR Workshop
- **Abstract**: Semantic segmentation of large-scale outdoor point clouds is essential for urban scene understanding in various applications, especially autonomous driving and urban high-definition (HD) mapping. With rapid developments of mobile laser scanning (MLS) systems, massive point clouds are available for scene understanding, but publicly accessible large-scale labeled datasets, which are essential for developing learning-based methods, are still limited. This paper introduces Toronto-3D, a large-scale urban outdoor point cloud dataset acquired by a MLS system in Toronto, Canada for semantic segmentation. This dataset covers approximately 1 km of point clouds and consists of about 78.3 million points with 8 labeled object classes. Baseline experiments for semantic segmentation were conducted and the results confirmed the capability of this dataset to train deep learning models effectively. Toronto-3D is released to encourage new research, and the labels will be improved and updated with feedback from the research community.

---

### [Apollo-DaoxiangLake Dataset](https://developer.apollo.auto/daoxianglake.html)

- **Year**: 2020
- **Sensor**: Velodyne HDL-64E, Camera, IMU
- **Objective**: Localization
- **Environment**: Lake Park
- **System**: Vehicle
- **Publication**: ECCV
- **Abstract**: The Apollo-DaoxiangLake dataset contains 8 trials of repetitive data within 14 weeks over the same road. In particular, the dataset includes different times of the day, for example, noon, afternoon, sunset, and seasonal changes, e.g., sunny, snowy days. The dataset contains 3D LiDAR point clouds (with motion compensation), images, IMU datas and post-processing ground truth poses in a local coordinate system. The camera images in the datasets are shown to demonstrate the coverage diversity of our datasets.

---

### [Apollo-SouthBay Dataset](https://developer.apollo.auto/southbay.html)
- **Year**: 2019
- **Sensor**: Velodyne HDL-64E, IMU
- **Objective**: Localization
- **Environment**: Park, Highway, Downtown
- **System**: Vehicle
- **Publication**: CVPR
- **Abstract**: The Apollo-SouthBay dataset contains six routes, BaylandsToSeafood, ColumbiaPark, Highway237, MathildaAVE, SanJoseDowntown, and SunnyvaleBigLoop covering different scenarios including but not limited to residential areas, urban downtown areas and highways. What's more, the dataset contains 3D LiDAR scans(as well as point clouds generated by accumulating scans with motion compensation), post-processed ground truth poses and online estimated poses from the GNSS/IMU integrated solution. In Figure 1, the six routes in southern SanFrancisco Bay area are shown for reference. In Figure 2, the camera images of four selected scenarios in the datasets are shown to demonstrate the coverage diversity of our datasets.

---

### [MulRan](https://sites.google.com/view/mulran-pr/dataset)
- **Year**: 2020
- **Sensor**: Ouster OS1-64, Navtech CIR204-H, IMU, FOG an GPS (Same as Complex Urban Dataset)
- **Objective**: Place Recognition, Odometry
- **Environment**: Campus, Urban
- **System**: Vehicle
- **Publication**: ICRA
- **Abstract**: This paper introduces a multimodal range dataset namely for radio detection and ranging (radar) and light detection and ranging (LiDAR) specifically targeting the urban environment. By extending our workshop paper [1] to a larger scale, this dataset focuses on the range sensor-based place recognition and provides 6D baseline trajectories of a vehicle for place recognition ground truth. Provided radar data support both raw-level and image-format data, including a set of time-stamped 1D intensity arrays and 360◦ polar images, respectively. In doing so, we provide flexibility between raw data and image data depending on the purpose of the research. Unlike existing datasets, our focus is at capturing both temporal and structural diversities for range-based place recognition research. For evaluation, we applied and validated that our previous location descriptor and its search algorithm [2] are highly effective for radar place recognition method. Furthermore, the result shows that radar-based place recognition outperforms LiDAR-based one exploiting its longer-range measurements. The dataset is available from https://sites.google.com/view/mulran-pr

---

### [The Oxford Radar RobotCar Dataset](https://oxford-robotics-institute.github.io/radar-robotcar-dataset/)

- **Year**: 2020
- **Sensor**: Navtech CTS350-X Millimetre-Wave FMCW radar, 2 x Velodyne HDL-32E 3D LIDAR, Point Grey Bumblebee XB3 (BBX3-13S2C38) trinocular stereo camera, 3 x Point Grey Grasshopper2 (GS2-FW-14S5C-C) monocular camera, 2xSICKLMS-151 2D LIDAR, NovAtel SPAN-CPT ALIGN inertial and GPS navigation system
- **Objective**: Odometry, Place Recognition
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: ICRA
- **Abstract**: In this paper we present The Oxford Radar RobotCar Dataset, a new dataset for researching scene understanding using Millimetre-Wave FMCW scanning radar data. The target application is autonomous vehicles where this modality is robust to environmental conditions such as fog, rain, snow, or lens flare, which typically challenge other sensor modalities such as vision and LIDAR. The data were gathered in January 2019 over thirty-two traversals of a central Oxford route spanning a total of 280km of urban driving. It encompasses a variety of weather, traffic, and lighting conditions. This 4.7TB dataset consists of over 240,000 scans from a Navtech CTS350-X radar and 2.4 million scans from two Velodyne HDL-32E 3D LIDARs; along with six cameras, two 2D LIDARs, and a GPS/INS receiver. In addition we release ground truth optimised radar odometry to provide an additional impetus to research in this domain. The full dataset is available for download at: ori.ox.ac.uk/datasets/radar-robotcar-dataset

---

### [Newer College Dataset](https://ori-drs.github.io/newer-college-dataset/)
- **Year**: 2020
- **Sensor**: Ouster OS1-64, Intel Realsense-D435i, Bosch BMI055 (Camera IMU), ICM-20948 (LiDAR IMU)
- **Objective**: Odometry
- **Environment**: College
- **System**: Handheld
- **Publication**: IROS
- **Abstract**: In this paper, we present a large dataset with a variety of mobile mapping sensors collected using a handheld device carried at typical walking speeds for nearly 2.2 km around New College, Oxford as well as a series of supplementary datasets with much more aggressive motion and lighting contrast. The datasets include data from two commercially available devices- a stereoscopic-inertial camera and a multibeam 3D LiDAR, which also provides inertial measurements. Additionally, we used a tripod-mounted survey grade LiDAR scanner to capture a detailed millimeter-accurate 3D map of the test location (containing ∼290 million points). Using the map, we generated a 6 Degrees of Freedom (DoF) ground truth pose for each LiDAR scan (with approximately 3 cm accuracy) to enable better benchmarking of LiDAR and vision localisation, mapping and reconstruction systems. This ground truth is the particular novel contribution of this dataset and we believe that it will enable systematic evaluation which many similar datasets have lacked. The large dataset combines both built environments, open spaces and vegetated areas so as to test localisation and mapping systems such as vision-based navigation, visual and LiDAR SLAM, 3D LiDAR reconstruction and appearance-based place recognition, while the supplementary datasets contain very dynamic motions to introduce more challenges for visual-inertial odometry systems. The datasets are available at: ori.ox.ac.uk/datasets/newer-college-dataset

---

### [nuScenes](https://www.nuscenes.org/)
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

---

### [LIBRE](https://sites.google.com/g.sp.m.is.nagoya-u.ac.jp/libre-dataset)
- **Year**: 2020
- **Sensor**: 12x Spinning LiDAR (each), Camera, IMU, GNSS, CAN, 360° 4K camera, Event camera, Infrared camera, 3D point cloud map, Vector map
- **Objective**: Odometry
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: IV
- **Abstract**: In this work, we present LIBRE: LiDAR Benchmarking and Reference, a first-of-its-kind dataset featuring 10 different LiDAR sensors, covering a range of manufacturers, models, and laser configurations. Data captured independently from each sensor includes three different environments and configurations: static targets, where objects were placed at known distances and measured from a fixed position within a controlled environment; adverse weather, where static obstacles were measured from a moving vehicle, captured in a weather chamber where LiDARs were exposed to different conditions (fog, rain, strong light); and finally, dynamic traffic, where dynamic objects were captured from a vehicle driven on public urban roads, multiple times at different times of the day, and including supporting sensors such as cameras, infrared imaging, and odometry devices. LIBRE will contribute to the research community to (1) provide a means for a fair comparison of currently available LiDARs, and (2) facilitate the improvement of existing self-driving vehicles and robotics-related software, in terms of development and tuning of LiDAR-based perception algorithms.

---

### [DurLAR](https://github.com/l1997i/DurLAR)
- **Year**: 2021
- **Sensor**: 1x Ouster OS1-128, 1x Carnegie Robotics MultiSense S21 stereo camera, 1x OxTS RT3000v3 GNSS/INS, 1x Yocto Light V3 lux meter
- **Objective**: Depth Estimation
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: 3DV
- **Abstract**: We present DurLAR, a high-fidelity 128-channel 3D LiDAR dataset with panoramic ambient (near infrared) and reflectivity imagery, as well as a sample benchmark task using depth estimation for autonomous driving applications. Our driving platform is equipped with a high resolution 128 channel LiDAR, a 2MPix stereo camera, a lux meter and a GNSS/INS system. Ambient and reflectivity images are made available along with the LiDAR point clouds to facilitate multi-modal use of concurrent ambient and reflectivity scene information. Leveraging DurLAR, with a resolution exceeding that of prior benchmarks, we consider the task of monocular depth estimation and use this increased availability of higher resolution, yet sparse ground truth scene depth information to propose a novel joint supervised/selfsupervised loss formulation. We compare performance over both our new DurLAR dataset, the established KITTI benchmark and the Cityscapes dataset. Our evaluation shows our joint use supervised and self-supervised loss terms, enabled via the superior ground truth resolution and availability within DurLAR improves the quantitative and qualitative performance of leading contemporary monocular depth estimation approaches (RMSE = 3.639, SqRel = 0.936).

---

### [EU Long-term Dataset](https://epan-utbm.github.io/utbm_robocar_dataset/)
- **Year**: 2021
- **Sensor**: 2x HDL-32E, 2x Pixelink PL-B742F, 1x ibeo LUX 4L, 1x Continental ARS 308, 1x SICK LMS100-10000, 1x Magellan ProFlex 500, 1x Xsens MTi-28A53G25
- **Objective**: Odometry
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: IROS
- **Abstract**: The field of autonomous driving has grown tremendously over the past few years, along with the rapid progress in sensor technology. One of the major purposes of using sensors is to provide environment perception for vehicle understanding, learning and reasoning, and ultimately interacting with the environment. In this paper, we first introduce a multisensor platform allowing vehicle to perceive its surroundings and locate itself in a more efficient and accurate way. The platform integrates eleven heterogeneous sensors including various cameras and lidars, a radar, an IMU (Inertial Measurement Unit), and a GPS-RTK (Global Positioning System / Real-Time Kinematic), while exploits a ROS (Robot Operating System) based software to process the sensory data. Then, we present a new dataset (https://epan-utbm.github.io/utbm_robocar_dataset/) for autonomous driving captured many new research challenges (e.g. highly dynamic environment), and especially for long-term autonomy (e.g. creating and maintaining maps), collected with our instrumented vehicle, publicly available to the community.

---

### [NTU VIRAL Dataset](https://ntu-aris.github.io/ntu_viral_dataset/)

- **Year**: 2021
- **Sensor**: 2x OS1-16, 1x Xsens MTi-28A53G25, 1x Humatic P440, 1x Leica MS60 TotalStation
- **Objective**: Odometry
- **Environment**: Urban
- **System**: drone
- **Publication**: IJRR
- **Abstract**: In recent years, autonomous robots have become ubiquitous in research and daily life. Among many factors, public datasets play an important role in the progress of this field, as they waive the tall order of initial investment in hardware and manpower. However, for research on autonomous aerial systems, there appears to be a relative lack of public datasets on par with those used for autonomous driving and ground robots. Thus, to fill in this gap, we conduct a data collection exercise on an aerial platform equipped with an extensive and unique set of sensors: two 3D lidars, two hardware-synchronized global-shutter cameras, multiple Inertial Measurement Units (IMUs), and especially, multiple Ultra-wideband (UWB) ranging units. The comprehensive sensor suite resembles that of an autonomous driving car, but features distinct and challenging characteristics of aerial operations. We record multiple datasets in several challenging indoor and outdoor conditions. Calibration results and ground truth from a high-accuracy laser tracker are also included in each package. All resources can be accessed via our webpage https://ntu-aris.github.io/ntu_viral_dataset/.

---

### [M2DGR](https://github.com/SJTU-ViSYS/M2DGR)
- **Year**: 2021
- **Sensor**: 1x VLP-32C, 1x Handsfree A9, 1x Ublox M8T, 1x FLIR, 1x Pointgrey CM3-U3-13Y3C, 1x Gaode PLUG 617, 1x Inivation DVXplorer, 1x Realsense d435i, 1x Vicon Vero 2.2, 1x Leica Nova MS60, 1x Xsens Mti 680G
- **Objective**: Odometry
- **Environment**: Urban
- **System**: UGV
- **Publication**: RA-L
- **Abstract**: We introduce M2DGR: a novel large-scale dataset collected by a ground robot with a full sensor-suite including six f ish-eye and one sky-pointing RGB cameras, an infrared camera, an event camera, a Visual-Inertial Sensor (VI-sensor), an inertial measurement unit (IMU), a LiDAR, a consumer-grade Global Navigation Satellite System (GNSS) receiver and a GNSS-IMU navigation system with real-time kinematic (RTK) signals. All those sensors were well-calibrated and synchronized, and their data were recorded simultaneously. The ground truth trajectories were obtained by the motion capture device, a laser 3D tracker, and an RTK receiver. The dataset comprises 36 sequences (about 1TB) captured in diverse scenarios including both indoor and outdoor environments. We evaluate state-of-the-art SLAM algorithms on M2DGR. Results show that existing solutions perform poorly in some scenarios. For the benefit of the research community, we make the dataset and tools public. The webpage of our project is https://github.com/SJTU-ViSYS/M2DGR.

---

### [Pandaset](https://pandaset.org/)
- **Year**: 2021
- **Sensor**: 1x Hesai Pandar64, 1x Hesai PandarGT, 5x Leopard LI-USB30-AR023ZWDRB, 1x Leopard LI-USB30-AR023ZWDRB, 1x NovAtel PwrPak7
- **Objective**: Segmentation
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: ITSC
- **Abstract**: The accelerating development of autonomous driving technology has placed greater demands on obtaining large amounts of high-quality data. Representative, labeled, real world data serves as the fuel for training deep learning networks, critical for improving self-driving perception algorithms. In this paper, we introduce PandaSet, the first dataset produced by a complete, high-precision autonomous vehicle sensor kit with a no-cost commercial license. The dataset was collected using one 360◦ mechanical spinning LiDAR, one forwardfacing, long-range LiDAR, and 6 cameras. The dataset contains more than 100 scenes, each of which is 8 seconds long, and provides 28 types of labels for object classification and 37 types of labels for semantic segmentation. We provide baselines for LiDAR-only 3D object detection, LiDAR-camera fusion 3D object detection and LiDAR point cloud segmentation. For more details about PandaSet and the development kit, see https://scale.com/open-datasets/pandaset.

---

### [UrbanNav Dataset]((https://github.com/IPNL-POLYU/UrbanNavDataset))
- **Year**: 2021
- **Sensor**: 1x HDL-32E, 1x VLP-16, 1x Xsens Mti 10, 3x u-blox ZED-F9P, 1x EVK-M8T, 1x NovAtel Flexpak6, 1x ZED2 Stereo, 1x SPAN-CPT
- **Objective**: Odometry
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: ION
- **Abstract**: Urban canyon is typical in megacities like Hong Kong and Tokyo. Accurate positioning in urban canyons remains a challenging problem for the applications with navigation requirements, such as the navigation for pedestrian, autonomous driving vehicles and unmanned aerial vehicles. The GNSS positioning can be significantly degraded in urban canyons, due to the signal blockage by tall buildings. The visual positioning and LiDAR positioning can be considerably affected by numerous dynamic objects. To facilitate the research and development of robust, accurate and precise positioning using multiple sensors in urban canyons, we build a multi-sensoroy dataset collected in diverse challenging urban scenarios in Hong Kong and Tokyo, that provides full-suite sensor data, which includes GNSS, INS, LiDAR and cameras. We call this open-sourced dataset, UrbanNav. In 2019, we formed a joint working group under the joint efforts from International Association of Geodesy (IAG) and ION. This working group is currently under Sub-Commission 4.1: Emerging Positioning Technologies and GNSS Augmentations of IAG. After consolidating the suggestions and comments from intenational navigation researchers, the objectives of this work group are: 1. Open-sourcing positioning sensor data, including GNSS, INS, LiDAR and cameras collected in Asian urban canyons; 2. Raising the awareness of the urgent navigation requirement in highly-urbanized areas, especially in Asian-Pacific regions; 3. Providing an integrated online platform for data sharing to facilitate the development of navigation solutions of the research community; and 4. Benchmarking positioning algorithms based on the open-sourcing data. Currently, two pilot dataset can be downloaded by the link in the following. https://www.polyu-ipn-lab.com/download We also provide a GitHub page to answer possible issues that users may encounter. Meanwhile, we also provide example usage of the dataset for applications of LiDAR simultaneous localization and mapping (SLAM) and visual-inertial navigation system (VINS), etc. https://github.com/weisongwen/UrbanNavDataset In this conference paper, we will introduce the detail sensors setup, data format, the calibration of the intrinsic/extrinsic parameters, and the ground truth generation. We believe this opensource dataset can facilitate to identify the challenges of different sensors in urban canyons. Finally we will address the future maintenance directions of the UrbanNav dataset including the following: 1. Building a website to let the researchers upload their paper and result that evaluated based on the open-source data in terms of the proposed criteria. 2. Identifying the experts in the field to design the assessment criteria for different positioning algorithms. 3. Reporting the performance of the state-of-the-art positioning and integration algorithms in the urban canyons every 2 years.

---

### [Livox Simu-Dataset](https://www.livoxtech.com/simu-dataset)

- **Year**: 2021
- **Sensor**: 5x Horizons, 1x Tele-15
- **Objective**: Object Detection, Segmentation
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: None
- **Abstract**: Livox Simu-dataset v1.0 is a completely public data set. It can support tasks such as 3D object detection and semantic segmentation, aiming to help customers quickly verify algorithms and assist in the development of Livox Lidar applications.

---

### [Hilti 2021 SLAM dataset](https://hilti-challenge.com/dataset-2021.html)

- **Year**: 2021
- **Sensor**: 1x Sevensense Alphasense, 1x Ouster OS0-64, 1x Livox MID70, 1x ADIS16445 IMU
- **Objective**: SLAM 
- **Environment**: Indoor, Outdoor (Small)
- **System**: Handheld
- **Publication**: Arxiv
- **Abstract**: Research in Simultaneous Localization and Mapping (SLAM) has made outstanding progress over the past years. SLAM systems are nowadays transitioning from academic to real world applications. However, this transition has posed new demanding challenges in terms of accuracy and robustness. To develop new SLAM systems that can address these challenges, new datasets containing cutting-edge hardware and realistic scenarios are required. We propose the Hilti SLAM Challenge Dataset. Our dataset contains indoor sequences of offices, labs, and construction environments and outdoor sequences of construction sites and parking areas. All these sequences are characterized by featureless areas and varying illumination conditions that are typical in real-world scenarios and pose great challenges to SLAM algorithms that have been developed in confined lab environments. Accurate sparse ground truth, at millimeter level, is provided for each sequence. The sensor platform used to record the data includes a number of visual, lidar, and inertial sensors, which are spatially and temporally calibrated. The purpose of this dataset is to foster the research in sensor fusion to develop SLAM algorithms that can be deployed in tasks where high accuracy and robustness are required, e.g., in construction environments. Many academic and industrial groups tested their SLAM systems on the proposed dataset in the Hilti SLAM Challenge. The results of the challenge, which are summarized in this paper, show that the proposed dataset is an important asset in the development of new SLAM algorithms that are ready to be deployed in the real-world.

---

### [S3LI Dataset](https://www.dlr.de/rm/en/s3li_dataset/#gallery/37227)

- **Year**: 2022
- **Sensor**: 2x AVT Mako, Blickfeld Cube-1, XSens MTi-G 10, Ublox f9p
- **Objective**: SLAM
- **Environment**: Unsctructured
- **System**: Handheld
- **Publication**: RA-L
- **Abstract**: We present the DLR Planetary Stereo, Solid-State LiDAR, Inertial (S3LI) dataset, recorded on Mt. Etna, Sicily, an environment analogous to the Moon and Mars, using a hand-held sensor suite with attributes suitable for implementation on a space-like mobile rover. The environment is characterized by challenging conditions regarding both the visual and structural appearance: severe visual aliasing poses significant limitations to the ability of visual SLAM systems to perform place recognition, while the absence of outstanding structural details, joined with the limited Field-of-View of the utilized Solid-State LiDAR sensor, challenges traditional LiDAR SLAM for the task of pose estimation using point clouds alone. With this data, that covers more than 4 kilometers of travel on soft volcanic slopes, we aim to: 1) provide a tool to expose limitations of state-of-the-art SLAM systems with respect to environments, which are not present in widely available datasets and 2) motivate the development of novel localization and mapping approaches, that rely efficiently on the complementary capabilities of the two sensors. The dataset is accessible at the following url: https://rmc.dlr.de/s3li_dataset


---

### [STHEREO](https://sites.google.com/view/rpmsthereo/)

- **Year**: 2022
- **Sensor**: 2x FLIR A-65, 2x FLIR Flea-3, Ouster OS1-128, Xsens MTi-300, Novatel CPT-7
- **Objective**: SLAM
- **Environment**: Campus, suburban
- **System**: Vehicle
- **Publication**: IROS
- **Abstract**: This paper introduces a stereo thermal camera dataset (STheReO) with multiple navigation sensors to encour- age thermal SLAM researches. A thermal camera measures infrared rays beyond the visible spectrum therefore it could provide a simple yet robust solution to visually degraded envi- ronments where existing visual sensor-based SLAM would fail. Existing thermal camera datasets mostly focused on monocular configuration using the thermal camera with RGB cameras in a visually challenging environment. A few stereo thermal rig were examined but in computer vision perspective without supporting sequential images for state estimation algorithms. To encourage the academia for the evolving stereo thermal SLAM, we obtain nine sequences in total across three spatial locations and three different times per location (e.g., morning, day, and night) to capture the variety of thermal characteristics. By using the STheReO dataset, we hope diverse types of researches will be made, including but not limited to odom- etry, mapping, and SLAM (e.g., thermal-LiDAR mapping or long-term thermal localization). Our datasets are available at https://sites.google.com/view/rpmsthereo/.

---

### [ORFD](https://github.com/chaytonmin/Off-Road-Freespace-Detection)

- **Year**: 2022
- **Sensor**: Hesai Pandora40, 5x Cameras
- **Objective**: Detection
- **Environment**: Off-road 
- **System**: Vehicle
- **Publication**: ICRA
- **Abstract**: Freespace detection is an essential component of autonomous driving technology and plays an important role in trajectory planning. In the last decade, deep learning based freespace detection methods have been proved feasible. However, these efforts were focused on urban road environments and few deep learning based methods were specifically designed for off-road freespace detection due to the lack of off-road dataset and benchmark. In this paper, we present the ORFD dataset, which, to our knowledge, is the first off-road freespace detection dataset. The dataset was collected in different scenes (woodland, farmland, grassland and countryside), different weather conditions (sunny, rainy, foggy and snowy) and different light conditions (bright light, daylight, twilight, darkness), which totally contains 12,198 LiDAR point cloud and RGB image pairs with the traversable area, non-traversable area and unreachable area annotated in detail. We propose a novel network named OFF-Net, which unifies Transformer architecture to aggregate local and global information, to meet the requirement of large receptive fields for freespace detection task. We also propose the crossattention to dynamically fuse LiDAR and RGB image information for accurate off-road freespace detection. Dataset and code are publicly available at https://github. com/chaytonmin/OFF-Net.

---

### [Tiers](https://github.com/TIERS/tiers-lidars-dataset)

- **Year**: 2022
- **Sensor**: 1x Velodyne VLP-16, 1x Ouster OS1-64, 1x Ouster OS0-128, 1x Livox Horizon, 1x Livox Avia, 1x RealSense L515, integrated IMU
- **Objective**: SLAM
- **Environment**: Outdoor, Indoor, Forest
- **System**: Vehicle
- **Publication**: IROS 
- **Abstract**: Lidar technology has evolved significantly over the last decade, with higher resolution, better accuracy, and lower cost devices available today. In addition, new scanning modalities and novel sensor technologies have emerged in recent years. Public datasets have enabled benchmarking of algorithms and have set standards for the cutting edge technology. However, existing datasets are not representative of the technological landscape, with only a reduced number of lidars available. This inherently limits the development and comparison of generalpurpose algorithms in the evolving landscape. This paper presents a novel multi-modal lidar dataset with sensors showcasing different scanning modalities (spinning and solid-state), sensing technologies, and lidar cameras. The focus of the dataset is on low-drift odometry, with ground truth data available in both indoors and outdoors environment with sub-millimeter accuracy from a motion capture (MOCAP) system. For comparison in longer distances, we also include data recorded in larger spaces indoors and outdoors. The dataset contains point cloud data from spinning lidars and solid-state lidars. Also, it provides range images from high resolution spinning lidars, RGB and depth images from a lidar camera, and inertial data from built-in IMUs. This is, to the best of our knowledge, the lidar dataset with the most variety of sensors and environments where ground truth data is available. This dataset can be widely used in multiple research areas, such as 3D LiDAR simultaneous localization and mapping (SLAM), performance comparison between multi-modal lidars, appearance recognition and loop closure detection. The datasets are available at: https://github.com/TIERS/tiers-lidarsdataset.

---

### [FusionPortable](https://fusionportable.github.io/dataset/fusionportable/)

- **Year**: 2022
- **Sensor**: 1x Ouster OS1-128, 2x FLIR BFS-U3-31S4C, 2x DAVIS346, 1x STIM300, 1x ZED-F9P
- **Objective**: SLAM 
- **Environment**: Indoor, Outdoor, Campus
- **System**: Vehicle, Handheld, quadrupled Robot
- **Publication**: IROS
- **Abstract**: Combining multiple sensors enables a robot to maximize its perceptual awareness of environments and enhance its robustness to external disturbance, crucial to robotic navigation. This paper proposes the FusionPortable benchmark, a complete multi-sensor dataset with a diverse set of sequences for mobile robots. This paper presents three contributions. We first advance a portable and versatile multi-sensor suite that offers rich sensory measurements: 10Hz LiDAR point clouds, 20Hz stereo frame images, high-rate and asynchronous events from stereo event cameras, 200Hz inertial readings from an IMU, and 10Hz GPS signal. Sensors are already temporally synchronized in hardware. This device is lightweight, self-contained, and has plug-and-play support for mobile robots. Second, we construct a dataset by collecting 17 sequences that cover a variety of environments on the campus by exploiting multiple robot platforms for data collection. Some sequences are challenging to existing SLAM algorithms. Third, we provide ground truth for the decouple localization and mapping performance evaluation. We additionally evaluate state-of-the-art SLAM approaches and identify their limitations. The dataset, consisting of raw sensor easurements, ground truth, calibration data, and evaluated algorithms, will be released: https://ram-lab.com/file/site/multi-sensor-dataset

---

### [Hilti 2022 SLAM Dataset](https://hilti-challenge.com/dataset-2022.html)

- **Year**: 2022
- **Sensor**: 1x Alphasense Core, 1x Hesai PandarXT-32, 1x Bosch BMI085 IMU
- **Objective**: SLAM
- **Environment**: Indoor, Construction site
- **System**: Handheld
- **Publication**: RA-L 
- **Abstract**: Simultaneous Localization and Mapping (SLAM) is being deployed in real-world applications, however many state-of-the-art solutions still struggle in many common scenarios. A key necessity in progressing SLAM research is the availability of high-quality datasets and fair and transparent benchmarking. To this end, we have created the Hilti-Oxford Dataset, to push state-of-the-art SLAM systems to their limits. The dataset has a variety of challenges ranging from sparse and regular construction sites to a 17th century neoclassical building with fine details and curved surfaces. To encourage multi-modal SLAM approaches, we designed a data collection platform featuring a lidar, five cameras, and an IMU (Inertial Measurement Unit). With the goal of benchmarking SLAM algorithms for tasks where accuracy and robustness are paramount, we implemented a novel ground truth collection method that enables our dataset to accurately measure SLAM pose errors with millimeter accuracy. To further ensure accuracy, the extrinsics of our platform were verified with a micrometer-accurate scanner, and temporal calibration was managed online using hardware time synchronization. The multi-modality and diversity of our dataset attracted a large field of academic and industrial researchers to enter the second edition of the Hilti SLAM challenge, which concluded in June 2022. The results of the challenge show that while the top three teams could achieve an accuracy of 2cm or better for some sequences, the performance dropped off in more difficult sequences.

---

### [USTC FLICAR](https://ustc-flicar.github.io/)
N
- **Year**: 2023
- **Sensor**: 1x Velodyne HDL-32E, 1x Ouster OS0-128, 1x Livox Avia, 1x Velodyne VLP-32C, 1x PointGrey Bumblebee xb3, 1x PointGrey Bumblebee xb2, 1x Hikvision MV-CB016-10GC-C, 1x Hikvision MV-CE060-10UC, 1x Xsens MTi-G-710
- **Objective**: SLAM, Semantic Segmentation 
- **Environment**: Urban
- **System**: Ground and Aerial acquisition system
- **Publication**: IJRR
- **Abstract**: In this paper, we present the USTC FLICAR Dataset, which is dedicated to the development of simultaneous localization and mapping and precise 3D reconstruction of the workspace for heavy-duty autonomous aerial work robots. In recent years, numerous public datasets have played significant roles in the advancement of autonomous cars and unmanned aerial vehicles (UAVs). However, these two platforms differ from aerial work robots: UAVs are limited in their payload capacity, while cars are restricted to two-dimensional movements. To fill this gap, we create the "Giraffe" mapping robot based on a bucket truck, which is equipped with a variety of well-calibrated and synchronized sensors: four 3D LiDARs, two stereo cameras, two monocular cameras, Inertial Measurement Units (IMUs), and a GNSS/INS system. A laser tracker is used to record the millimeter-level ground truth positions. We also make its ground twin, the "Okapi" mapping robot, to gather data for comparison. The proposed dataset extends the typical autonomous driving sensing suite to aerial scenes, demonstrating the potential of combining autonomous driving perception systems with bucket trucks to create a versatile autonomous aerial working platform. Moreover, based on the Segment Anything Model (SAM), we produce the Semantic FLICAR dataset, which provides fine-grained semantic segmentation annotations for multimodal continuous data in both temporal and spatial dimensions. The dataset is available for download at: https://ustc-flicar.github.io/

---


### [Wild Places](https://csiro-robotics.github.io/Wild-Places/)


- **Year**: 2023
- **Sensor**: 1x VLP-16, 4x e-CAM130ACUXVR Camera, 1x 3DM-CV5-25 IMU
- **Objective**: Place Recognition
- **Environment**: Forest 
- **System**: Handheld
- **Publication**: ICRA
- **Abstract**: Many existing datasets for lidar place recognition are solely representative of structured urban environments, and have recently been saturated in performance by deep learning based approaches. Natural and unstructured environments present many additional challenges for the tasks of long-term localisation but these environments are not represented in currently available datasets. To address this we introduce Wild-Places, a challenging large-scale dataset for lidar place recognition in unstructured, natural environments. Wild-Places contains eight lidar sequences collected with a handheld sensor payload over the course of fourteen months, containing a total of 63K undistorted lidar submaps along with accurate 6DoF ground truth. Our dataset contains multiple revisits both within and between sequences, allowing for both intra-sequence (i.e. loop closure detection) and inter-sequence (i.e. re-localisation) place recognition. We also benchmark several state-of-the-art approaches to demonstrate the challenges that this dataset introduces, particularly the case of long-term place recognition due to natural environments changing over time.

---

### [Hilti 2023 SLAM Dataset](https://hilti-challenge.com/dataset-2023.html)

- **Year**: 2023
- **Sensor**: 1x Robosense Bpearl, 4x Luxonis OAK-D, 1x XSens MTi670
- **Objective**: SLAM
- **Environment**: Construction site
- **System**: Handheld & UGV
- **Publication**: Arxiv
- **Abstract**: Simultaneous Localization and Mapping systems are a key enabler for positioning in both handheld and robotic applications. The Hilti SLAM Challenges organized over the past years have been successful at benchmarking some of the world’s best SLAMSystems with high accuracy. However, more capabilities of these systems are yet to be explored, such as platform agnosticism across varying sensor suites and multi-session SLAM. These factors indirectly serve as an indicator of robustness and ease of deployment in real-world applications. There exists no dataset plus benchmark combination publicly available, which considers these factors combined. The Hilti SLAM Challenge 2023 Dataset and Benchmark addresses this issue. Additionally, we propose a novel fiducial marker design for a pre-surveyed point on the ground to be observable from an off-the-shelf LiDAR mounted on a robot, and an algorithm to estimate its position at mm-level accuracy. Results from the challenge show an increase in overall participation, single-session SLAM systems getting increasingly accurate, successfully operating across varying sensor suites, but relatively few participants performing multi-session SLAM. Dataset URL: https://www.hilti-challenge.com/dataset-2023.html

---

### [City Dataset](https://github.com/minwoo0611/MA-LIO)

- **Year**: 2023
- **Sensor**: 1x OS2-128, 1x Livox Tele, 1x Livox Avia, 1x Xsens MTi-300
- **Objective**: SLAM
- **Environment**: City
- **System**: Vehicle
- **Publication**: RA-L 
- **Abstract**: In recent years, multiple Light Detection and Ranging (LiDAR) systems have grown in popularity due to their enhanced accuracy and stability from the increased field of view (FOV). However, integrating multiple LiDARs can be challenging, attributable to temporal and spatial discrepancies. Common practice is to transform points among sensors while requiring strict time synchronization or approximating transformation among sensor frames. Unlike existing methods, we elaborate the inter-sensor transformation using continuous-time (CT) inertial measurement unit (IMU) modeling and derive associated ambiguity as a point-wise uncertainty. This uncertainty, modeled by combining the state covariance with the acquisition time and point range, allows us to alleviate the strict time synchronization and to overcome FOV difference. The proposed method has been validated on both public and our datasets and is compatible with various LiDAR manufacturers and scanning patterns. We open-source the code for public access at https://github.com/minwoo0611/MA-LIO.

---

### [Ground-Challenge](https://github.com/sjtuyinjie/Ground-Challenge)
- **Year**: 2023
- **Sensor**: 1x Velodyne VLP-16, 1x Realsense d435i, 1x Xsens Mti-300 IMU, 1x AgileX Wheel Odometer
- **Objective**: SLAM
- **Environment**: Indoor
- **System**: UGV
- **Publication**: ROBIO 
- **Abstract**: We introduce Ground-Challenge: a novel dataset collected by a ground robot with multiple sensors including an RGB-D camera, an inertial measurement unit (IMU), a wheel odometer and a 3D LiDAR to support the research on corner cases of visual SLAM systems. Our dataset comprises 36 trajectories with diverse corner cases such as aggressive motion, severe occlusion, changing illumination, few textures, pure rotation, motion blur, wheel suspension, etc. Some state-of-the-art SLAM algorithms are tested on our dataset, showing that these systems are seriously drifting and even failing on specific sequences. We will release the dataset and relevant materials upon paper publication to benefit the research community.

---

### [RACECAR](https://github.com/linklab-uva/RACECAR_DATA)

- **Year**: 2023
- **Sensor**: 3x Luminar H3, 2x Novatel Pwrpak 7d, 6x Allied Vision Mak G319C, 2x Aptiv RADAR
- **Objective**: Localization, Object Detection
- **Environment**: Racing (High Speed)
- **System**: Vehicle
- **Publication**: Arxiv
- **Abstract**: This paper describes the first open dataset for fullscale and high-speed autonomous racing. Multi-modal sensor data has been collected from fully autonomous Indy race cars operating at speeds of up to 170 mph (273 kph). Six teams who raced in the Indy Autonomous Challenge have contributed to this dataset. The dataset spans 11 interesting racing scenarios across two race tracks which include solo laps, multi-agent laps, overtaking situations, high-accelerations, banked tracks, obstacle avoidance, pit entry and exit at different speeds. The dataset contains data from 27 racing sessions across the 11 scenarios with over 6.5 hours of sensor data recorded from the track. The data is organized and released in both ROS2 and nuScenes format. We have also developed the ROS2-to-nuScenes conversion library to achieve this. The RACECAR data is unique because of the high-speed environment of autonomous racing. We present several benchmark problems on localization, object detection and tracking (LiDAR, Radar, and Camera), and mapping using the RACECAR data to explore issues that arise at the limits of operation of the vehicle.

---

### [ConSLAM](https://github.com/mac137/ConSLAM)

- **Year**: 2023
- **Sensor**: 1x Velodyne VLP-16, 1x Alvium U-319c, 1x Alvium 1800 U-501, 1x Xsens MTi-610
- **Objective**: SLAM
- **Environment**: Construction Site
- **System**: Handheld
- **Publication**: ECCV 
- **Abstract**: This paper presents a dataset collected periodically on a construction site. The dataset aims to evaluate the performance of SLAM algorithms used by mobile scanners or autonomous robots. It includes ground-truth scans of a construction site collected using a terrestrial laser scanner along with five sequences of spatially registered and time-synchronized images, LiDAR scans and inertial data coming from our prototypical hand-held scanner. We also recover the ground-truth trajectory of the mobile scanner by registering the sequential LiDAR scans to the ground-truth scans and show how to use a popular software package to measure the accuracy of SLAM algorithms against our trajectory automatically. To the best of our knowledge, this is the first publicly accessible dataset consisting of periodically collected sequential data on a construction site.

---

### [Pohang Canal Dataset](https://sites.google.com/view/pohang-canal-dataset/home?authuser=0)

- **Year**: 2023
- **Sensor**: 1x OS1-64, 2x OS1-32, 1x Simrad Radar, 1x Flir Ladybug 5+, 1x Flir blackfly, 1x Flir A65 InfraRed, 1x Microstrain 3DM-GX5-45, 2x SinoGNSS AT340 GNSS, 1x TDR-3000 GNSS-RTK Receiver
- **Objective**: Autonomous Navigation
- **Environment**: Canal 
- **System**: Ship
- **Publication**: IJRR 
- **Abstract**: This paper presents a multimodal maritime dataset and the data collection procedure used to gather it, which aims to facilitate autonomous navigation in restricted water environments. The dataset comprises measurements obtained using various perception and navigation sensors, including a stereo camera, an infrared camera, an omnidirectional camera, three LiDARs, a marine radar, a global positioning system, and an attitude heading reference system. The data were collected along a 7.5-km-long route that includes a narrow canal, inner and outer ports, and near-coastal areas in Pohang, South Korea. The collection was conducted under diverse weather and visual conditions. The dataset and its detailed description are available for free download at https://sites.google.com/view/pohang-canal-dataset.

---

### [Boreas](https://www.boreas.utias.utoronto.ca/)

- **Year**: 2023
- **Sensor**: Velodyne Alpha-Prime (128-beam) lidar, a FLIR Blackfly S camera, a Navtech CIR304-H radar, and an Applanix POS LV GNSS-INS.
- **Objective**: Odometry, Metric Localization, 3D Object Detection
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: IJRR
- **Abstract**: The Boreas dataset was collected by driving a repeated route over the course of one year, resulting in stark seasonal variations and adverse weather conditions such as rain and falling snow. In total, the Boreas dataset includes over 350km of driving data featuring a 128-channel Velodyne Alpha Prime lidar, a 360◦ Navtech CIR304-H scanning radar, a 5MP FLIR Blackfly S camera, and centimetre-accurate post-processed ground truth poses. Our dataset will support live leaderboards for odometry, metric localization, and 3D object detection. The dataset and development kit are available at [boreas.utias.utoronto.ca.](boreas.utias.utoronto.ca.)

---

### [HeLiPR](https://sites.google.com/view/heliprdataset)
- **Year**: 2023
- **Sensor**: 1x Ouster OS2-128, 1x Velodyne VLP-16, 1x Livox Avia, 1x Aeva AeriesII, 1x Xsens MTi-300, 1x SPAN-CPT7
- **Objective**: Place Recognition, SLAM
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: IJRR
- **Abstract**: Place recognition is crucial for robot localization and loop closure in simultaneous localization and mapping (SLAM). Light Detection and Ranging (LiDAR), known for its robust sensing capabilities and measurement consistency even in varying illumination conditions, has become pivotal in various fields, surpassing traditional imaging sensors in certain applications. Among various types of LiDAR, spinning LiDARs are widely used, while non-repetitive scanning patterns have recently been utilized in robotics applications. Some LiDARs provide additional measurements such as reflectivity, Near Infrared (NIR), and velocity from Frequency modulated continuous wave (FMCW) LiDARs. Despite these advances, there is a lack of comprehensive datasets reflecting the broad spectrum of LiDAR configurations for place recognition. To tackle this issue, our paper proposes the HeLiPR dataset, curated especially for place recognition with heterogeneous LiDARs, embodying spatiotemporal variations. To the best of our knowledge, the HeLiPR dataset is the first heterogeneous LiDAR dataset supporting inter-LiDAR place recognition with both nonrepetitive and spinning LiDARs, accommodating different field of view (FOV)s and varying numbers of rays. The dataset covers diverse environments, from urban cityscapes to high-dynamic freeways, over a month, enhancing adaptability and robustness across scenarios. Notably, HeLiPR includes trajectories parallel to MulRan sequences, making it valuable for research in heterogeneous LiDAR place recognition and long-term studies. The dataset is accessible at https://sites.google.com/view/heliprdataset.

---

### [A Multi-LiDAR Multi-UAV Dataset](https://tiers.github.io/multi_lidar_multi_uav_dataset/)

- **Year**: 2023
- **Sensor**: 1x Ouster OS1-64, 1x Livox Mid360, 1x Livox Avia, 1x Intel RealSense D435, MOCAP
- **Objective**: Odomety, SLAM
- **Environment**: Indoor, Unstructured
- **System**: Drone
- **Publication**: ROBIO
- **Abstract**: With the increasing prevalence of drones in various industries, the navigation and tracking of unmanned aerial vehicles (UAVs) in challenging environments, particularly GNSSdenied areas, have become crucial concerns. To address this need, we present a novel multi-LiDAR dataset specifically designed for UAV tracking. Our dataset includes data from a spinning LiDAR, two solid-state LiDARs with different Field of View (FoV) and scan patterns, and an RGB-D camera. This diverse sensor suite allows for research on new challenges in the field, including limited FoV adaptability and multi-modality data processing. The dataset facilitates the evaluation of existing algorithms and the development of new ones, paving the way for advances in UAV tracking techniques. Notably, we provide data in both indoor and outdoor environments. We also consider variable UAV sizes, from micro-aerial vehicles to more standard commercial UAV platforms. The outdoor trajectories are selected with close proximity to buildings, targeting research in UAV detection in urban areas, e.g., within counter-UAV systems or docking for UAV logistics. Holybro X500 Autel Evo II Optitrack PrimeX 22 Camera Tello Fig. 1: Illustration of the hardware used in the experiments. At the bottom, the tracking sensors including Ouster OS1-64, Livox Mid360, Livox Avia and Intel RealSense D435. In addition to the dataset, we provide a baseline comparison with recent LiDAR-based UAV tracking algorithms, benchmarking the performance with different sensors, UAVs, and algorithms. Importantly, our dataset shows that current methods have shortcomings and are unable to track UAVs consistently across different scenarios.

---

### [ParisLuco3D](https://npm3d.fr/parisluco3d)

- **Year**: 2023
- **Sensor**: 1x Velodyne HDL 32E
- **Objective**: Segmentation, Detection
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: RA-L
- **Abstract**: LiDAR is an essential sensor for autonomous driving by collecting precise geometric information regarding a scene. As the performance of various LiDAR perception tasks has improved, generalizations to new environments and sensors has emerged to test these optimized models in real-world conditions. Unfortunately, the various annotation strategies of data providers complicate the computation of cross-domain performances. This paper provides a novel dataset, ParisLuco3D, specifically designed for cross-domain evaluation to make it easier to evaluate the performance utilizing various source datasets. Alongside the dataset, online benchmarks for LiDAR semantic segmentation, LiDAR object detection, and LiDAR tracking are provided to ensure a fair comparison across methods. The ParisLuco3D dataset, evaluation scripts, and links to benchmarks can be found at the following website: https://npm3d.fr/parisluco3d

---

### [MARS-LVIG dataset](https://journals.sagepub.com/doi/full/10.1177/02783649241227968)

- **Year**: 2024
- **Sensor**: 1x Livox Avia, 1x Hikvision CA-050-11UC, 1x u-blox ZED-F9P, 1x DJI L1
- **Objective**: SLAM
- **Environment**: Airfield, Island, Rural town, and Valley
- **System**: Drone
- **Publication**: IJRR
- **Abstract**: In recent years, advancements in Light Detection and Ranging (LiDAR) technology have made 3D LiDAR sensors more compact, lightweight, and affordable. This progress has spurred interest in integrating LiDAR with sensors such as Inertial Measurement Units (IMUs) and cameras for Simultaneous Localization and Mapping (SLAM) research. Public datasets covering different scenarios, platforms, and viewpoints are crucial for multi-sensor fusion SLAM studies, yet most focus on handheld or vehicle-mounted devices with front or 360-degree views. Data from aerial vehicles with downward-looking views is scarce, existing relevant datasets usually feature low altitudes and are mostly limited to small campus environments. To fill this gap, we introduce the Multi-sensor Aerial Robots SLAM dataset (MARS-LVIG dataset), providing unique aerial downward-looking LiDAR-Visual-Inertial-GNSS data with viewpoints from altitudes between 80 m and 130 m. The dataset not only offers new aspects to test and evaluate existing SLAM algorithms, but also brings new challenges which can facilitate researches and developments of more advanced SLAM algorithms. The MARS-LVIG dataset contains 21 sequences, acquired across diversified large-area environments including an aero-model airfield, an island, a rural town, and a valley. Within these sequences, the UAV has speeds varying from 3 m/s to 12 m/s, a scanning area reaching up to 577,000 m2, and the max path length of 7.148 km in a single flight. This dataset encapsulates data collected by a lightweight, hardware-synchronized sensor package that includes a solid-state 3D LiDAR, a global-shutter RGB camera, IMUs, and a raw message receiver of the Global Navigation Satellite System (GNSS). For algorithm evaluation, this dataset releases ground truth of both localization and mapping, which are acquired by on-board Real-time Kinematic (RTK) and DJI L1 (post-processed by its supporting software DJI Terra), respectively. The dataset can be downloaded from: https://mars.hku.hk/dataset.html.

---

### [M2DGR-plus](https://github.com/SJTU-ViSYS/M2DGR-plus?tab=readme-ov-file)

- **Year**: 2024
- **Sensor**: 1x Robosense 16, 1x Ublox F9p, 1x Realsense d435i, 1x Xsens Mti 680G, 1x Vicon Vero 2.2
- **Objective**: SLAM
- **Environment**: Outdoor (Bridge, Street, Parking Lot and Building)
- **System**: UGV
- **Publication**: ICRA 
- **Abstract**: We introduce Ground-Fusion, a low-cost sensor fusion simultaneous localization and mapping (SLAM) system for ground vehicles. Our system features efficient initialization, effective sensor anomaly detection and handling, realtime dense color mapping, and robust localization in diverse environments. We tightly integrate RGB-D images, inertial measurements, wheel odometer and GNSS signals within a factor graph to achieve accurate and reliable localization both indoors and outdoors. To ensure successful initialization, we propose an efficient strategy that comprises three different methods: stationary, visual, and dynamic, tailored to handle diverse cases. Furthermore, we develop mechanisms to detect sensor anomalies and degradation, handling them adeptly to maintain system accuracy. Our experimental results on both public and self-collected datasets demonstrate that Ground-Fusion outperforms existing low-cost SLAM systems in corner cases. We release the code and datasets at https://github.com/SJTUViSYS/Ground-Fusion.

---

### [ENWIDE](https://projects.asl.ethz.ch/datasets/doku.php?id=enwide)

- **Year**: 2024
- **Sensor**: 1x Ouster OS0-128
- **Objective**: Odometry
- **Environment**: Degraded Outdoor, Degraded Indoor
- **System**: Handheld
- **Publication**: ICRA 
- **Abstract**: We present COIN-LIO, a LiDAR Inertial Odometry pipeline that tightly couples information from LiDAR intensity with geometry-based point cloud registration. The focus of our work is to improve the robustness of LiDAR-inertial odometry in geometrically degenerate scenarios, like tunnels or flat fields. We project LiDAR intensity returns into an intensity image, and propose an image processing pipeline that produces filtered images with improved brightness consistency within the image as well as across different scenes. To effectively leverage intensity as an additional modality, we present a novel feature selection scheme that detects uninformative directions in the point cloud registration and explicitly selects patches with complementary image information. Photometric error minimization in the image patches is then fused with inertial measurements and point-to-plane registration in an iterated Extended Kalman Filter. The proposed approach improves accuracy and robustness on a public dataset. We additionally publish a new dataset, that captures five real-world environments in challenging, geometrically degenerate scenes. By using the additional photometric information, our approach shows drastically improved robustness against geometric degeneracy in environments where all compared baseline approaches fail.

---

### [LiDAR-Degeneray-Datasets](https://github.com/ntnu-arl/lidar_degeneracy_datasets)

- **Year**: 2024
- **Sensor**: 1x Ouster OS0-128, 1x VectorNav VN100, 1x Texas Instruments IWR6843AOP-EVM (Radar)
- **Objective**: Odometry
- **Environment**: Degraded Indoor
- **System**: Drone
- **Publication**: ICRA
- **Abstract**: Enabling autonomous robots to operate robustly in challenging environments is necessary in a future with increased autonomy. For many autonomous systems, estimation and odometry remains a single point of failure, from which it can often be difficult, if not impossible, to recover. As such robust odometry solutions are of key importance. In this work a method for tightly-coupled LiDAR-Radar-Inertial fusion for odometry is proposed, enabling the mitigation of the effects of LiDAR degeneracy by leveraging a complementary perception modality while preserving the accuracy of LiDAR in wellconditioned environments. The proposed approach combines modalities in a factor graph-based windowed smoother with sensor information-specific factor formulations which enable, in the case of degeneracy, partial information to be conveyed to the graph along the non-degenerate axes. The proposed method is evaluated in real-world tests on a flying robot experiencing degraded conditions including geometric self-similarity as well as obscurant occlusion. For the benefit of the community we release the datasets presented: https://github.com/ ntnu-arl/lidar_degeneracy_datasets.

---

### [BotanicGarden](https://github.com/robot-pesg/BotanicGarden)

- **Year**: 2024
- **Sensor**: 1x Velodyne VLP-16, 1x Livox Avia, 1x Xsens MTi-680G, 1x Leica RTC360, 1x Scout V1.0, 1x BMI088, 1x DALSA M1930, 1x DALSA C1930
- **Objective**: SLAM, Semantic Segmentation
- **Environment**: Unstructured Natural
- **System**: UGV
- **Publication**: RA-L 
- **Abstract**: The rapid developments of mobile robotics and autonomous navigation over the years are largely empowered by public datasets for testing and upgrading, such as sensor odometry and SLAM tasks. Impressive demos and benchmark scores have arisen, which may suggest the maturity of existing navigation techniques. However, these results are primarily based on moderate structured scenario testing. When transitioning to challenging unstructured environments, especially in GNSS-denied, texture-monotonous, and dense-vegetated natural fields, their performance can hardly sustain at a high level and requires further validation and improvement. To bridge this gap, we build a novel robot navigation dataset in a luxuriant botanic garden of more than 48000m2. Comprehensive sensors are used, including Gray and RGB stereo cameras, spinning and MEMS 3D LiDARs, and low-cost and industrial-grade IMUs, all of which are well calibrated and hardware-synchronized. An all-terrain wheeled robot is employed for data collection, traversing through thick woods, riversides, narrow trails, bridges, and grasslands, which are scarce in previous resources. This yields 33 short and long sequences, forming 17.1km trajectories in total. Excitedly, both highly-accurate ego-motions and 3D map ground truth are provided, along with fine-annotated vision semantics. We firmly believe that our dataset can advance robot navigation and sensor fusion research to a higher level.

---

### [WOMD Dataset](https://waymo.com/open/data/motion/)

- **Year**: 2024
- **Sensor**: 1x mid-range lidar, 4x short-range lidars
- **Objective**: Motion Forecasting 
- **Environment**: Urban
- **System**: Vehicle
- **Publication**: ICRA
- **Abstract**: Widely adopted motion forecasting datasets substitute the observed sensory inputs with higher-level abstractions such as 3D boxes and polylines. These sparse shapes are inferred through annotating the original scenes with perception systems’ predictions. Such intermediate representations tie the quality of the motion forecasting models to the performance of computer vision models. Moreover, the human-designed explicit interfaces between perception and motion forecasting typically pass only a subset of the semantic information present in the original sensory input. To study the effect of these modular approaches, design new paradigms that mitigate these limitations, and accelerate the development of end-to-end motion forecasting models, we augment the Waymo Open Motion Dataset (WOMD) with large-scale, high-quality, diverse LiDAR data for the motion forecasting task. The new augmented dataset (WOMD-LiDAR)1 consists of over 100,000 scenes that each spans 20 seconds, consisting of well-synchronized and calibrated high quality LiDAR point clouds captured across a range of urban and suburban geographies. Compared to Waymo Open Dataset (WOD), WOMDLiDAR dataset contains 100× more scenes. Furthermore, we integrate the LiDAR data into the motion forecasting model training and provide a strong baseline. Experiments show that the LiDAR data brings improvement in the motion forecasting task. We hope that WOMD-LiDAR will provide new opportunities for boosting end-to-end motion forecasting models.

---
### [3DRef](https://3dref.github.io/)

- **Year**: 2024
- **Sensor**: 1x OS0-128, 1x Livox Avia, 1x Hesai QT64, 1x Insta360
- **Objective**: Reflection Detection (Semantic Segmentation)
- **Environment**: Indoor
- **System**: Handheld
- **Publication**: 3DV
- **Abstract**: Reflective surfaces present a persistent challenge for reliable 3D mapping and perception in robotics and autonomous systems. However, existing reflection datasets and benchmarks remain limited to sparse 2D data. This paper introduces the first large-scale 3D reflection detection dataset containing more than 50,000 aligned samples of multi-return Lidar, RGB images, and 2D/3D semantic labels across diverse indoor environments with various reflections. Textured 3D ground truth meshes enable automatic point cloud labeling to provide precise ground truth annotations. Detailed benchmarks evaluate three Lidar point cloud segmentation methods, as well as current state-of-theart image segmentation networks for glass and mirror detection. The proposed dataset advances reflection detection by providing a comprehensive testbed with precise global alignment, multi-modal data, and diverse reflective objects and materials. It will drive future research towards reliable reflection detection. The dataset is publicly available at http://3dref.github.io

---

### [FusionPortableV2](https://fusionportable.github.io/dataset/fusionportable_v2/#various-platforms-and-scenarios)
- **Year**: 2024
- **Sensor**: 1x Ouster OS1-128, 1x FLIR BFS-U3-31S4C, 1x DAVIS346, 1x STIM300, 1x MPU6150, 1x ICM20948, 1x 3DM-GQ7-GNSS/INS, 1x Omron E6B2-CWZ6C, 1x Leica BLK360, 1x Leica MS60
- **Objective**: SLAM
- **Environment**: Indoor and Outdoor
- **System**: Handheld, Legged Robot, UGV, Vehicle
- **Publication**: IJRR
- **Abstract**: Simultaneous Localization and Mapping (SLAM) technology has been widely applied in various robotic scenarios, from rescue operations to autonomous driving. However, the generalization of SLAM algorithms remains a significant challenge, as current datasets often lack scalability in terms of platforms and environments. To address this limitation, we present FusionPortableV2, a multi-sensor SLAM dataset featuring sensor diversity, varied motion patterns, and a wide range of environmental scenarios. Our dataset comprises 27 sequences, spanning over 2.5 hours and collected from four distinct platforms: a handheld suite, a legged robot, a unmanned ground vehicle (UGV), and a vehicle. These sequences cover diverse settings, including buildings, campuses, and urban areas, with a total length of 38.7km. Additionally, the dataset includes ground-truth (GT) trajectories and RGB point cloud maps covering approximately 0.3km2. To validate the utility of our dataset in advancing SLAM research, we assess several state-of-the-art (SOTA) SLAM algorithms. Furthermore, we demonstrate the dataset’s broad application beyond traditional SLAM tasks by investigating its potential for monocular depth estimation. The completae dataset, including sensor data, GT, and calibration details, is accessible at https://fusionportable.github.io/dataset/fusionportable v2.

---

### [HeLiMOS](https://sites.google.com/view/helimos)

- **Year**: 2024
- **Sensor**: 1x Ouster OS2-128, 1x Velodyne VLP-16, 1x Livox Avia, 1x Aeva AeriesII, 1x Xsens MTi-300, 1x SPAN-CPT7
- **Objective**: Moving Object Segmentation
- **Environment**: Outdoor
- **System**: Vehicle
- **Publication**: IROS
- **Abstract**: Moving object segmentation (MOS) using a 3D light detection and ranging (LiDAR) sensor is crucial for scene understanding and identification of moving objects. Despite the availability of various types of 3D LiDAR sensors in the market, MOS research still predominantly focuses on 3D point clouds from mechanically spinning omnidirectional LiDAR sensors. Thus, we are, for example, lacking a dataset with MOS labels for point clouds from solid-state LiDAR sensors which have irregular scanning patterns. In this paper, we present a labeled dataset, called HeLiMOS, that enables to test MOS approaches on four heterogeneous LiDAR sensors, including two solid-state LiDAR sensors. Furthermore, we introduce a novel automatic labeling method to substantially reduce the labeling effort required from human annotators. To this end, our framework exploits an instance-aware static map building approach and tracking-based false label filtering. Finally, we provide experimental results regarding the performance of commonly used state-of-the-art MOS approaches on HeLiMOS that suggest a new direction for a sensor-agnostic MOS, which generally works regardless of the type of LiDAR sensors used to capture 3D point clouds. Our dataset is available at https://sites.google.com/view/helimos.

---

### [GEODE Dataset](https://github.com/PengYu-Team/GEODE_dataset)

- **Year**: 2024
- **Sensor**: 1x Velodyne VLP-16, 1x Ouster OS1-64, 1x Livox Avia, 1x Xsens MTi-30, 1x HikRobot MV-CS050-10GC, 1x CHCNAV CGI610, 1x Vicon Vero 2.2, 1x Leica Nova MS60, 1x Leica RTC360
- **Objective**: Odometry, SLAM
- **Environment**: Degenerate Indoor and Outdoor (Flat Ground, Stairs, Metro Tunnel, Off-road, Inland Waterway, Bridges, Urban Tunnel)
- **System**: Handheld, UGV, Sailboat
- **Publication**: IJRR 
- **Abstract**: The ability to estimate pose and generate maps using 3D LiDAR significantly enhances robotic system autonomy. However, existing open-source datasets lack representation of geometrically degenerate environments, limiting the development and benchmarking of robust LiDAR SLAM algorithms. To address this gap, we introduce GEODE, a comprehensive multi-LiDAR, multi-scenario dataset specifically designed to include real-world geometrically degenerate environments. GEODE comprises64trajectories spanning over 64 kilometers across seven diverse settings with varying degrees of degeneracy. The data was meticulously collected to promote the development of versatile algorithms by incorporating various LiDAR sensors, stereo cameras, IMUs, and diverse motion conditions. We evaluate state-of-the-art SLAM approaches using the GEODE dataset to highlight current limitations in LiDAR SLAM techniques. This extensive dataset will be publicly available at https://github.com/PengYu-Team/GEODE dataset, supporting further advancements in LiDAR-based SLAM.

---

### [HK MEMS Dataset](https://github.com/RuanJY/HK_MEMS_Dataset)

- **Year**: 2024
- **Sensor**: 1x Robosense M1, 1x Ouster OS1-32, 1x RealSense L515, 1x Xsens MTI-30, 1x CUAV V5+ (INS)
- **Objective**: Odometry, SLAM
- **Environment**: Urban (Tunnel, Dynamic Scenarios)
- **System**: Handheld, UGV and Vehicle
- **Publication**: Github (Under Review)
- **Abstract**: This paper presents a multimodular dataset, HK-MEMS, incorporating data from MEMS LiDARs, a camera, GNSS, and Inertial Navigation Systems. To our best knowledge, it is the first dataset to offer automotive-grade MEMS LiDAR data on urban roads for research in Simultaneous Localization and Mapping (SLAM).
This dataset emphasizes extreme environments like degenerate urban tunnels and dynamic scenarios, aiming to enhance the robustness of SLAM systems.
We collect 187 minutes and 75.4 kilometers of data. State-of-the-art SLAM methods are evaluated on this benchmark. The result highlights the challenges in extreme environments and underscores the ongoing need to enhance the robustness of SLAM systems. This dataset serves as a valuable platform for exploring the potential and limitations of MEMS LiDAR, and a challenge to enhance the robustness of SLAM in urban navigation scenarios.
