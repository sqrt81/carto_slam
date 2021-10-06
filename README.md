# carto_slam
## 运行环境和程序安装方法
### 环境依赖
1. Ubuntu 16.04及以上
2. ROS（ROS安装可参考http://wiki.ros.org/ROS/Installation）

### 依赖库安装
> sudo apt-get update  
> sudo apt-get install -y clang cmake g++ git google-mock libboost-all-dev libcairo2-dev libcurl4-openssl-dev libeigen3-dev libgflags-dev libgoogle-glog-dev liblua5.2-dev libsuitesparse-dev lsb-release ninja-build stow

### 程序下载
进入ROS的工作空间 `cd /path/to/your/catkin_ws`  
新建两个文件夹 `mkdir src src_carto`  
导入源码  
> git clone https://github.com/sqrt81/carto_slam.git src/carto_slam -b deployment  
> git clone https://github.com/ceres-solver/ceres-solver.git src_carto/ceres-solver -b 1.13.0  
> git clone https://github.com/sqrt81/carto_modified.git src_carto/cartographer  
> git clone https://github.com/sqrt81/carto_ros.git src_carto/cartographer_ros  
> git clone https://github.com/abseil/abseil-cpp.git abseil-cpp  

## 程序编译安装
1. 编译安装abseil
> cd abseil-cpp  
> mkdir build && cd build  
> cmake .. -DCMAKE_CXX_STANDARD=11  
> make -j8  
> sudo make install  
> cd ../..

2. 编译安装protobuf
> ./src_carto/cartographer/scripts/install_proto3.sh

3. 编译安装cartographer
> catkin_make_isolated --source=src_carto --use-ninja

4. 下载地图模型文件
> mkdir src/carto_slam/recorded_bag  
> wget https://cloud.tsinghua.edu.cn/f/563c7a5b03db40c2aa62/?dl=1  
> mv index.html?dl=1 src/carto_slam/recorded_bag/state_gym.pbstream  

5. 修改~/.bashrc文件（如果用bash作为shell的话）
在`source /opt/ros/kinetic/setup.bash`这一行后面加入如下脚本，使得ROS能够定位到cartographer  
> source /path/to/your/catkin_ws/devel_isolated/setup.bash  
> source /path/to/your/catkin_ws/devel/setup.bash  
然后在终端中输入`source ~/.bashrc`。

6. 在ROS中注册carto\_slam包 
> catkin_make

### 程序结构说明
安装完毕后，在catkin_ws目录下的文件结构如下图所示（仅展示部分文件夹）
catkin_ws  
├── src  
│   └── carto_slam  
│       ├── config  
│       ├── launch  
│       ├── recorded_bag  
│       ├── scripts  
│       ├── src  
│       └── urdf  
├── src_carto  
│   ├── ceres-solver  
│   ├── cartographer  
│   └── cartographer_ros  
└── abseil-cpp  
以下对上述文件夹进行说明。

- carto_slam
这是存放设置文件和可执行文件的目录。
- config
此目录下存放配置文件。这些配置文件的参数经过专门调整，对目前的任务有较好的表现。
- launch
此目录下存放可以被执行的程序脚本。这些脚本会启动一系列程序。具体流程会在后续“数据处理流程”中详述。
- recorded_bag
此目录下存放地图模型文件。
- scripts
此目录下存放辅助的python脚本。这些脚本会被launch目录下的程序调用。
- src
此目录下存放两个演示用c++程序的源码。这些程序不会在定位流程中被使用。
- urdf
此目录下存放一个机器人模型文件，其作用是确定激光雷达和车体的相对位置。

- ceres-solver
此文件夹中包含一种非线性优化器。这种优化器被用于cartographer的定位过程。
- cartographer
此文件夹包含cartographer的源码。
- cartographer_ros
此文件夹中包含cartographer与ros交互用到的接口函数。

- abseil-cpp
此文件夹是安装abseil使用的。在完成了abseil的安装（安装步骤1）后，此文件夹可以被删除。

## 程序操作方法
1. 配置激光雷达  
本程序利用32线Velodyne激光雷达定位。在启动程序前，应先设定`src/carto_slam/launch/velodyne.launch`文件中的激光雷达IP地址、端口号、转速。  
为了提高定位精度，应校准激光雷达的实际安装角度，并将校准结果记录在`src/carto_slam/urdf/bot.urdf`文件第29行。

2. 启动  
打开终端，输入 `roslaunch carto_slam pure_local.launch map:=gym`，即可开始定位。此后程序会持续发布位置信息。

3. 重新定位  
在程序刚启动时，或运行途中，如果出现程序无法正确估计当前位置的情况，可能需要重新定位。首先，抵达一个确定的位置和朝向（如有GPS的位置，或预先标定的位置）。然后，调用ROS中的`/relocation`服务（service），在request中填写当前的位置和朝向角（朝向角应以四元数的形式表示，转换关系为：`x=0, y=0, z=sin(theta/2), w=cos(theta/2)`，其中`theta`为朝向角。  
注意，在建图时，程序会产生一个坐标系map。map坐标系和世界坐标系的相对位姿需要标定才能得到。在重新定位时，发送的位置和朝向角是**map坐标系下的**测量结果。

4. 关闭  
在上述终端中按`Ctrl+c`，杀死进程

5. 查看/修改参数  
本程序的参数保存在`src/carto_slam/config/carto_config.lua`文件中。关于其中配置文件的参数的含义会在“算法说明”小节中介绍。


## 数据处理流程
在程序运行过程中，如下ROS结点会被生成（即ROS程序）：  
> /cartographer_node
> /rviz
> /cartographer_occupancy_grid_node
> /fake_imu
> /robot_state_publisher

其中，后三个结点均为辅助用，用于以一个固定频率向第一个结点提供固定不变的数据。这三个结点不需要外界输入任何数据。第二个结点用于可视化，是可选的。

### 数据采集设备
第一个结点是运行定位程序的ROS结点。除了从另外三个结点获取数据以外，还需要从话题/velodyne_points处接收激光雷达的点云数据。一般来说，这个数据通过激光雷达的驱动给出。当运行在线定位程序时，会自动生成激光雷达的驱动结点。激光雷达的驱动结点会按照“程序操作方法”中步骤1里的配置读取激光雷达硬件数据，并转换为ROS消息发布于话题/velodyne_points中。**除此之外，本程序不需要其它数据。**

### 数据处理过程
在cartographer_node受到点云数据后，会调用`SensorBridge::HandlePointCloud2Message`函数（cartographer_ros/sensor_bridge.cc:173），对点云数据做位置变换。
处理过的点云被`GlobalTrajectoryBuilder::AddSensorData`（cartographer/mapping/internal/global_trajectory_builder.cc:52）接收，该函数会将数据传递给自己成员变量local_trajectory_builder_的`LocalTrajectoryBuilder3D::AddRangeData`（cartographer/mapping/internal/3d/local_trajectory_builder_3d.cc:133）函数中。
`LocalTrajectoryBuilder3D::AddRangeData`函数会把收到的数据和现有的地图点云做比较，搜索使得比较误差最小的位姿作为激光雷达的当前位姿。
被估计出的激光雷达位姿会以坐标变换的形式发布在话题/tf中。由于激光雷达和车辆是固连的，所以通过/tf可以获取车辆在地图坐标系中的位姿。从/tf中获取车辆的位姿方法请参考[官方文档](http://wiki.ros.org/tf2/)。在/tf中，地图坐标系的名称为`map`，与车辆固连的坐标系名称为`base_link`。**这是本程序唯一的输出。**  
另外，我们准备了一个名为/relocation的ROS service。此service的作用是重置定位算法的当前位置。如果发现算法跟踪失败，可以调用此service来重新设定车辆的位置和姿态。

### 数据效果展示
可以使用Rviz看到地图、当前车辆位置和点云的匹配情况。默认情况下，Rviz会自动启动。


## 算法说明
本程序的功能是，在特定场合（即重汽园区的室内区域）对运动的、带有激光雷达的车辆进行实时定位。  
本程序采用[Cartographer算法](https://github.com/cartographer-project/cartographer)作为建图和定位的核心算法。该算法使用点云与栅格地图最优匹配的方式确定当前位置。该算法的定位误差在0.2米左右。

### 定位算法流程
1. 初始化。加载栅格地图文件，并将当前位姿`X`设为模型的原点。如果收到ROS的`/relocation`请求，则将当前位姿`X`设为请求中指定的值。
2. 监听点云信息。算法接收激光雷达的点云消息。收到后，对之进行简单的体素滤波处理。
3. 计算匹配函数。对处理过的点云，它与栅格地图的匹配程度可以用一个指标`f`来衡量。当点云经过旋转、平移后，`f`的值会发生变化，所以`f`是当前位姿`X`的函数：`f = f(X)`。实际上，为了提高精度，栅格地图分为高精度地图和低精度地图两种，点云与栅格地图的匹配程度的衡量指标是与两个地图匹配程度的加权和：`f(X) = c_1 * f_1(X) + c_2 * f_2(X)`。
4. 最大化匹配函数。使用一种非凸优化的求解器（Ceres Solver）在当前位姿`X`的邻域内搜索`f`的最大值，并得到`f`取最大时的`X'`：`X' = argmax(f)`。
5. 更新位姿：`X <- X'`

### 参数说明
carto_config.lua

#### options部分
map_frame：地图坐标系的名称  
tracking_frame：需要定位的坐标系的名称，也就是车辆所在坐标系的名称  
published_frame：发布的需要定位的坐标系的名称，一般和tracking_frame保持一致  
provide_odom_frame：是否需要发布“里程计”参考坐标系  
publish_frame_projected_to_2d：是否需要把定位的坐标系限制在二维。我们使用的是三维定位所以不需要  
use_odometry：是否使用里程计信息，我们的程序不需要。  
odom_frame：“里程计”参考坐标系的名称  
use_nav_sat：是否使用GPS。我们这边不需要  
use_landmarks：是否使用地标。这里不需要  
num_laser_scans：单线激光雷达的数目，我们这里不使用单线激光雷达，所以为零。  
num_multi_echo_laser_scans：同上。  
num_subdivisions_per_laser_scan：单线雷达降采样系数，不使用。  
num_point_clouds：多线激光雷达的数目。本工程中使用一个。  
lookup_transform_timeout_sec：查找坐标变换的时限。本工程中坐标变换是通过robot_state_publisher实时发布的，因此此项设为一个小数即可。  
submap_publish_period_sec：发布子图的时间。此项用于建图过程，本工程中不使用。  
pose_publish_period_sec：发布位姿的时间，也就是**程序更新间隔**，数值越小发布越快。但是由于两次激光雷达扫描之间的位姿是通过插值得到的，因此数值过小没有意义，只要小于激光雷达更新间隔即可。  
trajectory_publish_period_sec：发布轨迹的时间间隔。本工程中不使用。  
rangefinder_sampling_ratio：确定位姿时，点云的采样率（1.0表示使用每个数据，0.5表示隔一个数据使用一个，0.0表示完全不使用）  
odometry_sampling_ratio：确定位姿时，里程计的采样率。本工程中不使用。  
fixed_frame_pose_sampling_ratio：确定位姿时，GPS的采样率。本工程中不使用。  
imu_sampling_ratio：确定位姿时，IMU的采样率。本工程中定为1即可。  
landmarks_sampling_ratio：确定位姿时，地标位置的采样率。本工程中不使用。  

#### TRAJECTORY_BUILDER_3D部分
num_accumulated_range_data：累计点云数。把多个点云数据合并起来再定位可以增加准确性。但是在本工程中，由于车辆运行较快，实际上这种手段效果不好。因此此参数设为1即可。  
min_range：激光雷达最近距离，过近的点云数据会被舍弃。  
max_range：激光雷达最远距离，过远的点云数据会被舍弃。  
voxel_filter_size：点云体素滤波的滤波器尺寸。在这个大小的方块内的点在滤波后只会留下一个。  
high_resolution_adaptive_voxel_filter：关于自适应体素滤波器的设置。  
submaps.high_resolution：高分辨率子图的栅格大小。栅格越小定位越精确，但是对干扰也更敏感，定位容易抖动。  
submaps.low_resolution：低分辨率子图的栅格大小。栅格越小定位越精确，但是运动中容易出现漂移。  
use_online_correlative_scan_matching：另外一种匹配算法。此算法非常慢，不足以适应实时需要。  
ceres_scan_matcher.occupied_space_weight_0：高分辨率子图匹配权重。  
ceres_scan_matcher.occupied_space_weight_1：低分辨率子图匹配权重。  
ceres_scan_matcher.translation_weight：匹配中位置的权重。此项越小，算法越倾向于通过调整位置来使点云和栅格匹配。  
ceres_scan_matcher.rotation_weight：匹配中旋转的权重。此项越小，算法越倾向于通过调整角度来使点云和栅格匹配。  
ceres_scan_matcher.only_optimize_yaw：计算旋转时只计算偏航角。  
ceres_scan_matcher.ceres_solver_options：求解器设置。

#### POSE_GRAPH
POSE_GRAPH用于建图。工程中不使用。


### 建图方法
**注意：此小节内容不是定位所必需的**
Cartographer算法是同步定位与建图算法。因此，不借助任何外部工具，仅凭Cartographer，也能根据点云的时序序列对场地建图。  
为了做到这一点，首先要记录点云数据。在确保激光雷达已连接并正确配置的前提下运行`roslaunch carto_slam record_bag.launch`，然后在场地内移动即可。在记录数据完成后，通过`Ctrl+c`终止进程，记录好的数据会保存在`src/carto_slam/recorded_bag`目录下。  
根据记录好的数据，Cartographer可以离线建图。之所以选择离线建图而非在线建图，是为了防止因为算力问题导致点云丢包。将记录好的数据包更名为`test.bag`，然后运行`roslaunch carto_slam demo_slam_bag.launch`。建好的地图会被保存在`src/carto_slam/recorded_bag/state.pbstream`文件中。  
然而，这样建图的效果不一定足够理想。用户可以编辑Cartographer建图得到的地图文件。Cartographer在建图过程中，并非将所有的点云信息融合在同一张图上，而是依据时间顺序，将点云分为多个组，每个组的点云融合为一张子图，再依据约束等，确定子图之间的相对位姿。因此，Cartographer的地图实际上是一系列子图的集合。考虑到数量问题，编辑子图的栅格比较困难，但是手动修改子图的相对位姿是可行的。在`carto_slam/src/`中有两个修改子图的示例程序。

### 具体算法说明
对于算法的详细说明，请参考[cartographer的官方文档](https://google-cartographer.readthedocs.io/en/latest)。
针对本工程的应用场合（重汽园区），本程序中做了如下改进：
1. 使用调整过的地图。
一般的slam程序会在定位的同时建立环境的地图，但是这样得到的地图往往会出现累积误差大、无法回环等问题，降低定位精度。为了克服此问题，我们针对地图做了一些调整，使得地图的累积误差大幅度减小。

2. 优化地图存储格式。
对于尺寸长达一千米的场景，按照原版的Cartographer程序，需要非常巨大的内存空间才能存储下足够精细的地图（以达到10cm的定位精度）。如此巨大的内存空间是不现实的，也是非常不必要的，因为实际上这个区域内的大部分地方都不会被使用到。针对此问题，我们修改了地图存储和扩展的方式，让展开的地图的内存占用被限制在1GB以内。

3. 简化预处理过程。
原版的程序会对数据做严格的检查，如果数据的时间戳过期就会立刻退出。但是ROS系统不是一个实时系统，不同数据通过话题的方式传递时常会出现先后顺序不同的问题。这个问题实际上对定位没有任何影响，因此我们在程序中对时间戳做了一些处理，并删除了一些对时间戳的检查。这使得我们的程序面对数据延迟更加稳定。
