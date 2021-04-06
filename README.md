# carto_slam
## 程序安装方法
1. 环境依赖：Ubuntu 16.04及以上，已安装ROS（ROS安装可参考http://wiki.ros.org/ROS/Installation）。
2. 进入ROS的工作空间 `cd /path/to/your/catkin_ws`
3. 新建两个文件夹 `mkdir src src_carto`
4. 导入源码  
> git clone https://github.com/sqrt81/carto_slam.git src/carto_slam -b deployment  
> git clone https://github.com/ceres-solver/ceres-solver.git src_carto/ceres-solver.git -b 1.13.0  
> git clone https://github.com/sqrt81/carto_modified.git src_carto/cartographer  
> git clone https://github.com/sqrt81/carto_ros.git src_carto/cartographer_ros  

5. 安装`src_carto`目录下的cartographer

5.1. 安装依赖项  
> sudo apt-get update  
> sudo apt-get install -y clang cmake g++ git google-mock libboost-all-dev libcairo2-dev libcurl4-openssl-dev libeigen3-dev libgflags-dev libgoogle-glog-dev liblua5.2-dev libsuitesparse-dev lsb-release ninja-build stow

5.2. 编译安装abseil
> git clone https://github.com/abseil/abseil-cpp.git  
> mkdir build  
> cd build  
> cmake .. -DCMAKE_CXX_STANDARD=11  
> make -j8  
> sudo make install  

5.3. 编译安装protobuf
> ./src_carto/cartographer/scripts/install_proto3.sh

5.4. 编译安装cartographer
> catkin_make_isolated --source=src_carto --use-ninja

6. 下载地图模型文件
> mkdir src/carto_slam/recorded_bag  
> wget https://cloud.tsinghua.edu.cn/f/563c7a5b03db40c2aa62/?dl=1  
> mv index.html?dl=1 src/carto_slam/recorded_bag/state_gym.pbstream  

7. 修改~/.bashrc文件（如果用bash作为shell的话），在`source /opt/ros/kinetic/setup.bash`这一行后面加入如下脚本，使得ROS能够定位到cartographer  
> source /path/to/your/catkin_ws/devel_isolated/setup.bash  
> source /path/to/your/catkin_ws/devel/setup.bash  
然后在终端中输入`source ~/.bashrc`。
8. 在ROS中注册carto\_slam包 `catkin_make`


## 程序使用方法
1. 配置激光雷达  
本程序利用32线Velodyne激光雷达定位。在启动程序前，应先设定`src/carto_slam/launch/velodyne.launch`文件中的激光雷达IP地址、端口号、转速。  
为了提高定位精度，应校准激光雷达的实际安装角度，并将校准结果记录在`src/carto_slam/urdf/bot.urdf`文件第29行。

2. 启动  
打开终端，输入 `roslaunch carto_slam pure_local.launch map:=gym`

3. 重新定位  
在程序刚启动时，或运行途中，如果出现程序无法正确估计当前位置的情况，可能需要重新定位。首先，抵达一个确定的位置和朝向（如有GPS的位置，或预先标定的位置）。然后，调用ROS中的`/relocation`服务（service），在request中填写当前的位置和朝向角（朝向角应以四元数的形式表示，转换关系为：`x=0, y=0, z=sin(theta/2), w=cos(theta/2)`，其中`theta`为朝向角。  
注意，在建图时，程序会产生一个坐标系map。map坐标系和世界坐标系的相对位姿需要标定才能得到。在重新定位时，发送的位置和朝向角是**map坐标系下的**测量结果。

4. 关闭  
在上述终端中按`Ctrl+c`，杀死进程

5. 查看/修改参数  
本程序的参数保存在`src/carto_slam/config/carto_config.lua`文件中。


## 算法说明
本程序采用[Cartographer算法](https://github.com/cartographer-project/cartographer)作为建图和定位的核心算法。该算法使用点云与栅格地图最优匹配的方式确定当前位置。该算法的定位误差在0.2米左右。

该算法的流程如下：  
1. 初始化。加载栅格地图文件，并将当前位姿`X`设为模型的原点。如果收到ROS的`/relocation`请求，则将当前位姿`X`设为请求中指定的值。
2. 监听点云信息。算法接收激光雷达的点云消息。收到后，对之进行简单的体素滤波处理。
3. 计算匹配函数。对处理过的点云，它与栅格地图的匹配程度可以用一个指标`f`来衡量。当点云经过旋转、平移后，`f`的值会发生变化，所以`f`是当前位姿`X`的函数：`f = f(X)`。
4. 最大化匹配函数。使用一种非凸优化的求解器（Ceres Solver）在当前位姿`X`的邻域内搜索`f`的最大值，并得到`f`取最大时的`X'`：`X' = argmax(f)`。
5. 更新位姿：`X <- X'`


## 建图方法
Cartographer算法是同步定位与建图算法。因此，不借助任何外部工具，仅凭Cartographer，也能根据点云的时序序列对场地建图。  
为了做到这一点，首先要记录点云数据。在确保激光雷达已连接并正确配置的前提下运行`roslaunch carto_slam record_bag.launch`，然后在场地内移动即可。在记录数据完成后，通过`Ctrl+c`终止进程，记录好的数据会保存在`src/carto_slam/recorded_bag`目录下。  
根据记录好的数据，Cartographer可以离线建图。之所以选择离线建图而非在线建图，是为了防止因为算力问题导致点云丢包。将记录好的数据包更名为`test.bag`，然后运行`roslaunch carto_slam demo_slam_bag.launch`。建好的地图会被保存在`src/carto_slam/recorded_bag/state.pbstream`文件中。

然而，这样建图的效果不一定足够理想。用户可以编辑Cartographer建图得到的地图文件。Cartographer在建图过程中，并非将所有的点云信息融合在同一张图上，而是依据时间顺序，将点云分为多个组，每个组的点云融合为一张子图，再依据约束等，确定子图之间的相对位姿。因此，Cartographer的地图实际上是一系列子图的集合。考虑到数量问题，编辑子图的栅格比较困难，但是手动修改子图的相对位姿是可行的。在`carto_slam/src/`中有两个修改子图的示例程序。
