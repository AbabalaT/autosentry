# 机甲大师2023赛季南京航空航天大学 哨兵算法部分
## 软件功能介绍
   autosentry包含除了雷达驱动、LIO里程计、自瞄系统之外的哨兵算法
1. key_rc_test:调试时按键控制哨兵移动功能包
2. localize:定位功能包
   1. localize_startup：静态坐标系转换及启动调用
   2. slam/spectacularAI: 运行在OAK相机上的VIO功能包
3. logic_contrl: 逻辑控制节点，各类任务控制，决策类型选择
4. navigation: 导航部分
   1. depth2scan: 视觉深度图转2D激光雷达数据，项目开始时用于避障
   2. move_controller: TEB导航系统
   3. pcl2process: 用于避障的点云地面分割
   4. project2plane: 负责将EKF输出的3维位姿投影到2维上，同时将2D激光数据去除俯仰和横滚带来的偏差
5. oak/oak_imu:测试启用OAK而不用IMU的功能（为后续升级做准备）
6. robot/robot_model: urdf坐标
7. serial/referee: 裁判系统信号解析
8. startup: 启动控制

## 部署
1. 安装下述依赖项
2. 编辑`localize/slam/spectacular_ai/main.launch`  
   该文件中目前只启用了一个vio模块，后续可以多模块同时启动  
   1. 首先确保只启用了一个vio模块，其他模块需要注释，然后将参数`mxId`留空，只连接一个oak启动程序
   2. 启动后程序会输出当前oak的mxId，需要复制到对应的模块参数中
3. 部署完成，通过`startup/launch/start.launch`启动程序

## 依赖项
1. depthAI-core https://gitee.com/oakchina/depthai-core/releases/
2. depthAI-ros https://gitee.com/oakchina/depthai-ros
3. ros-\<version\>-serial
4. ros-\<version\>-robot-localization
5. ros-\<version\>-move-base
6. ros-\<version\>-map-server
7. ros-\<version\>-depthimage-to-laserscan
8. cmake时会自动下载spectacularAI的依赖，如果下载过慢，可以手工下载下面的包，并放置于`localize/slam/spectacular_ai/thirdparty`下
   https://github.com/SpectacularAI/sdk/releases/download/v1.10.0/spectacularAI_depthaiPlugin_cpp_non-commercial_1.10.0.tar.gz

## 坐标系定义
使用标准ROS坐标系 https://www.ros.org/reps/rep-0103.html
