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
1. 若使用LIO,确认LIO已经可以正常运行（我们使用fastlio2/fastlio_sam， 初次运行可以根据报错微调坐标系名字，需要打开lio中将点云发布到世界系）
   有条件可以调试ekf  
   也可以选择最信任的里程计输出发布到base_link坐标系，系统自动完成投影到平面  
2. 安装下述依赖项（其实不用oak可以不装depthai等驱动，可以删除项目oak相关文件后再编译）
3. 编辑`localize/slam/spectacular_ai/main.launch`  
   该文件中目前只启用了一个vio模块，后续可以多模块同时启动  
   1. 首先确保只启用了一个vio模块，其他模块需要注释，然后将参数`mxId`留空，只连接一个oak启动程序
   2. 启动后程序会输出当前oak的mxId，需要复制到对应的模块参数中
4. 根据自己车辆设置以下topic输入:（根据自己队伍的自瞄和上下位机通信修改）
   1. /robot/auto_aim      : 自瞄topic （我们的自瞄输出包含俯仰、航向角度、角速度、目标编号、建议射频等数据）
   2. /robot/chassis_angle : 底盘yaw轴电机绝对位置编码器信号topic（用于装甲板受击感知）
5. 确保自己的车辆已经可以执行如下参数：
   1. /robot/logic_recommend_angle： 自瞄未瞄准时云台建议俯仰角
   2. /robot/spinning_speed： 小陀螺速度 （范围 0-32768，根据自己的车辆情况缩放执行）
   3. /cmd_cel: 模拟操作手的前后左右移动和航向转速（不考虑底盘与云台解耦，就当步兵用）（逻辑节点有时会直接控制运动，越过导航）
   4. /robot/armor_select： 装甲板白名单 0啥也不是， 1-5对应装甲板编号， 6前哨站、7哨兵、8基地，输出为0 不瞄准
   5. /cmd_force_scanning： 真值表示不用跟着导航走了直接原地转就行了
6. 部署完成，通过`startup/launch/start.launch`启动程序

## 运行环境与编译：
   我们的系统为ubuntu20.04
   livox_ros_driver2、fastlio、auto_sentry分别在三个工作空间中，分别编译

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
