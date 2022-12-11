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
