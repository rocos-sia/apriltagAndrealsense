## Apriltag + Realsense + 可视化

该功能包目前基于Apriltag码和Realsense的摄像头，无法普遍适用于其他摄像头。

但是可以使用ROS下image_transport中的CameraSubscriber类节点同步发送OPencv得到的图像和相机内参，畸变等参数，直接替代realsense的ROS包，详细的两个包之间的话题见内部README.md或者视觉知识整理.

目前已测试了同步接收图像和T，R数据进行二次处理得到想要的可视化界面，证明上述方法的可行性。

机械臂和码结合的代码节点为apriltag.py，大家可结合实际所用机械臂自行更改。

image_receive为单纯接收图像节点，大家可基于进行更改和开发。

imageAndPose为二次开发的可视化节点，主要增加码的图像坐标轴的可视化，非必须。

## 软件包介绍

本项目包含以下软件包(仍在开发中)：

| 软件包名称  | 内容  |
|    ----    | -------  |
| apriltag_ros | Apriltag码识别源码 |
| realsense2_camera      | realsense的ROS的启动和传参 |
| apriltag | Apriltag字典和相关头文件 |



## 环境

- Ubuntu 20.04
- ROS noetic
- Realsense D435

## 话题解析
https://blog.csdn.net/qq_45494729/article/details/127133349?spm=1001.2014.3001.5502

