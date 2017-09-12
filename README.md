## <center>ROS控制移动底座</center>

## 1 要提前讲解的内容
### 1.1 运动控制分层
《ROS入门实例》31页到34页的运动控制的分层结构，主要针对各层包含的大致的内容和一些开源方案。
### 1.2 ROS中的tf Package
主要涉及以下内容：
&emsp;&emsp;(1) tf树的概念
&emsp;&emsp;(2) 如果广播机器人的坐标系并构造tf树
&emsp;&emsp;(3) 如何监听指定的两个tf树节点。
## 2 依赖项
- ROS Indigo
- ArbotiX Simulator
```shell
sudo apt-get install ros-indigo-arbotix- *
```
或者选择源码的安装方式(__推荐__)
```shell
git clone https://github.com/pirobot/rbx1.git
```
- Rbx1_Control
```shell
git clone https://github.com/gaochq/Ros_Course
```

## 3 实验内容
### 3.1 Twist 消息和轮子转动
Ros使用[Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)消息发布运动指令给基控制器，可以通过下面命令查看Twist消息，
```shell
rosmsg show geometry_msgs/Twist
```
#### 3.1.1 Twist 消息示例
```python
{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}
```
#### 3.1.2 用RViz监控机器人运动
在已经成功安装了ArbotiX的基础上，运行下面命令，启动一个模拟的Turtlebot：
```shell
roslaunch rbx1_bringup fake_turtlebot.launch
```
在RViz中查看模拟机器人
```shell
rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz
```
发布一个Twist消息，使机器人实现顺时针的圆周运动
```shell
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.5}}'
```
“Ctrl-C”停止上一个消息之后，发布一个空的Twist消息，使机器人停止运动
```shell
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
```
在rbx1中一个topic的作用周期是3s，所以可以发布一条消息包含模拟机器人两种不同的运动。发布下面的消息，使模拟机器人前行3s之后循环做逆时针的圆周运动。
```shell
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0,z: 0}, angular: {x: 0, y: 0, z: 0}}'; rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'
```
发布空消息，停止机器人运动，关闭窗口。
### 3.2 从ROS节点发布Twist消息
首先启动模拟机器人和RViz监视窗口
```shell
roslaunch rbx1_bringup fake_turtlebot.launch
rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz
```
然后启动定时运动节点
```shell
rosrun rbx1_control timed_out_and_back
```
### 3.3 使用测程法前进并返回
首先启动模拟机器人和RViz监视窗口
```shell
roslaunch rbx1_bringup fake_turtlebot.launch
rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz
```
然后启动测量反馈运动节点
```shell
rosrun rbx1_control odom_out_and_back
```
### 3.4 使用测量走正方形
首先启动模拟机器人和RViz监视窗口
```shell
roslaunch rbx1_bringup fake_turtlebot.launch
rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz
```
然后启动走正方形节点
```shell
rosrun rbx1_control Move_square
```

### 3.5 键盘控制底座运动
首先启动模拟机器人和RViz监视窗口
```shell
roslaunch rbx1_bringup fake_turtlebot.launch
rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz
```
然后启动键盘控制节点
```shell
rosrun rbx1_control Keyboard_Control
```
将光标停留在遥控终端窗口，输入`wsad`，机器人会按照对应的方向移动，`shift`按键会加速移动。       
__注:键盘控制节点代码出自[古-月](http://blog.csdn.net/hcx25909/article/details/9004617)的CSDN博客。__

