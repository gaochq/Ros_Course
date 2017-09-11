## ROS控制移动底座
&emsp;&emsp;本代码是[rbx1](https://github.com/pirobot/rbx1)部分节点的C++版本。

## 1 依赖项
- ROS Indigo
- ArbotiX Simulator
```shell
sudo apt-get install ros-indigo-arbotix- *
```
- Rbx1_Control
```shell
git clone https://github.com/pirobot/rbx1.git 
```

## 2 实验内容
### 2.1 Twist 消息和轮子转动
&emsp;&emsp;Ros使用[Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)消息发布运动指令给基控制器，可以通过下面命令查看Twist消息，
```shell
rosmsg show geometry_msgs/Twist
```
#### 2.1.1 Twist 消息示例
```python
{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}
```
#### 2.1.2 用RViz监控机器人运动
&emsp;&emsp;在已经成功安装了ArbotiX的基础上，运行下面命令，启动一个模拟的Turtlebot：
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
&emsp;&emsp;在rbx1中一个topic的作用周期是3s，所以可以发布一条消息包含模拟机器人两种不同的运动。发布下面的消息，使模拟机器人前行3s之后循环做逆时针的圆周运动。
```shell
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0,z: 0}, angular: {x: 0, y: 0, z: 0}}'; rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'
```
发布空消息，停止机器人运动，关闭窗口。
### 2.1 从ROS节点发布Twist消息
&emsp;&emsp;首先启动模拟机器人和RViz监视窗口
```shell
roslaunch rbx1_bringup fake_turtlebot.launch
rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz
```
然后启动定时运动节点
```shell
rosrun rbx1_control timed_out_and_back
```
### 2.2 使用测程法前进并返回
&emsp;&emsp;首先启动模拟机器人和RViz监视窗口
```shell
roslaunch rbx1_bringup fake_turtlebot.launch
rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz
```
然后启动定时运动节点
```shell
rosrun rbx1_control odom_out_and_back
```
### 2.3 使用测量走正方形
&emsp;&emsp;首先启动模拟机器人和RViz监视窗口
```shell
roslaunch rbx1_bringup fake_turtlebot.launch
rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz
```
然后启动定时运动节点
```shell
rosrun rbx1_control Move_square
```

### 2.3 使用测量走正方形
&emsp;&emsp;首先启动模拟机器人和RViz监视窗口
```shell
roslaunch rbx1_bringup fake_turtlebot.launch
rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz
```
然后启动定时运动节点
```shell
rosrun rbx1_control Keyboard_Control
```
将光标停留在遥控终端窗口，输入`wsad`，机器人会按照对应的方向移动，`shift`按键会加速移动。       
__注:键盘控制节点代码使用[古-月](http://blog.csdn.net/hcx25909/article/details/9004617)的CSDN博客。__

