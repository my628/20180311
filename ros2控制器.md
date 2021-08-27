## 命名法

ros2_control 框架使用命名空间根据控制器使用的命令接口类型对控制器进行排序。

控制器使用通用硬件接口定义。

控制器的命名空间命令以下命令接口类型：


- position_controllers:  ```hardware_interface::HW_IF_POSITION```
- velocity_controller: ```hardware_interface::HW_IF_VELOCITY```
- effort_controllers: ```hardware_interface::HW_IF_EFFORT```
- ...



## 控制器

实现了以下标准控制器：

- 关节轨迹控制器——提供用位置、速度和加速度定义的航路点或目标点列表，控制器通过它插入关节轨迹。

- 列表不完整




## 指南和最佳实践

编写一个新的控制器

## 可用控制器

差速驱动

转发命令

关节轨迹

位置控制器

速度控制器

作用力控制器




## 可用的广播

关节状态广播

Imu传感器广播

