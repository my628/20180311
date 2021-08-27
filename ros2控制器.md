## 命名法

ros2_control 框架使用命名空间根据控制器使用的命令接口类型对控制器进行排序。

控制器使用通用硬件接口定义。

控制器的命名空间命令以下命令接口类型：


- position_controllers:  ```hardware_interface::HW_IF_POSITION```
- velocity_controller: ```hardware_interface::HW_IF_VELOCITY```
- effort_controllers: ```hardware_interface::HW_IF_EFFORT```
- ...


