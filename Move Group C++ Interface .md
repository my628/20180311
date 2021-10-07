![image alt](http://moveit2_tutorials.picknik.ai/_images/move_group_interface_tutorial_start_screen.png)

在 MoveIt 中，最简单的用户界面是通过 MoveGroupInterface 类。 它为用户可能想要执行的大多数操作提供了易于使用的功能，特别是设置关节或姿势目标、创建运动计划、移动机器人、将对象添加到环境中以及从机器人附加/分离对象。 该接口通过 ROS 主题、服务和操作与 MoveGroup 节点进行通信。

观看此快速 YouTube 视频演示，了解移动组界面的强大功能！

入门
如果您还没有这样做，请确保您已完成入门中的步骤。

注意：由于 MoveitVisualTools 尚未移植到 ROS2，因此本教程使用了 xterm 和一个简单的提示器来帮助用户完成每个演示步骤。 要安装 xterm，请运行以下命令：
