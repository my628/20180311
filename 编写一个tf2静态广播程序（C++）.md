# 编写一个tf2静态广播程序



**描述：** 本教程将教你如何向tf2广播静态坐标系
**教程级别：** 初学者
**下一篇教程：** 编写tf2广播程序（C++）



  在本教程中，我们将编写代码将静态转换发布到tf2上。
  这是一个独立的教程，介绍静态变换的基础知识。
  在接下来的两个教程中，我们将编写代码来重现tf2简介教程中的演示。
  之后，以下教程将重点介绍如何使用更高级的tf2功能扩展演示。
  
  
  
  1. ## 创建一个包

      首先，我们将创建一个用于本教程和以下教程的包。
      这个名为learning_tf2的包将依赖于tf2、tf2_ros、rclcpp。
      
      ```bash
      ros2 pkg create --build-type ament_cmake learning_tf2 tf2 tf2_ros rclcpp
      ```
      
      
      
  2. ## 如何广播变换

      本节将教你如何向tf2广播坐标系。
      让我们首先创建源文件。转到我们刚刚创建的包：
      
      ```bash
      colcon_cd learning_tf2
      ```
      
      1. ### 代码

          启动你喜爱的编辑器，将以下代码粘贴到名为src/static_transform_broadcaster.cpp的新文件中。
          
          ```c++
          #include "rclcpp/rclcpp.hpp"
          #include "tf2/LinearMath/Quaternion.h"
          #include "tf2_ros/static_transform_broadcaster.h"

          int main(int argc, char * argv[])
          {
              rclcpp::init(argc, argv);
              //
              auto node = rclcpp::Node::make_shared("testing_tf2");
              //
              std::shared_ptr<tf2_ros::StaticTransformBroadcaster> stb = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);;
              //
              geometry_msgs::msg::TransformStamped ts;
              //
              ts.header.stamp = node->now();
              ts.header.frame_id = "world";
              ts.child_frame_id = "rigid_body";
              //位置
              ts.transform.translation.x = 0.0;
              ts.transform.translation.y = 0.0;
              ts.transform.translation.z = 1.0;

              tf2::Quaternion q;
              //使用固定轴roll、pitch、yaw设置四元数
              q.setRPY(0.0, 0.0, 0.0);
              //姿态
              ts.transform.rotation.x = q.x();
              ts.transform.rotation.y = q.y();
              ts.transform.rotation.z = q.z();
              ts.transform.rotation.w = q.w();
              //发送一条带有戳记的消息
              //数据结构包括坐标系id、时间和父id。
              stb->sendTransform(ts);

              rclcpp::spin(node);
              rclcpp::shutdown();
              return 0;
          }
          ```
          
          
          
  3. ## 运行静态广播

      现在我们已经创建了代码，让我们先编译它。打开CMakeLists.txt文件并添加以下内容：
      
      ```cmake
      add_executable(broadcaster src/static_transform_broadcaster)
      ament_target_dependencies(broadcaster rclcpp tf2 tf2_ros)
      install(TARGETS broadcaster DESTINATION lib/${PROJECT_NAME})
      ```
      
      并尝试构建并运行你的包
      
      ```bash
      colcon build
      . install/setup.bash
      ros2 run learning_tf2 broadcaster
      ```
      
      
      
      
  4. ## 检查结果
      
      现在我们可以通过回显tf_static主题来检查static_transform转换是否已发布
      
      ```bash
      ros2 topic echo /tf_static
      ```
      
      如果一切顺利，你将看到一个静态转换
      
      ```
      transforms:
        -
          header:
            seq: 0
            stamp:
              secs: 1459282870
              nsecs: 126883440
            frame_id: world
          child_frame_id: mystaticturtle
          transform:
            translation:
              x: 0.0
              y: 0.0
              z: 1.0
            rotation:
              x: 0.0
              y: 0.0
              z: 0.0
              w: 1.0
      ---
      ```
      
      
      
      
      
  5. ## 发布静态转换的正确方法

      本教程旨在展示如何使用StaticTransformBroadcaster发布静态转换。
      在实际的开发过程中，你不必自己编写此代码，并且应该有权使用专用的tf2_ros工具来编写此代码。
      tf2_ros提供了一个名为static_transform_publisher的可执行文件，
      该文件既可用作命令行工具，也可用作可添加到启动文件中的节点。
      ```
      static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
      # 使用以米为单位的x/y/z偏移和以弧度为单位的偏航/俯仰/滚动，将静态坐标变换发布到tf2(偏航是围绕Z旋转，俯仰是围绕Y旋转，滚动是围绕X旋转）。
      ```
      
      ```
      static_transform_publisher x y z qx qy qz qw frame_id child_frame_id
      # 使用以米和四元数为单位的x/y/z偏移将静态坐标变换发布到tf2。
      ```
      
      与tf不同，tf中没有句点参数，并且使用了锁存主题。
      static_transform_publisher被设计为手动使用的命令行工具，也可以在roslaunch文件中用于设置静态转换。例如：
      
      ```
      <launch>
      <node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="1 0 0 0 0 0 1 link1_parent link1" />
      </launch>
      ```
