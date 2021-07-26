# 编写一个简单的服务和客户端（C++）



**目标：** 使用 C++ 创建和运行服务和客户端节点。

**教程级别：** 初学者

**时间：** 20分钟

**内容**



- ## 背景

  当节点使用服务进行通信时，发送数据请求的节点称为客户端节点，响应请求的节点称为服务节点。
  
  请求和响应的结构由.srv文件确定。

  这里使用的例子是一个简单的整数加法系统；
  一个节点请求两个整数的和，另一个节点以结果作为响应。



- ## 先决条件

  在之前的教程中，你学习了如何创建工作区和创建包。



- ## 任务

  1. ### 创建一个包

      打开一个新终端并[获取ROS2安装源]，以便ros2命令可以工作。

      导航到上一教程中创建的dev_ws目录。

      回想一下，包应该在src目录中创建，而不是在工作空间的根目录中。
      导航到 dev_ws/src并创建一个新包：
      
      ```bash
      ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces
      ```
      
      你的终端将返回一条消息，验证你的包cpp_srvcli及其所有必要文件和文件夹的创建。

      --dependencies参数会自动将必要的依赖行添加到package.xml和CMakeLists.txt。
      example_interfaces是包含.srv文件的包，你需要构建请求和响应：
      
      ```
      int64 a
      int64 b
      ---
      int64 sum
      ```
      
      前两行是请求的参数，破折号下面是响应。
      
      1. #### 更新package.xml 
      因为您在包创建期间使用了 --dependencies 选项，所以您不必手动将依赖项添加到 package.xml 或 CMakeLists.txt。 但是，与往常一样，请确保将描述、维护者电子邮件和姓名以及许可证信息添加到 package.xml。
      
