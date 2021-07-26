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
      
          因为你在包创建期间使用了```--dependencies```选项，所以你不必手动将依赖项添加到```package.xml```或```CMakeLists.txt```。
          但是，与往常一样，请确保将描述、维护者电子邮件和姓名以及许可证信息添加到```package.xml```。
          
          ```
          <description>C++ client server tutorial</description>
          <maintainer email="you@email.com">Your Name</maintainer>
          <license>Apache License 2.0</license>
          ```



  2. ### 编写服务节点
  
      在```dev_ws/src/cpp_srvcli/src```目录中，创建一个名为```add_two_ints_server.cpp```的新文件并将以下代码粘贴到其中：

      ```c++
      #include "rclcpp/rclcpp.hpp"
      #include "example_interfaces/srv/add_two_ints.hpp"

      #include <memory>

      void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
      {
        response->sum = request->a + request->b;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                      request->a, request->b);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
      }

      int main(int argc, char **argv)
      {
        rclcpp::init(argc, argv);

        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
          node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

        rclcpp::spin(node);
        rclcpp::shutdown();
      }
      ```
      
      1. #### 审查代码
      
          前两个#include语句是你的包依赖项。

          add函数将请求中的两个整数相加，并将总和提供给响应，同时使用日志通知控制台其状态。
          
          ```c++
          void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                   std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
          {
              response->sum = request->a + request->b;
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                  request->a, request->b);
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
          }
          ```
          
          ```main```函数逐行实现以下工作：
          
              - 初始化 ROS 2 C++ 客户端库：
              
                  ```c++
                  rclcpp::init(argc, argv);
                  ```
              
              - 创建一个名为```add_two_ints_server```的节点：

                  ```c++
                  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");
                  ```
                  
              - 为该节点创建一个名为add_two_ints的服务，并使用&add方法在网络上自动通告它：

                  ```c++
                  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service = node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);
                  ```
                  
              - 准备好时打印日志消息：

                  ```c++
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
                  ```
                  
              - 旋转节点，使服务可用。

                  ```c++
                  rclcpp::spin(node);
                  ```
                  
                  
      2. #### 添加可执行文件
      
          ```add_executable```宏生成一个可执行文件，你可以使用```ros2 run```运行。
          将以下代码块添加到CMakeLists.txt以创建名为server的可执行文件：
          
          ```
          add_executable(server src/add_two_ints_server.cpp)
          ament_target_dependencies(server
          rclcpp example_interfaces)
          ```
          
          所以```ros2 run```可以找到可执行文件，将以下几行添加到文件末尾，就在```ament_package()```之前：
          
          ```
          install(TARGETS
          server
          DESTINATION lib/${PROJECT_NAME})
          ```
          
          你现在可以构建你的包，获取本地安装文件并运行它，但让我们先创建客户端节点，以便你可以看到完整的系统在工作。
          
