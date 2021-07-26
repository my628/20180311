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
          



  3. ### 编写客户端节点

      在```dev_ws/src/cpp_srvcli/src```目录中，创建一个名为```add_two_ints_client.cpp```的新文件并将以下代码粘贴到其中：

      ```c++
      #include "rclcpp/rclcpp.hpp"
      #include "example_interfaces/srv/add_two_ints.hpp"

      #include <chrono>
      #include <cstdlib>
      #include <memory>

      using namespace std::chrono_literals;

      int main(int argc, char **argv)
      {
        rclcpp::init(argc, argv);

        if (argc != 3) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
            return 1;
        }

        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
          node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = atoll(argv[1]);
        request->b = atoll(argv[2]);

        while (!client->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
          }
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) ==
          rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
        }

        rclcpp::shutdown();
        return 0;
      }
      ```
    
    
        1. #### 审查代码

        与服务节点类似，以下代码行创建节点，然后为该节点创建客户端：
        
        ```c++
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        ```
        
        接下来，创建请求。 
        它的结构由前面提到的```.srv```文件定义。
        
        ```c++
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = atoll(argv[1]);
        request->b = atoll(argv[2]);
        ```
        
        ```while```循环给了客户端1秒钟的时间来搜索网络中的服务节点。
        如果找不到，它将继续等待。
        
        ```c++
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        ```
        
        如果客户端被取消（例如，通过在终端中输入Ctrl+C），它将返回一条错误日志消息，说明它被中断了。
        
        ```c++
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
        ```
        
        然后客户端发送它的请求，节点旋转直到它收到它的响应，或者失败。



        2. #### 添加可执行文件

        返回CMakeLists.txt 以添加新节点的可执行文件和目标。 
        从自动生成的文件中删除一些不必要的样板后，你的CMakeLists.txt应如下所示：
        
        ```
        cmake_minimum_required(VERSION 3.5)
        project(cpp_srvcli)

        find_package(ament_cmake REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(example_interfaces REQUIRED)

        add_executable(server src/add_two_ints_server.cpp)
        ament_target_dependencies(server
          rclcpp example_interfaces)

        add_executable(client src/add_two_ints_client.cpp)
        ament_target_dependencies(client
          rclcpp example_interfaces)

        install(TARGETS
          server
          client
          DESTINATION lib/${PROJECT_NAME})

        ament_package()
        ```
        


  4. ### 构建和运行

      在构建之前，在工作区(dev_ws)的根目录中运行rosdep以检查缺少的依赖项是一种很好的做法：
      
      ```bash
      rosdep install -i --from-path src --rosdistro galactic -y
      ```
      
      导航回你工作区的根目录dev_ws，并构建你的新包：
      
      ```bash
      colcon build --packages-select cpp_srvcli
      ```
      
      打开一个新终端，导航到dev_ws，读取并执行设置文件中的shell命令：
      
      ```bash
      . install/setup.bash
      ```
      
      现在运行服务节点：
      
      ```bash
      ros2 run cpp_srvcli server
      ```
      
      终端应返回以下消息，然后等待：
      
      ```
      [INFO] [rclcpp]: Ready to add two ints.
      ```
      
      打开另一个终端，再次从 dev_ws 内部获取安装文件。 启动客户端节点，后跟任意两个以空格分隔的整数：
      
      ```bash
      ros2 run cpp_srvcli client 2 3
      ```
      
      例如，如果你选择2和3，客户端将收到如下响应：
      
      ```
      [INFO] [rclcpp]: Sum: 5
      ```
      
      返回到运行服务节点的终端。
      你将看到它在收到请求和收到的数据以及发回的响应时发布日志消息：
      
      ```
      [INFO] [rclcpp]: Incoming request
      a: 2 b: 3
      [INFO] [rclcpp]: sending back response: [5]
      ```
      
      在服务器终端中输入```Ctrl+C```以停止节点旋转。
      
      
      
      
- ## 总结

  你创建了两个节点来通过服务请求和响应数据。
  你将它们的依赖项和可执行文件添加到包配置文件中，以便你可以构建和运行它们，并查看工作中的服务/客户端系统。

- ## 下一步

  在过去的几个教程中，你一直在使用接口来跨主题和服务传递数据。 
  接下来，你将学习如何创建自定义界面。

- ## 相关内容

  有几种方法可以用C++编写服务和客户端； 查看ros2/examples存储库中的minimum_service和minimum_client 包。
