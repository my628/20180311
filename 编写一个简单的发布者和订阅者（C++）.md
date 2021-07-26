# 编写一个简单的发布者和订阅者 (C++)



**目标：** 使用 C++ 创建并运行发布者和订阅者节点。

**教程级别：** 初学者

**时间：** 20分钟

**内容**



- ## 背景

  节点是通过ROS图进行通信的可执行进程。
  在本教程中，节点将通过主题以字符串消息的形式相互传递信息。
  这里使用的例子是一个简单的“talker”和“listener”系统；一个节点发布数据，另一个节点订阅主题，以便它可以接收该数据。

  可以在此处找到这些示例中使用的代码。



- ## 先决条件

  在之前的教程中，你学习了如何创建工作区和创建包。



- ## 任务

  1. ### 创建一个包

      打开一个新终端并获取ROS2安装源，以便ros2命令可以工作。

      导航到上一教程中创建的dev_ws 目录。

      回想一下，包应该在src目录中创建，而不是在工作空间的根目录中。 
      因此，导航到dev_ws/src，并运行包创建命令：
      
      ```bash
      ros2 pkg create --build-type ament_cmake cpp_pubsub
      ```
      
      你的终端将返回一条消息，验证你的包```cpp_pubsub```及其所有必要文件和文件夹的创建。

      导航到dev_ws/src/cpp_pubsub/src。 
      回想一下，这是包含可执行文件的源文件所属的任何CMake包中的目录。



  2. ### 编写发布者节点
      
      通过输入以下命令下载示例talker代码：
      
      ```bash
      wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_publisher/member_function.cpp
      ```
      
      现在会有一个名为publisher_member_function.cpp的新文件。
      使用你喜欢的文本编辑器打开文件。
      
      ```c++
      #include <chrono>
      #include <functional>
      #include <memory>
      #include <string>

      #include "rclcpp/rclcpp.hpp"
      #include "std_msgs/msg/string.hpp"

      using namespace std::chrono_literals;

      /* This example creates a subclass of Node and uses std::bind() to register a
      * member function as a callback from the timer. */

      class MinimalPublisher : public rclcpp::Node
      {
        public:
          MinimalPublisher()
          : Node("minimal_publisher"), count_(0)
          {
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
          }

        private:
          void timer_callback()
          {
            auto message = std_msgs::msg::String();
            message.data = "Hello, world! " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
          }
          rclcpp::TimerBase::SharedPtr timer_;
          rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
          size_t count_;
      };

      int main(int argc, char * argv[])
      {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<MinimalPublisher>());
        rclcpp::shutdown();
        return 0;
      }
      ```
      
      
      
      
      1. #### 检查代码

          代码的顶部包括你将使用的标准C++头文件。
          在标准C++ 头文件之后是rclcpp/rclcpp.hpp包含，它允许你使用ROS2系统中最常见的部分。
          最后是std_msgs/msg/string.hpp，其中包括你将用于发布数据的内置消息类型。

          这些线代表节点的依赖关系。
          回想一下，必须将依赖项添加到package.xml和CMakeLists.txt，你将在下一节中执行这些操作。
          
          ```c++
          #include <chrono>
          #include <functional>
          #include <memory>
          #include <string>

          #include "rclcpp/rclcpp.hpp"
          #include "std_msgs/msg/string.hpp"

          using namespace std::chrono_literals;
          ```
          
          下一行通过从rclcpp::Node继承来创建节点类MinimalPublisher。
          代码中的每一个this都是指节点。
          
          ```c++
          class MinimalPublisher : public rclcpp::Node
          ```
          
          公共构造函数将节点命名为minimum_publisher并将count_初始化为0。
          在构造函数内部，发布者使用String消息类型、主题名称主题和所需的队列大小进行初始化，以在发生备份时限制消息。 
          接下来， timer_被初始化，这导致timer_callback函数每秒执行两次。
          
          ```c++
          public:
          MinimalPublisher()
          : Node("minimal_publisher"), count_(0)
          {
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
          }
          ```
          
          timer_callback函数是设置消息数据和实际发布消息的地方。
          RCLCPP_INFO宏确保每个发布的消息都打印到控制台。
          
          ```c++
          private:
          void timer_callback()
          {
            auto message = std_msgs::msg::String();
            message.data = "Hello, world! " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
          }
          ```
          
          最后是计时器、发布者和计数器字段的声明。
          
          ```c++
          rclcpp::TimerBase::SharedPtr timer_;
          rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
          size_t count_;
          ```
          
          在MinimalPublisher类之后是main，节点实际执行的地方。
          rclcpp::init初始化ROS2，rclcpp::spin开始处理来自节点的数据，包括来自定时器的回调。
          
          ```c++
          int main(int argc, char * argv[])
          {
            rclcpp::init(argc, argv);
            rclcpp::spin(std::make_shared<MinimalPublisher>());
            rclcpp::shutdown();
            return 0;
          }
          ```
          
          
          
      2. #### 添加依赖

          导航一级回到dev_ws/src/cpp_pubsub目录，其中已为你创建CMakeLists.txt和package.xml 文件。

          使用文本编辑器打开package.xml。

          如上一个教程中所述，确保填写```<description> <maintainer> <license>```标签：
  
          ```
          <description>Examples of minimal publisher/subscriber using rclcpp</description>
          <maintainer email="you@email.com">Your Name</maintainer>
          <license>Apache License 2.0</license>
          ```
          
          在```ament_cmake buildtool```依赖项之后添加新行，并粘贴与你节点的include语句对应的以下依赖项：
          
          ```
          <depend>rclcpp</depend>
          <depend>std_msgs</depend>
          ```
          
          这声明包在其代码执行时需要 rclcpp 和 std_msgs。

          确保保存文件。

      3. #### CMakeLists.txt

          现在打开CMakeLists.txt文件。 
          在现有依赖项```find_package(ament_cmake REQUIRED)```下方，添加以下行：
          
          ```
          find_package(rclcpp REQUIRED)
          find_package(std_msgs REQUIRED)
          ```
          
          之后，添加可执行文件并将其命名为talker，以便你可以使用```ros2 run```运行你的节点：
          
          ```
          add_executable(talker src/publisher_member_function.cpp)
          ament_target_dependencies(talker rclcpp std_msgs)
          ```
          
          最后，添加```install(TARGETS...)```部分，以便```ros2 run```可以找到你的可执行文件：
          
          ```
          install(TARGETS
          talker
          DESTINATION lib/${PROJECT_NAME})
          ```
          
          你可以通过删除一些不必要的部分和注释来清理你的CMakeLists.txt，所以它看起来像这样：
          
          ```
          cmake_minimum_required(VERSION 3.5)
          project(cpp_pubsub)

          Default to C++14
          if(NOT CMAKE_CXX_STANDARD)
            set(CMAKE_CXX_STANDARD 14)
          endif()

          if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            add_compile_options(-Wall -Wextra -Wpedantic)
          endif()

          find_package(ament_cmake REQUIRED)
          find_package(rclcpp REQUIRED)
          find_package(std_msgs REQUIRED)

          add_executable(talker src/publisher_member_function.cpp)
          ament_target_dependencies(talker rclcpp std_msgs)

          install(TARGETS
            talker
            DESTINATION lib/${PROJECT_NAME})

          ament_package()
          ```
          
          你现在可以构建你的包，获取本地安装文件并运行它，但让我们先创建订阅者节点，以便你可以看到完整的系统在工作。
          
          

  3. ### 编写订阅者节点

      返回dev_ws/src/cpp_pubsub/src创建下一个节点。
      在终端中输入以下代码：
      
      ```bash
      wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_subscriber/member_function.cpp
      ```
      
      在控制台中输入ls现在将返回：
      
      ```
      publisher_member_function.cpp  subscriber_member_function.cpp
      ```
      
      使用文本编辑器打开subscriber_member_function.cpp。
      
      ```c++
      #include <memory>

      #include "rclcpp/rclcpp.hpp"
      #include "std_msgs/msg/string.hpp"
      using std::placeholders::_1;

      class MinimalSubscriber : public rclcpp::Node
      {
        public:
          MinimalSubscriber()
          : Node("minimal_subscriber")
          {
            subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
          }

        private:
          void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
          {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
          }
          rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
      };

      int main(int argc, char * argv[])
      {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<MinimalSubscriber>());
        rclcpp::shutdown();
        return 0;
      }
      ```
      
      
      
      1. #### 审查代码

          订阅者节点的代码与发布者的代码几乎相同。
          现在节点被命名为minimum_subscriber，构造函数使用节点的create_subscription类来执行回调。

          没有计时器，因为只要数据发布到主题，订阅者就会做出响应。
          
          ```c++
          public:
          MinimalSubscriber()
          : Node("minimal_subscriber")
          {
            subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
          }
          ```
          
          回忆一下主题教程，发布者和订阅者使用的主题名称和消息类型必须匹配才能进行通信。

          topic_callback函数接收通过主题发布的字符串消息数据，并使用RCLCPP_INFO宏简单地将其写入控制台。

          此类中唯一的字段声明是订阅。
          
          ```c++
          private:
          void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
          {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
          }
          rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
          ```
          
          主要功能完全相同，除了现在它旋转MinimalSubscriber节点。 
          对于发布者节点，旋转意味着启动计时器，但对于订阅者来说，它只是意味着准备在消息到来时接收它们。

          由于此节点与发布者节点具有相同的依赖关系，因此无需向package.xml添加任何新内容。
          
          
          
        2. #### CMakeLists.txt

            重新打开CMakeLists.txt并在发布者条目下方添加订阅者节点的可执行文件和目标。
            
            ```
            add_executable(listener src/subscriber_member_function.cpp)
            ament_target_dependencies(listener rclcpp std_msgs)

            install(TARGETS
              talker
              listener
              DESTINATION lib/${PROJECT_NAME})
            ```
            
            确保保存文件，然后你的发布/订阅系统应该可以使用了。
            
            
            
 4. ### 构建和运行

    你可能已经将rclcpp和std_msgs软件包安装为ROS2系统的一部分。
    在构建之前，在工作区(dev_ws)的根目录中运行rosdep以检查缺少的依赖项是一种很好的做法：
    
    ```bash
    rosdep install -i --from-path src --rosdistro galactic -y
    ```
    
    仍然在你工作区的根目录dev_ws中，构建你的新包：
    
    ```bash
    colcon build --packages-select cpp_pubsub
    ```
    
    打开一个新终端，导航到dev_ws，读取并执行设置文件中的shell命令：
    
    ```bash
    . install/setup.bash
    ```
    
    终端应该每0.5秒开始发布信息消息，如下所示：
    
    ```
    [INFO] [minimal_publisher]: Publishing: "Hello World: 0"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 1"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 2"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 3"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 4"
    ```
    
    打开另一个终端，再次从dev_ws内部读取并执行设置文件中的shell命令，然后启动listener节点：
    
    ```bash
    ros2 run cpp_pubsub listener
    ```
    
    侦听器将开始向控制台打印消息，从发布者当时处于的任何消息计数开始，如下所示：
    
    ```
    [INFO] [minimal_subscriber]: I heard: "Hello World: 10"
    [INFO] [minimal_subscriber]: I heard: "Hello World: 11"
    [INFO] [minimal_subscriber]: I heard: "Hello World: 12"
    [INFO] [minimal_subscriber]: I heard: "Hello World: 13"
    [INFO] [minimal_subscriber]: I heard: "Hello World: 14"
    ```
    
    在每个终端中输入```Ctrl+C```以停止节点旋转。
    
   
   
- ## 总结

  你创建了两个节点来发布和订阅主题上的数据。
  在编译和运行它们之前，你将它们的依赖项和可执行文件添加到包配置文件中。



- ## 下一步

  接下来，你将使用服务/客户端模型创建另一个简单的ROS2包。
  同样，你可以选择用C++或Python编写它。
  

- ## 相关内容

  有几种方法可以用C++编写发布者和订阅者；
  查看ros2/examples存储库中的minimum_publisher和minimum_subscriber包。
