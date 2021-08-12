# 在类中使用参数 (C++)



**目标：** 使用C++创建并运行带有ROS参数的类。

**教程级别：** 初学者

**时间：** 20分钟

**内容**



- ## 背景

  创建自己的节点时，有时需要添加可以从启动文件设置的参数。

  本教程将向您展示如何在 C++ 类中创建这些参数，以及如何在启动文件中设置它们。



- ## 先决条件

  在之前的教程中，您学习了如何创建工作区和创建包。 您还了解了 ROS 2 系统中的参数及其功能。



- ## 任务

  1. ### 创建一个包

      打开一个新终端并读取和执行ROS2设置文件中的shell命令，以便ros2命令可以工作。

      导航到上一教程中创建的dev_ws目录。

      回想一下，包应该在src目录中创建，而不是在工作空间的根目录中。
      导航到```dev_ws/src```并创建一个新包：
      
      ```bash
      ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp
      ```
      
      你的终端将返回一条消息，验证你的包```cpp_parameters```及其所有必要文件和文件夹的创建。

      ```--dependencies```参数会自动将必要的依赖行添加到```package.xml```和```CMakeLists.txt```。

      1. #### 更新 package.xml
      
          因为你在包创建期间使用了```--dependencies```选项，所以你不必手动将依赖项添加到```package.xml```或```CMakeLists.txt```。

          但是，与往常一样，请确保将描述、维护者电子邮件和姓名以及许可证信息添加到```package.xml```。
          
          ```
          <description>C++ parameter tutorial</description>
          <maintainer email="you@email.com">Your Name</maintainer>
          <license>Apache License 2.0</license>
          ```
          
          
          
  2. ### 编写C++节点

      在```dev_ws/src/cpp_parameters/src```目录中，创建一个名为```cpp_parameters_node.cpp```的新文件并将以下代码粘贴到其中：
      
      ```c++
      #include <rclcpp/rclcpp.hpp>
      #include <chrono>
      #include <string>
      #include <functional>

      using namespace std::chrono_literals;

      class ParametersClass: public rclcpp::Node
      {
        public:
          ParametersClass()
            : Node("parameter_node")
          {
            this->declare_parameter<std::string>("my_parameter", "world");
            timer_ = this->create_wall_timer(
            1000ms, std::bind(&ParametersClass::respond, this));
          }
          void respond()
          {
            this->get_parameter("my_parameter", parameter_string_);
            RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
          }
        private:
          std::string parameter_string_;
          rclcpp::TimerBase::SharedPtr timer_;
      };

      int main(int argc, char** argv)
      {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ParametersClass>());
        rclcpp::shutdown();
        return 0;
      }
      ```
      
     2. #### 审查代码

        顶部的```#include```语句是包依赖项。

        下一段代码创建类和构造函数。 
        这个构造函数的第一行创建了我们的参数。
        我们的参数名为my_parameter并被分配了默认值world。
        接下来，```timer_```被初始化，这会导致响应函数每秒执行一次。
        
        ```c++
        class ParametersClass: public rclcpp::Node
        {
          public:
            ParametersClass()
              : Node("parameter_node")
            {
              this->declare_parameter<std::string>("my_parameter", "world");
              timer_ = this->create_wall_timer(
              1000ms, std::bind(&ParametersClass::respond, this));
            }
        ```
        
        我们的响应函数的第一行从节点获取参数```my_parameter```，并将其存储在```parameter_string_```中。
        ```RCLCPP_INFO```函数确保记录消息。
        
        ```c++
        void respond()
        {
          this->get_parameter("my_parameter", parameter_string_);
          RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
        }
        ```
        
        最后是timer_和parameter_string_的声明
        
        ```
        private:
        std::string parameter_string_;
        rclcpp::TimerBase::SharedPtr timer_;
        ```
        
        遵循我们的```ParametersClass```是我们的主要内容。
        这里ROS2被初始化，```rclcpp::spin```开始处理来自节点的数据。
        
        ```c++
        int main(int argc, char** argv)
        {
          rclcpp::init(argc, argv);
          rclcpp::spin(std::make_shared<ParametersClass>());
          rclcpp::shutdown();
          return 0;
        }
        ```

       2. #### 添加可执行文件

          现在打开CMakeLists.txt文件。 
          在依赖项```find_package(rclcpp REQUIRED)```下方添加以下代码行。
          
          ```
          add_executable(parameter_node src/cpp_parameters_node.cpp)
          ament_target_dependencies(parameter_node rclcpp)

          install(TARGETS
            parameter_node
            DESTINATION lib/${PROJECT_NAME}
          )
          ```
          
          
          
          
  3. ### 构建并运行

      在构建之前，在工作区(dev_ws)的根目录中运行rosdep以检查缺少的依赖项是一个好习惯：

      ```bash
      rosdep install -i --from-path src --rosdistro galactic -y
      ```

      导航回你工作区的根目录dev_ws，并构建你的新包：

      ```bash
      colcon build --packages-select cpp_parameters
      ```

      打开一个新终端，导航到dev_ws，并获取安装文件：

      ```bash
      . install/setup.bash
      ```

      现在运行节点：

      ```bash
      现在运行节点：
      ```

      终端应该每秒返回以下消息：

      ```
      [INFO] [parameter_node]: Hello world
      ```

      现在你可以看到参数的默认值，但你希望能够自己设置它。
      有两种方法可以实现这一点。



      1. #### 通过控制台更改

          这部分将使用你从教程中获得的有关参数的知识并将其应用于你刚刚创建的节点。

          确保节点正在运行：
          
          ```bash
          ros2 run cpp_parameters parameter_node
          ```
          
          打开另一个终端，再次从dev_ws内部获取安装文件，然后输入以下行：
          
          ```bash
          ros2 param list
          ```
          
          在那里你将看到自定义参数```my_parameter```。
          要更改它，只需在控制台中运行以下行：
          
          ```
          ros2 param set /parameter_node my_parameter earth
          ```
          
          如果你成功获得输出Set参数，你就知道一切顺利。
          如果你查看另一个终端，你应该看到输出更改为 ```[INFO] [parameter_node]: Hello earth```
          
          如果你成功获得输出```Set```参数，你就知道一切顺利。
          如果你查看另一个终端，你应该看到输出更改为```[INFO] [parameter_node]: Hello earth```

      2. #### 通过启动文件更改

          你也可以在启动文件中设置参数，
          但首先你需要添加启动目录。
          在dev_ws/src/cpp_parameters/目录中，创建一个名为launch的新目录。
          在那里，创建一个名为cpp_parameters_launch.py的新文件
          
          ```
          from launch import LaunchDescription
          from launch_ros.actions import Node

          def generate_launch_description():
              return LaunchDescription([
                  Node(
                      package="cpp_parameters",
                      executable="parameter_node",
                      name="custom_parameter_node",
                      output="screen",
                      emulate_tty=True,
                      parameters=[
                          {"my_parameter": "earth"}
                      ]
                  )
              ])
          ```
          
          在这里你可以看到我们在启动节点parameter_node时将my_parameter设置为earth。
          通过添加下面的两行，我们确保我们的输出打印在我们的控制台中。
          
          ```
          output="screen",
          emulate_tty=True,
          ```

          现在打开CMakeLists.txt文件。
          在你之前添加的行下方，添加以下代码行。
          
          ```
          install(
            DIRECTORY launch
            DESTINATION share/${PROJECT_NAME}
          )
          ```
          
          打开控制台并导航到工作区的根目录dev_ws，然后构建新包：
          
          ```bash
          colcon build --packages-select cpp_parameters
          ```
          
          然后在新终端中获取安装文件：
          
          ```bash
          . install/setup.bash
          ```
          
          现在使用我们刚刚创建的启动文件运行节点：
          
          ```bash
          ros2 launch cpp_parameters cpp_parameters_launch.py
          ```
          
          终端应该每秒返回以下消息：
          
          ```
          [parameter_node-1] [INFO] [custom_parameter_node]: Hello earth
          ```
          
          
          
          
- ## 概括

  你创建了一个带有自定义参数的节点，该参数可以从启动文件或命令行进行设置。
  你将依赖项、可执行文件和启动文件添加到包配置文件中，以便你可以构建和运行它们，并查看参数的运行情况。




- ## 下一步

  现在你已经有了自己的一些软件包和ROS2系统，下一个教程将向你展示如何在遇到问题时检查你的环境和系统中的问题。
