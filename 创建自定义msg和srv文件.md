# 创建自定义msg和srv文件



**目标：** 自定义接口文件（``.msg`` 和 ``.srv``）并将它们与Python和C++节点一起使用。

**教程级别：** 初学者

**时间：** 20 分钟

**内容**
 


- ## 背景

  在之前的教程中，你使用消息和服务接口来了解[topics]、[services]和简单的发布者/订阅者和服务/客户端节点。

  在这些情况下，你使用的接口是预定义的。

  虽然使用预定义的接口定义是一种很好的做法，但有时你可能也需要定义自己的消息和服务。

  本教程将向你介绍创建自定义接口定义的最简单方法。
  
  
  
- ## 先决条件
 
  你应该有一个[ROS2工作区]。

  本教程还使用在发布者/订阅者和服务/客户端教程来尝试新的自定义消息。

- ## 任务

  1. ### 创建一个新包

      在本教程中，你将在自己的包中创建自定义```.msg```和```.srv```文件，然后在单独的包中使用它们。

      两个包应该在同一个工作区中。

      由于我们将使用之前教程中创建的pub/sub和service/client包，请确保你与这些包位于同一工作区（dev_ws/src），然后运行以下命令以创建新包：

      ```bash
      ros2 pkg create --build-type ament_cmake tutorial_interfaces
      ```
 

      tutorial_interfaces是新包的名称。

      注意它是一个CMake包；目前没有办法在纯Python包中生成```.msg```或```.srv```文件。

      你可以在CMake包中创建自定义接口，然后在Python节点中使用它，这将在上一节中介绍。

      将```.msg```和```.srv```文件保存在包内各自的目录中是一种很好的做法。

      在“dev_ws/src/tutorial_interfaces”中创建目录：

      ```bash
      sudo mkdir msg
      sudo mkdir srv
      ```

 

  2. ### 创建自定义

      1. #### 消息定义 


          在刚刚创建的tutorial_interfaces/msg目录中，创建一个名为Num.msg的新文件，其中包含一行代码声明其数据结构：

          ```
          int64 num
          ```

          这是你的自定义消息，它传输一个名为“num”的单个64位整数。



      2. ### 服务定义

          回到刚刚创建的tutorial_interfaces/srv目录，创建一个名为AddThreeInts.srv的新文件，其请求和响应结构如下：

          ```
          int64 a
          int64 b
          int64 c
          ---
          int64 sum
          ```

          这是你的自定义服务，它请求三个名为“a”、“b”和“c”的整数，并以一个名为“sum”的整数作为响应。




  3. ### CMakeLists.txt

      要将你定义的接口转换为特定于语言的代码（如C++和Python），以便它们可以在这些语言中使用，请将以下几行添加到“CMakeLists.txt”：

      ```cmake
      find_package(rosidl_default_generators REQUIRED)

      rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Num.msg"
        "srv/AddThreeInts.srv"
       )
      ```

 

  4. ### package.xml

      由于接口依赖于rosidl_default_generators 来生成特定于语言的代码，因此您需要声明对它的依赖。

      将以下行添加到``package.xml``

      ```xml
      <build_depend>rosidl_default_generators</build_depend>

      <exec_depend>rosidl_default_runtime</exec_depend>

      <member_of_group>rosidl_interface_packages</member_of_group>
      ```
 

  5. ### 构建tutorial_interfaces包

      现在你的自定义接口包的所有部分都已就位，你可以构建该包。

      在工作区的根目录 (~/dev_ws) 中，运行以下命令：

      ```bash
      colcon build --packages-select tutorial_interfaces
      ```

      现在，其他ROS2软件包可以发现这些接口。

 

  6. ### 确认 msg 和 srv 创建

      在新终端中，从你的工作区（dev_ws）运行以下命令以获取它：

      ```bash
      . install/setup.bash
      ```

      现在，你可以使用```ros2 interface show```命令确认您的界面创建是否有效：

      ```bash
      ros2 interface show tutorial_interfaces/msg/Num
      ```

      应该返回：

      ```
      int64 num
      ```

      和

      ```bash
      ros2 interface show tutorial_interfaces/srv/AddThreeInts
      ```

      应该返回：

      ```
      int64 a
      int64 b
      int64 c
      ---
      int64 sum
      ```



  7. ### 测试新接口

      对于此步骤，你可以使用你在之前的教程中创建的包。

      对节点、CMakeLists和package文件的一些简单修改将允许你使用新的接口。

 

      1. #### 使用pub/sub测试Num.msg
      
          对上一教程中创建的发布者/订阅者包稍加修改，你可以看到Num.msg正在运行。

          由于你要将标准字符串msg更改为数字字符串，因此输出将略有不同。

 

          发布者：

          ```c++
          #include <chrono>
          #include <memory>

          #include "rclcpp/rclcpp.hpp"
          #include "tutorial_interfaces/msg/num.hpp"                                            // CHANGE

          using namespace std::chrono_literals;

          class MinimalPublisher : public rclcpp::Node
          {
          public:
            MinimalPublisher()
            : Node("minimal_publisher"), count_(0)
            {
              publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);  // CHANGE
              timer_ = this->create_wall_timer(
                500ms, std::bind(&MinimalPublisher::timer_callback, this));
            }

          private:
            void timer_callback()
            {
              auto message = tutorial_interfaces::msg::Num();                                   // CHANGE
              message.num = this->count_++;                                                     // CHANGE
              RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'");    // CHANGE
              publisher_->publish(message);
            }
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;             // CHANGE
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



          订阅者：

          ```c++
          #include <functional>
          #include <memory>

          #include "rclcpp/rclcpp.hpp"
          #include "tutorial_interfaces/msg/num.hpp"                                       // CHANGE

          using std::placeholders::_1;

          class MinimalSubscriber : public rclcpp::Node
          {
          public:
            MinimalSubscriber()
            : Node("minimal_subscriber")
            {
              subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(    // CHANGE
                "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
            }

          private:
            void topic_callback(const tutorial_interfaces::msg::Num::SharedPtr msg) const  // CHANGE
            {
              RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg->num << "'");     // CHANGE
            }
            rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;  // CHANGE
          };

          int main(int argc, char * argv[])
          {
            rclcpp::init(argc, argv);
            rclcpp::spin(std::make_shared<MinimalSubscriber>());
            rclcpp::shutdown();
            return 0;
          }
          ```



          CMakeLists.txt：

          添加以下几行（仅限 C++）：

          ```cmake
          #...

          find_package(ament_cmake REQUIRED)
          find_package(rclcpp REQUIRED)
          find_package(tutorial_interfaces REQUIRED)                      # CHANGE

          add_executable(talker src/publisher_member_function.cpp)
          ament_target_dependencies(talker rclcpp tutorial_interfaces)    # CHANGE

          add_executable(listener src/subscriber_member_function.cpp)
          ament_target_dependencies(listener rclcpp tutorial_interfaces)  # CHANGE

          install(TARGETS
            talker
            listener
            DESTINATION lib/${PROJECT_NAME})

          ament_package()
          ```



          package.xml：

          添加以下行：

          ```xml
          <depend>tutorial_interfaces</depend>
          ```

          进行上述编辑并保存所有更改后，构建包：

          ```bash
          colcon build --packages-select cpp_pubsub
          ```

          然后打开两个新终端，在每个终端中读取并执行设置文件中的shell命令，然后运行：

          ```bash
          ros2 run cpp_pubsub talker
          ros2 run cpp_pubsub listener
          ```

          由于Num.msg只传递一个整数，所以发布者应该只发布整数值，而不是它之前发布的字符串：

          ```
          [INFO] [minimal_publisher]: Publishing: '0'
          [INFO] [minimal_publisher]: Publishing: '1'
          [INFO] [minimal_publisher]: Publishing: '2'
          ```
 

      2. #### 使用服务/客户端测试AddThreeInts.srv

          对上一教程中创建的服务/客户端包稍加修改，你可以看到“AddThreeInts.srv” 的运行情况。

          由于你要将原始的两个整数请求srv更改为三个整数请求srv，因此输出将略有不同。
          
          服务：

          ```c++
           #include "rclcpp/rclcpp.hpp"
           #include "tutorial_interfaces/srv/add_three_ints.hpp"                                        // CHANGE

           #include <memory>

           void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,     // CHANGE
                     std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>       response)  // CHANGE
           {
             response->sum = request->a + request->b + request->c;                                      // CHANGE
             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",  // CHANGE
                           request->a, request->b, request->c);                                         // CHANGE
             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
           }

           int main(int argc, char **argv)
           {
             rclcpp::init(argc, argv);

             std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");   // CHANGE

             rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service =               // CHANGE
               node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints",  &add);   // CHANGE

             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");                     // CHANGE

             rclcpp::spin(node);
             rclcpp::shutdown();
           }
           ```



           客户：

           ```c++
           #include "rclcpp/rclcpp.hpp"
           #include "tutorial_interfaces/srv/add_three_ints.hpp"                                       // CHANGE

           #include <chrono>
           #include <cstdlib>
           #include <memory>

           using namespace std::chrono_literals;

           int main(int argc, char **argv)
           {
             rclcpp::init(argc, argv);

             if (argc != 4) { // CHANGE
                 RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_three_ints_client X Y Z");      // CHANGE
                 return 1;
             }

             std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client");  // CHANGE
             rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedPtr client =                // CHANGE
               node->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints");          // CHANGE

             auto request = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>();       // CHANGE
             request->a = atoll(argv[1]);
             request->b = atoll(argv[2]);
             request->c = atoll(argv[3]);                                                              // CHANGE

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
               RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");    // CHANGE
             }

             rclcpp::shutdown();
             return 0;
           }
           ```

           CMakeLists.txt：

           ```
           #...

           find_package(ament_cmake REQUIRED)
           find_package(rclcpp REQUIRED)
           find_package(tutorial_interfaces REQUIRED)         # CHANGE

           add_executable(server src/add_two_ints_server.cpp)
           ament_target_dependencies(server
             rclcpp tutorial_interfaces)                      # CHANGE

           add_executable(client src/add_two_ints_client.cpp)
           ament_target_dependencies(client
             rclcpp tutorial_interfaces)                      # CHANGE

           install(TARGETS
             server
             client
             DESTINATION lib/${PROJECT_NAME})

           ament_package()
           ```

           package.xml：

           ```xml
           <depend>tutorial_interfaces</depend>
           ```

           进行上述编辑并保存所有更改后，构建包：

           ```bash
           colcon build --packages-select cpp_srvcli
           ```

           然后打开两个新终端，在每个终端中读取并执行设置文件中的shell命令，然后运行：

           ```bash
           ros2 run cpp_srvcli server
           ros2 run cpp_srvcli client 2 3 1
           ```

- ## 总结

  在本教程中，你学习了如何在自己的包中创建自定义接口以及如何在其他包中利用这些接口。

  这是一种创建和使用接口的简单方法。

  你可以在[nterfaceConcept]处了解更多关于接口的信息：[这里]。

 

- ## 下一步

  [下一个教程]涵盖了在ROS2中使用接口的更多方法。

 
