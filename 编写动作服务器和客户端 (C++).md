# 编写动作服务器和客户端 (C++)


 

**目标：** 用C++实现动作服务器和客户端。

**教程级别：** 中级

**时间：** 15分钟




- ## 背景

动作是 ROS 中的一种异步通信形式。

动作客户端向动作服务器发送目标请求。  

动作服务器向动作客户端发送目标反馈和结果。  

 

- ## 先决条件

你将需要```action_tutorials_interfaces```包和```Fibonacci.action```

上一个教程中定义的接口，:ref:`ActionCreate`。

 

- ## 任务

1. ### 创建action_tutorials_cpp 包

正如我们在 :ref:`CreatePkg` 教程中看到的，我们需要创建一个新包来保存我们的C++ 和支持代码。

 

1.1. #### 创建 action_tutorials_cpp 包 

 

进入你在 :ref:`上一教程 <ActionCreate>` 中创建的操作工作区（记得找到工作区的源），并为 C++ 操作服务器创建一个新包：

```bash
cd ~/action_ws/src
ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp
```


1.2. 添加可见性控制 

为了使包在 Windows 上编译和工作，我们需要添加一些“可见性控制”。

有关为什么需要这样做的详细信息，请参阅[此处](https://docs.microsoft.com/en-us/cpp/cpp/dllexport-dllimport)。

打开action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h，放入如下代码：

```C++
#ifndef ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
#define ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((dllexport))
    #define ACTION_TUTORIALS_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTION_TUTORIALS_CPP_EXPORT __declspec(dllexport)
    #define ACTION_TUTORIALS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_TUTORIALS_CPP_BUILDING_DLL
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_EXPORT
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_IMPORT
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE ACTION_TUTORIALS_CPP_PUBLIC
  #define ACTION_TUTORIALS_CPP_LOCAL
#else
  #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((visibility("default")))
  #define ACTION_TUTORIALS_CPP_IMPORT
  #if __GNUC__ >= 4
    #define ACTION_TUTORIALS_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define ACTION_TUTORIALS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC
    #define ACTION_TUTORIALS_CPP_LOCAL
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
```

 

2. ### 编写动作服务器

让我们专注于编写一个动作服务器，使用我们在 :ref:`ActionCreate` 教程中创建的动作来计算斐波那契数列。

 

2.1. 编写动作服务器代码 

打开action_tutorials_cpp/src/fibonacci_action_server.cpp，放入如下代码：

```c++
#include <functional>
#include <memory>
#include <thread>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

namespace action_tutorials_cpp
{
class FibonacciActionServer : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class FibonacciActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)

```
 

前几行包括我们需要编译的所有头文件。

接下来我们创建一个类，是一个派生类的```rclcpp :: Node```：

```c++
class FibonacciActionServer : public rclcpp::Node
```

```FibonacciActionServer```类的构造函数将节点名称初始化为```fibonacci_action_server```：

```c++
  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("fibonacci_action_server", options)
```

构造函数还实例化了一个新的动作服务器：

```c++
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
```

一个动作服务器需要 6 件事：

 

1. 模板化的动作类型名称：“Fibonacci”。
2. 将操作添加到的ROS2节点：```this```。
3. 动作名称：```'fibonacci'```。
4. 处理目标的回调函数：```handle_goal```。
5. 处理取消的回调函数：```handle_cancel```。
6. 用于处理目标接受的回调函数：```handle_accept```。
 

文件中的下一个是各种回调的实现。

请注意，所有回调都需要快速返回，否则我们可能会饿死 executor。

我们从处理新目标的回调开始：

```c++
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
```

此实现仅接受所有目标。

接下来是处理取消的回调：

```c++
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
```

这个实现只是告诉客户端它接受了取消。

最后一个回调接受一个新目标并开始处理它：

```c++
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
  }
```

由于执行是一个长时间运行的操作，我们产生一个线程来完成实际工作并快速从```handle_accepted```返回。

所有进一步的处理和更新都在新线程的“execute”方法中完成：

```c++
  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
```

这个工作线程每秒处理一个斐波那契数列的序列号，为每一步发布一个反馈更新。

当它完成处理时，它将“goal_handle”标记为成功，然后退出。

我们现在有一个功能齐全的动作服务器。让我们构建并运行它。



2.2. 编译动作服务器

在上一节中，我们将操作服务器代码放置到位。

为了让它编译和运行，我们需要做一些额外的事情。

首先我们需要设置 CMakeLists.txt 以便编译动作服务器。

打开action_tutorials_cpp/CMakeLists.txt，并在find_package调用之后添加以下内容：

```
add_library(action_server SHARED
  src/fibonacci_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

现在我们可以编译这个包了。
转到“action_ws”的顶层，然后运行：

```bash
colcon build
```

这应该编译整个工作区，包括“action_tutorials_cpp”包中的“fibonacci_action_server”。

 

2.3. 运行动作服务器 

现在我们已经构建了动作服务器，我们可以运行它。

获取我们刚刚构建的工作空间（action_ws），并尝试运行操作服务器：

```bash
ros2 run action_tutorials_cpp fibonacci_action_server
```



3. ### 编写动作客户端

3.1. #### 编写动作客户端代码

打开action_tutorials_cpp/src/fibonacci_action_client.cpp，放入如下代码：

```c++
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)

```

前几行包括我们需要编译的所有头文件。

接下来我们创建一个类，是一个派生类的```rclcpp :: Node```：

```c++
class FibonacciActionClient : public rclcpp::Node
```

FibonacciActionClient类的构造函数将节点名称初始化为fibonacci_action_client：

```c++
  explicit FibonacciActionClient(const rclcpp::NodeOptions & options) : Node("fibonacci_action_client", options)
```

构造函数还实例化了一个新的动作客户端：

```c++
this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");
```

一个动作客户端需要三件事：

1. 模板化的动作类型名称：“斐波那契”。
2. 将动作客户端添加到的 ROS 2 节点：```this```。
3. 动作名称：```'fibonacci'```。

我们还实例化了一个ROS计时器，它将启动对“send_goal”的唯一调用：

```bash
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));
```

当计时器到期时，它将调用```send_goal```：

```c++
  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }
```

该函数执行以下操作：

1. 取消计时器（所以它只被调用一次）。
2. 等待动作服务器出现。
3. 实例化一个新的“Fibonacci::Goal”。
4. 设置响应、反馈和结果回调。
5. 将目标发送到服务器。

当服务器接收并接受目标时，它会向客户端发送响应。

该响应由“goal_response_callback”处理：

```c++
  void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }
```

假设目标已被服务器接受，它将开始处理。

对客户端的任何反馈都将由“feedback_callback”处理：

```c++
  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }
```

当服务器完成处理时，它会将结果返回给客户端。

结果由“result_callback”处理：

```c++
  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
```

我们现在有一个功能齐全的动作客户端。让我们构建并运行它。

 

3.2. 编译动作客户端 

在上一节中，我们将操作客户端代码放置到位。

为了让它编译和运行，我们需要做一些额外的事情。

首先我们需要设置 CMakeLists.txt 以便编译动作客户端。

打开``action_tutorials_cpp/CMakeLists.txt``，并在``find_package`` 调用之后添加以下内容：

```bash
add_library(action_client SHARED
  src/fibonacci_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

现在我们可以编译这个包了。转到“action_ws”的顶层，然后运行：

```bash
colcon build
```

这应该编译整个工作区，包括“action_tutorials_cpp”包中的“fibonacci_action_client”。

 

3.3. 运行动作客户端


现在我们已经构建了动作客户端，我们可以运行它。

首先确保动作服务器在单独的终端中运行。

现在获取我们刚刚构建的工作区（``action_ws``），并尝试运行操作客户端：

```bash
ros2 run action_tutorials_cpp fibonacci_action_client
```

你应该会看到已接受的目标、正在打印的反馈以及最终结果的记录消息。

 

- ## 总结

在本教程中，你将一个 C++ 动作服务器和动作客户端逐行组合在一起，并将它们配置为交换目标、反馈和结果。

 

- ## 相关内容

有几种方法可以用C++ 编写动作服务器和客户端；
查看[ros2/examples](https://github.com/ros2/examples/tree/master/rclcpp)中的``minimal_action_server`` 和``minimal_action_client`` 包。  

更详细的ROS动作信息请参考[设计文章](http://design.ros2.org/articles/actions.html)。  
