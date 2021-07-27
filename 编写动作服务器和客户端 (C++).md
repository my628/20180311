.. _ActionsCpp：

 

编写动作服务器和客户端 (C++)

==========================================

 

**目标：** 用 C++ 实现动作服务器和客户端。

 

**教程级别：** 中级

 

**时间：** 15 分钟

 

.. 内容:: 内容

：深度：2

：当地的：

 

背景

----------

 

动作是 ROS 中的一种异步通信形式。

*动作客户端*向*动作服务器*发送目标请求。  

*动作服务器*向*动作客户端*发送目标反馈和结果。  

 

先决条件

-------------

 

您将需要 ``action_tutorials_interfaces`` 包和 ``Fibonacci.action``

上一个教程中定义的接口，:ref:`ActionCreate`。

 

任务

-----

 

创建 action_tutorials_cpp 包
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

 

正如我们在 :ref:`CreatePkg` 教程中看到的，我们需要创建一个新包来保存我们的 C++ 和支持代码。

 

1.1. 创建 action_tutorials_cpp 包 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 

进入您在 :ref:`上一教程 <ActionCreate>` 中创建的操作工作区（记得找到工作区的源），并为 C++ 操作服务器创建一个新包：

 

 

.. 标签::

 

.. 组标签:: Linux

 

.. 代码块:: bash

 

cd ~/action_ws/src

ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

 

.. 组标签:: macOS

 

.. 代码块:: bash

 

cd ~/action_ws/src

ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

 

.. 组标签:: Windows

 

.. 代码块:: bash

 

cd \dev\action_ws\src

ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

 

1.2. 添加可见性控制 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 

为了使包在 Windows 上编译和工作，我们需要添加一些“可见性控制”。

有关为什么需要这样做的详细信息，请参阅`此处<https://docs.microsoft.com/en-us/cpp/cpp/dllexport-dllimport>`_。

 

打开``action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h``，放入如下代码：

 

.. 代码块:: C++

 

#ifndef ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

#define ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

 

#ifdef __cplusplus

外部“C”

{

＃万一

 

// 这个逻辑是从 gcc wiki 上的示例中借用（然后命名空间）的：

// https://gcc.gnu.org/wiki/Visibility

 

#如果定义了_WIN32 || 定义 __CYGWIN__

#ifdef __GNUC__

#define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((dllexport))

#define ACTION_TUTORIALS_CPP_IMPORT __attribute__ ((dllimport))

＃别的

#define ACTION_TUTORIALS_CPP_EXPORT __declspec(dllexport)

#define ACTION_TUTORIALS_CPP_IMPORT __declspec(dllimport)

＃万一

#ifdef ACTION_TUTORIALS_CPP_BUILDING_DLL

#define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_EXPORT

＃别的

#define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_IMPORT

＃万一

#define ACTION_TUTORIALS_CPP_PUBLIC_TYPE ACTION_TUTORIALS_CPP_PUBLIC

#define ACTION_TUTORIALS_CPP_LOCAL

＃别的

#define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((visibility("default")))

#define ACTION_TUTORIALS_CPP_IMPORT

#if __GNUC__ >= 4

#define ACTION_TUTORIALS_CPP_PUBLIC __attribute__ ((visibility("default")))

#define ACTION_TUTORIALS_CPP_LOCAL __attribute__ ((visibility("hidden")))

＃别的

#define ACTION_TUTORIALS_CPP_PUBLIC

#define ACTION_TUTORIALS_CPP_LOCAL

＃万一

#define ACTION_TUTORIALS_CPP_PUBLIC_TYPE

＃万一

 

#ifdef __cplusplus

}

＃万一

 

#endif // ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

 

编写动作服务器
^^^^^^^^^^^^^^^^^^^^^^^^^^

 

让我们专注于编写一个动作服务器，使用我们在 :ref:`ActionCreate` 教程中创建的动作来计算斐波那契数列。

 

2.1. 编写动作服务器代码 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 

打开``action_tutorials_cpp/src/fibonacci_action_server.cpp``，放入如下代码：

 

.. 文字包括:: server.cpp

：语言：C++

:linenos:

 

前几行包括我们需要编译的所有头文件。

 

接下来我们创建一个类，是一个派生类的``rclcpp :: Node``：

 

.. 文字包括:: server.cpp

：语言：C++

:行: 14

 

``FibonacciActionServer`` 类的构造函数将节点名称初始化为 ``fibonacci_action_server``：

 

.. 文字包括:: server.cpp

：语言：C++

：行：21-22

 

构造函数还实例化了一个新的动作服务器：

 

.. 文字包括:: server.cpp

：语言：C++

：行：26-31

 

一个动作服务器需要 6 件事：

 

模板化的动作类型名称：“斐波那契”。
将操作添加到的 ROS 2 节点：``this``。
动作名称：``'fibonacci'``。
处理目标的回调函数：``handle_goal``
处理取消的回调函数：``handle_cancel``。
用于处理目标接受的回调函数：``handle_accept``。
 

文件中的下一个是各种回调的实现。

请注意，所有回调都需要快速返回，否则我们可能会饿死 executor。

 

我们从处理新目标的回调开始：

 

.. 文字包括:: server.cpp

：语言：C++

：行：37-44

 

此实现仅接受所有目标。

 

接下来是处理取消的回调：

 

.. 文字包括:: server.cpp

：语言：C++

：行：46-52

 

这个实现只是告诉客户端它接受了取消。

 

最后一个回调接受一个新目标并开始处理它：

 

.. 文字包括:: server.cpp

：语言：C++

：行：54-59

 

由于执行是一个长时间运行的操作，我们产生一个线程来完成实际工作并快速从 ``handle_accepted`` 返回。

 

所有进一步的处理和更新都在新线程的“execute”方法中完成：

 

.. 文字包括:: server.cpp

：语言：C++

：行：61-95

 

这个工作线程每秒处理一个斐波那契数列的序列号，为每一步发布一个反馈更新。

当它完成处理时，它将“goal_handle”标记为成功，然后退出。

 

我们现在有一个功能齐全的动作服务器。让我们构建并运行它。

 

2.2. 编译动作服务器 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 

在上一节中，我们将操作服务器代码放置到位。

为了让它编译和运行，我们需要做一些额外的事情。

 

首先我们需要设置 CMakeLists.txt 以便编译动作服务器。

打开``action_tutorials_cpp/CMakeLists.txt``，并在``find_package`` 调用之后添加以下内容：

 

.. 代码块:: cmake

 

add_library(action_server 共享

src/fibonacci_action_server.cpp)

target_include_directories（action_server 私有

$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

$<INSTALL_INTERFACE:include>)

目标编译定义（action_server

私人“ACTION_TUTORIALS_CPP_BUILDING_DLL”）

ament_target_dependencies（动作服务器

“action_tutorials_interfaces”

“rccp”

“rclcpp_action”

"rclcpp_components")

rclcpp_components_register_node（action_server 插件“action_tutorials_cpp::FibonacciActionServer”可执行 fibonacci_action_server）

安装（目标

动作服务器

存档目的地库

图书馆目的地 lib

运行时目标箱）

 

现在我们可以编译这个包了。转到“action_ws”的顶层，然后运行：

 

.. 代码块:: bash

 

colcon 构建

 

这应该编译整个工作区，包括“action_tutorials_cpp”包中的“fibonacci_action_server”。

 

2.3. 运行动作服务器 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 

现在我们已经构建了动作服务器，我们可以运行它。

获取我们刚刚构建的工作空间（``action_ws``），并尝试运行操作服务器：

 

.. 代码块:: bash

 

ros2 运行 action_tutorials_cpp fibonacci_action_server

 

编写动作客户端
^^^^^^^^^^^^^^^^^^^^^^^^^^

 

7.1. 编写动作客户端代码 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 

打开``action_tutorials_cpp/src/fibonacci_action_client.cpp``，放入如下代码：

 

.. 文字包括:: client.cpp

：语言：C++

:linenos:

 

前几行包括我们需要编译的所有头文件。

 

接下来我们创建一个类，是一个派生类的``rclcpp :: Node``：

 

.. 文字包括:: client.cpp

：语言：C++

:行: 15

 

``FibonacciActionClient`` 类的构造函数将节点名称初始化为 ``fibonacci_action_client``：

 

.. 文字包括:: client.cpp

：语言：C++

：行：20-22

 

构造函数还实例化了一个新的动作客户端：

 

.. 文字包括:: client.cpp

：语言：C++

：行：24-26

 

一个动作客户端需要三件事：

 

模板化的动作类型名称：“斐波那契”。
将动作客户端添加到的 ROS 2 节点：``this``。
动作名称：``'fibonacci'``。
 

我们还实例化了一个 ROS 计时器，它将启动对“send_goal”的唯一调用：

 

.. 文字包括:: client.cpp

：语言：C++

：行：27-30

 

当计时器到期时，它将调用``send_goal``：

 

.. 文字包括:: client.cpp

：语言：C++

：行：32-57

 

该函数执行以下操作：

 

取消计时器（所以它只被调用一次）。
等待动作服务器出现。
实例化一个新的“Fibonacci::Goal”。
设置响应、反馈和结果回调。
将目标发送到服务器。
 

当服务器接收并接受目标时，它会向客户端发送响应。

该响应由“goal_response_callback”处理：

 

.. 文字包括:: client.cpp

：语言：C++

：行：62-71

 

假设目标已被服务器接受，它将开始处理。

对客户端的任何反馈都将由“feedback_callback”处理：

 

.. 文字包括:: client.cpp

：语言：C++

：行：72-83

 

当服务器完成处理时，它会将结果返回给客户端。

结果由“result_callback”处理：

 

.. 文字包括:: client.cpp

：语言：C++

：行：84-107

 

我们现在有一个功能齐全的动作客户端。让我们构建并运行它。

 

7.2. 编译动作客户端 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 

在上一节中，我们将操作客户端代码放置到位。

为了让它编译和运行，我们需要做一些额外的事情。

 

首先我们需要设置 CMakeLists.txt 以便编译动作客户端。

打开``action_tutorials_cpp/CMakeLists.txt``，并在``find_package`` 调用之后添加以下内容：

 

.. 代码块:: cmake

 

add_library(action_client 共享

src/fibonacci_action_client.cpp)

target_include_directories（action_client 私人

$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

$<INSTALL_INTERFACE:include>)

目标编译定义（action_client

私人“ACTION_TUTORIALS_CPP_BUILDING_DLL”）

ament_target_dependencies（action_client

“action_tutorials_interfaces”

“rccp”

“rclcpp_action”

"rclcpp_components")

rclcpp_components_register_node（action_client 插件“action_tutorials_cpp::FibonacciActionClient”可执行 fibonacci_action_client）

安装（目标

action_client

存档目的地库

图书馆目的地 lib

运行时目标箱）

 

现在我们可以编译这个包了。转到“action_ws”的顶层，然后运行：

 

.. 代码块:: bash

 

colcon 构建

 

这应该编译整个工作区，包括“action_tutorials_cpp”包中的“fibonacci_action_client”。

 

7.3. 运行动作客户端 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 

现在我们已经构建了动作客户端，我们可以运行它。

首先确保动作服务器在单独的终端中运行。

现在获取我们刚刚构建的工作区（``action_ws``），并尝试运行操作客户端：

 

.. 代码块:: bash

 

ros2 运行 action_tutorials_cpp fibonacci_action_client

 

您应该会看到已接受的目标、正在打印的反馈以及最终结果的记录消息。

 

概括

-------

 

在本教程中，您将一个 C++ 动作服务器和动作客户端逐行组合在一起，并将它们配置为交换目标、反馈和结果。

 

相关内容

---------------

 

*有几种方法可以用 C++ 编写动作服务器和客户端；查看`ros2/examples <https://github.com/ros2/examples/tree/master/rclcpp>`_ repo 中的``minimal_action_server`` 和``minimal_action_client`` 包。  

 

*更详细的ROS动作信息请参考`设计文章<http://design.ros2.org/articles/actions.html>`__。  

 
