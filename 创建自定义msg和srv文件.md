# 创建自定义 ROS 2 msg 和 srv 文件



**目标：** 定义自定义接口文件（``.msg`` 和 ``.srv``）并将它们与 Python 和 C++ 节点一起使用。

 

**教程级别：**初学者

 

**时间：** 20 分钟

 

.. 内容:: 内容

：深度：2

：当地的：

 

背景

----------

 

在之前的教程中，您使用消息和服务接口来了解 :ref:`topics <ROS2Topics>`、:ref:`services <ROS2Services>` 和简单的发布者/订阅者（:ref:`C++<CppPubSub>`/:ref :`Python<PyPubSub>`) 和服务/客户端 (:ref:`C++<CppSrvCli>`/:ref:`Python<PySrvCli>`) 节点。

在这些情况下，您使用的接口是预定义的。

 

虽然使用预定义的接口定义是一种很好的做法，但有时您可能也需要定义自己的消息和服务。

本教程将向您介绍创建自定义接口定义的最简单方法。

 

先决条件

-------------

 

你应该有一个 :ref:`ROS 2 工作区 <ROS2Workspace>`。

 

本教程还使用在发布者/订阅者（:ref:`C++ <CppPubSub>` 和 :ref:`Python<PyPubSub>`）和服务/客户端（:ref:`C++ <CppSrvCli>` 和 :ref :`Python<PySrvCli>`) 教程来尝试新的自定义消息。

 

任务

-----

 

创建一个新包
^^^^^^^^^^^^^^^^^^^^^^^

 

在本教程中，您将在自己的包中创建自定义 ``.msg`` 和 ``.srv`` 文件，然后在单独的包中使用它们。

两个包应该在同一个工作区中。

 

由于我们将使用之前教程中创建的 pub/sub 和 service/client 包，请确保您与这些包位于同一工作区（``dev_ws/src``），然后运行以下命令以创建新包：

 

.. 代码块::控制台

 

ros2 pkg create --build-type ament_cmake tutorial_interfaces

 

``tutorial_interfaces`` 是新包的名称。

注意它是一个 CMake 包；目前没有办法在纯 Python 包中生成 ``.msg`` 或 ``.srv`` 文件。

您可以在 CMake 包中创建自定义接口，然后在 Python 节点中使用它，这将在上一节中介绍。

 

将 ``.msg`` 和 ``.srv`` 文件保存在包内各自的目录中是一种很好的做法。

在“dev_ws/src/tutorial_interfaces”中创建目录：

 

.. 代码块::控制台

 

mkdir msg

 

目录服务器

 

创建自定义定义
^^^^^^^^^^^^^^^^^^^^^^^^^^^

 

2.1. 消息定义 

~~~~~~~~~~~~~~~~~~

 

在刚刚创建的 ``tutorial_interfaces/msg`` 目录中，创建一个名为 ``Num.msg`` 的新文件，其中包含一行代码声明其数据结构：

 

.. 代码块::控制台

 

int64 号

 

这是您的自定义消息，它传输一个名为“num”的单个 64 位整数。

 

2.2. 服务定义 

~~~~~~~~~~~~~~~~~~

 

回到刚刚创建的 ``tutorial_interfaces/srv`` 目录，创建一个名为 ``AddThreeInts.srv`` 的新文件，其请求和响应结构如下：

 

.. 代码块::控制台

 

int64 一个

int64 b

int64 c

---

int64 总和

 

这是您的自定义服务，它请求三个名为“a”、“b”和“c”的整数，并以一个名为“sum”的整数作为响应。

 

``CMakeLists.txt``
^^^^^^^^^^^^^^^^^^^^

 

要将您定义的接口转换为特定于语言的代码（如 C++ 和 Python），以便它们可以在这些语言中使用，请将以下几行添加到“CMakeLists.txt”：

 

.. 代码块:: cmake

 

find_package(rosidl_default_generators REQUIRED)

 

rosidl_generate_interfaces(${PROJECT_NAME}

"msg/Num.msg"

“srv/AddThreeInts.srv”

)

 

``package.xml``
^^^^^^^^^^^^^^^^^^

 

由于接口依赖于 rosidl_default_generators 来生成特定于语言的代码，因此您需要声明对它的依赖。

将以下行添加到``package.xml``

 

.. 代码块:: xml

 

<build_depend>rosidl_default_generators</build_depend>

 

<exec_depend>rosidl_default_runtime</exec_depend>

 

<member_of_group>rosidl_interface_packages</member_of_group>

 

构建 ``tutorial_interfaces`` 包
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

 

现在您的自定义接口包的所有部分都已就位，您可以构建该包。

在工作区的根目录 (``~/dev_ws``) 中，运行以下命令：

 

.. 标签::

 

.. 组标签:: Linux

 

.. 代码块::控制台

 

colcon build --packages-select tutorial_interfaces

 

.. 组标签:: macOS

 

.. 代码块::控制台

 

colcon build --packages-select tutorial_interfaces

 

.. 组标签:: Windows

 

.. 代码块::控制台

 

colcon build --merge-install --packages-select tutorial_interfaces

 

现在，其他 ROS 2 软件包可以发现这些接口。

 

确认 msg 和 srv 创建
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

 

在新终端中，从您的工作区（``dev_ws``）运行以下命令以获取它：

 

.. 标签::

 

.. 组标签:: Linux

 

.. 代码块::控制台

 

. 安装/设置.bash

 

.. 组标签:: macOS

 

.. 代码块::控制台

 

. 安装/设置.bash

 

.. 组标签:: Windows

 

.. 代码块::控制台

 

调用 install/setup.bat

 

现在，您可以使用“ros2 interface show”命令确认您的界面创建是否有效：

 

 

.. 代码块::控制台

 

ros2界面显示tutorial_interfaces/msg/Num

 

应该返回：

 

.. 代码块::控制台

 

int64 号

 

和

 

.. 代码块::控制台

 

ros2界面显示tutorial_interfaces/srv/AddThreeInts

 

应该返回：

 

.. 代码块::控制台

 

int64 一个

int64 b

int64 c

---

int64 总和

 

测试新接口
^^^^^^^^^^^^^^^^^^^^^^^^^

 

对于此步骤，您可以使用您在之前的教程中创建的包。

对节点、``CMakeLists`` 和``package`` 文件的一些简单修改将允许您使用新的接口。

 

7.1. 使用 pub/sub 测试“Num.msg” 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 

对上一教程中创建的发布者/订阅者包（:ref:`C++ <CppPubSub>` 或 :ref:`Python <PyPubSub>`）稍加修改，您可以看到 Num.msg 正在运行。

由于您要将标准字符串 msg 更改为数字字符串，因此输出将略有不同。

 

出版商：

 

.. 标签::

 

.. 组标签:: C++

 

.. 代码块:: C++

 

#include <时间>

#include <内存>

 

#include "rclcpp/rclcpp.hpp"

#include "tutorial_interfaces/msg/num.hpp" // 改变

 

使用命名空间 std::chrono_literals;

 

类 MinimalPublisher : 公共 rclcpp::Node

{

民众：

最小发布者()

：节点（“minimal_publisher”），count_（0）

{

Publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10); // 改变

timer_ = this->create_wall_timer(

500ms, std::bind(&MinimalPublisher::timer_callback, this));

}

 

私人的：

void timer_callback()

{

自动消息 = tutorial_interfaces::msg::Num(); // 改变

message.num = this->count_++; // 改变

RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'"); // 改变

发布者_->发布（消息）；

}

rclcpp::TimerBase::SharedPtr timer_;

rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_; // 改变

size_t 计数_；

};

 

int main(int argc, char * argv[])

{

rclcpp::init(argc, argv);

rclcpp::spin(std::make_shared<MinimalPublisher>());

rclcpp::关机();

返回0；

}

 

.. 组标签:: Python

 

.. 代码块:: python

 

导入 rclpy

从 rclpy.node 导入节点

 

from tutorial_interfaces.msg import Num # CHANGE

 

 

类 MinimalPublisher（节点）：

 

def __init__(self):

super().__init__('minimal_publisher')

self.publisher_ = self.create_publisher(Num, 'topic', 10) # CHANGE

定时器周期 = 0.5

self.timer = self.create_timer(timer_period, self.timer_callback)

自我.i = 0

 

def timer_callback(self):

msg = Num() # 改变

msg.num = self.i # 改变

self.publisher_.publish(msg)

self.get_logger().info('Publishing: "%d"' % msg.num) # CHANGE

self.i += 1

 

 

定义主（参数=无）：

rclpy.init(args=args)

 

minimum_publisher = MinimalPublisher()

 

rclpy.spin(minimal_publisher)

 

minimum_publisher.destroy_node()

rclpy.shutdown()

 

 

如果 __name__ == '__main__'：

主要的（）

 

 

订户：

 

.. 标签::

 

.. 组标签:: C++

 

.. 代码块:: C++

 

#include <功能>

#include <内存>

 

#include "rclcpp/rclcpp.hpp"

#include "tutorial_interfaces/msg/num.hpp" // 改变

 

使用 std::placeholders::_1;

 

类 MinimalSubscriber : 公共 rclcpp::Node

{

民众：

最小订阅者()

：节点（“minimal_subscriber”）

{

subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>( // 改变

"主题", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

}

 

私人的：

void topic_callback(const tutorial_interfaces::msg::Num::SharedPtr msg) const // 改变

{

RCLCPP_INFO_STREAM(this->get_logger(), "我听说：'" << msg->num << "'"); // 改变

}

rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_; // 改变

};

 

int main(int argc, char * argv[])

{

rclcpp::init(argc, argv);

rclcpp::spin(std::make_shared<MinimalSubscriber>());

rclcpp::关机();

返回0；

}

 

.. 组标签:: Python

 

.. 代码块:: python

 

导入 rclpy

从 rclpy.node 导入节点

 

from tutorial_interfaces.msg import Num # CHANGE

 

 

类 MinimalSubscriber（节点）：

 

def __init__(self):

super().__init__('minimal_subscriber')

self.subscription = self.create_subscription(

编号，# 更改

'话题'，

self.listener_callback,

10)

自我订阅

 

def listener_callback(self, msg):

self.get_logger().info('我听说: "%d"' % msg.num) # CHANGE

 

 

定义主（参数=无）：

rclpy.init(args=args)

 

最小订阅者 = 最小订阅者（）

 

rclpy.spin(minimal_subscriber)

 

minimum_subscriber.destroy_node()

rclpy.shutdown()

 

 

如果 __name__ == '__main__'：

主要的（）

 

 

CMakeLists.txt：

 

添加以下几行（仅限 C++）：

 

.. 代码块:: cmake

 

#...

 

find_package(ament_cmake 需要)

find_package（需要rclcpp）

find_package(tutorial_interfaces REQUIRED) # CHANGE

 

add_executable(talker src/publisher_member_function.cpp)

ament_target_dependencies(talker rclcpp tutorial_interfaces) # CHANGE

 

add_executable(listener src/subscriber_member_function.cpp)

ament_target_dependencies(listener rclcpp tutorial_interfaces) # CHANGE

 

安装（目标

健谈者

听众

目的地库/${PROJECT_NAME})

 

ament_package()

 

 

包.xml：

 

添加以下行：

 

.. 标签::

 

.. 组标签:: C++

 

.. 代码块:: C++

 

<depend>tutorial_interfaces</depend>

 

.. 组标签:: Python

 

.. 代码块:: python

 

<exec_depend>tutorial_interfaces</exec_depend>

 

 

进行上述编辑并保存所有更改后，构建包：

 

.. 标签::

 

.. 组标签:: C++

 

.. 代码块::控制台

 

colcon build --packages-select cpp_pubsub

 

在 Windows 上：

 

.. 代码块::控制台

 

colcon build --merge-install --packages-select cpp_pubsub

 

.. 组标签:: Python

 

.. 代码块::控制台

 

colcon 构建 --packages-select py_pubsub

 

在 Windows 上：

 

.. 代码块::控制台

 

colcon 构建 --merge-install --packages-select py_pubsub

 

然后打开两个新终端，在每个终端中输入“dev_ws”，然后运行：

 

.. 标签::

 

.. 组标签:: C++

 

.. 代码块::控制台

 

ros2 运行 cpp_pubsub 谈话者

 

.. 代码块::控制台

 

ros2 运行 cpp_pubsub 监听器

 

.. 组标签:: Python

 

.. 代码块::控制台

 

ros2 运行 py_pubsub 谈话者

 

.. 代码块::控制台

 

ros2 运行 py_pubsub 监听器

 

由于 Num.msg 只传递一个整数，所以说话者应该只发布整数值，而不是它之前发布的字符串：

 

.. 代码块::控制台

 

[信息] [minimal_publisher]：发布：'0'

[信息] [minimal_publisher]：发布：'1'

[信息] [minimal_publisher]：发布：'2'

 

 

7.2. 使用服务/客户端测试“AddThreeInts.srv” 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ~~

 

对上一教程中创建的服务/客户端包（:ref:`C++ <CppSrvCli>` 或 :ref:`Python <PySrvCli>`）稍加修改，您可以看到“AddThreeInts.srv” 的运行情况。

由于您要将原始的两个整数请求 srv 更改为三个整数请求 srv，因此输出将略有不同。

 

服务：

 

.. 标签::

 

.. 组标签:: C++

 

.. 代码块:: C++

 

#include "rclcpp/rclcpp.hpp"

#include "tutorial_interfaces/srv/add_three_ints.hpp" // 改变

 

#include <内存>

 

void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request, // CHANGE

std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response> response) // CHANGE

{

response->sum = request->a + request->b + request->c; // 改变

RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "传入请求\na: %ld" " b: %ld" " c: %ld", // CHANGE

请求->a，请求->b，请求->c)；// 改变

RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "发回响应: [%ld]", (long int)response->sum);

}

 

int main(int argc, char **argv)

{

rclcpp::init(argc, argv);

 

std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server"); // 改变

 

rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service = // 改变

node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints", &add); // 改变

 

RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "准备添加三个整数。"); // 改变

 

rclcpp::旋转（节点）；

rclcpp::关机();

}

 

.. 组标签:: Python

 

.. 代码块:: python

 

from tutorial_interfaces.srv import AddThreeInts # CHANGE

 

导入 rclpy

从 rclpy.node 导入节点

 

 

类 MinimalService（节点）：

 

def __init__(self):

super().__init__('minimal_service')

self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback) # CHANGE

 

def add_three_ints_callback(self, request, response):

response.sum = request.a + request.b + request.c # CHANGE

self.get_logger().info('传入请求\na: %db: %dc: %d' % (request.a, request.b, request.c)) # CHANGE

 

返回响应

 

定义主（参数=无）：

rclpy.init(args=args)

 

minimum_service = MinimalService()

 

rclpy.spin(minimal_service)

 

rclpy.shutdown()

 

如果 __name__ == '__main__'：

主要的（）

 

客户：

 

.. 标签::

 

.. 组标签:: C++

 

.. 代码块:: C++

 

#include "rclcpp/rclcpp.hpp"

#include "tutorial_interfaces/srv/add_three_ints.hpp" // 改变

 

#include <时间>

#include <cstdlib>

#include <内存>

 

使用命名空间 std::chrono_literals;

 

int main(int argc, char **argv)

{

rclcpp::init(argc, argv);

 

if (argc != 4) { // 改变

RCLCPP_INFO（rclcpp::get_logger（“rclcpp”），“用法：add_three_ints_client XY Z”）；// 改变

返回 1；

}

 

std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client"); // 改变

rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedPtr client = // 改变

节点->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints"); // 改变

 

自动请求 = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>(); // 改变

request->a = atoll(argv[1]);

request->b = atoll(argv[2]);

request->c = atoll(argv[3]); // 改变

 

而 (!client->wait_for_service(1s)) {

如果 (!rclcpp::ok()) {

RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "在等待服务时中断。退出。");

返回0；

}

RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务不可用，再次等待...");

}

 

自动结果 = 客户端->async_send_request(request);

// 等待结果。

如果（rclcpp::spin_until_future_complete（节点，结果）==

rclcpp::FutureReturnCode::SUCCESS)

{

RCLCPP_INFO（rclcpp::get_logger（“rclcpp”），“总和：%ld”，result.get（）->总和）；

} 别的 {

RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "调用服务 add_three_ints 失败"); // 改变

}

 

rclcpp::关机();

返回0；

}

 

.. 组标签:: Python

 

.. 代码块:: python

 

from tutorial_interfaces.srv import AddThreeInts # CHANGE

导入系统

导入 rclpy

从 rclpy.node 导入节点

 

 

类 MinimalClientAsync(Node):

 

def __init__(self):

super().__init__('minimal_client_async')

self.cli = self.create_client(AddThreeInts, 'add_three_ints') # CHANGE

而不是 self.cli.wait_for_service(timeout_sec=1.0)：

self.get_logger().info('服务不可用，再次等待...')

self.req = AddThreeInts.Request() # CHANGE

 

def send_request(self):

self.req.a = int(sys.argv[1])

self.req.b = int(sys.argv[2])

self.req.c = int(sys.argv[3]) # 改变

self.future = self.cli.call_async(self.req)

 

 

定义主（参数=无）：

rclpy.init(args=args)

 

minimum_client = MinimalClientAsync()

minimum_client.send_request()

 

而 rclpy.ok():

rclpy.spin_once(minimal_client)

如果 minimum_client.future.done():

尝试：

响应 = minimum_client.future.result()

除了作为 e 的例外：

minimum_client.get_logger().info(

'服务调用失败 %r' % (e,))

别的：

minimum_client.get_logger().info(

'add_three_ints 的结果：对于 %d + %d + %d = %d' % # CHANGE

(minimal_client.req.a, minimum_client.req.b, minimum_client.req.c, response.sum)) # CHANGE

休息

 

minimum_client.destroy_node()

rclpy.shutdown()

 

 

如果 __name__ == '__main__'：

主要的（）

 

 

 

CMakeLists.txt：

 

添加以下几行（仅限 C++）：

 

.. 代码块:: cmake

 

#...

 

find_package(ament_cmake 需要)

find_package（需要rclcpp）

find_package(tutorial_interfaces REQUIRED) # CHANGE

 

add_executable（服务器源代码/add_two_ints_server.cpp）

ament_target_dependencies（服务器

rclcpp tutorial_interfaces) # CHANGE

 

add_executable（客户端 src/add_two_ints_client.cpp）

ament_target_dependencies（客户端

rclcpp tutorial_interfaces) # CHANGE

 

安装（目标

服务器

客户

目的地库/${PROJECT_NAME})

 

ament_package()

 

 

包.xml：

 

添加以下行：

 

.. 标签::

 

.. 组标签:: C++

 

.. 代码块:: C++

 

<depend>tutorial_interfaces</depend>

 

.. 组标签:: Python

 

.. 代码块:: python

 

<exec_depend>tutorial_interfaces</exec_depend>

 

 

进行上述编辑并保存所有更改后，构建包：

 

.. 标签::

 

.. 组标签:: C++

 

.. 代码块::控制台

 

colcon build --packages-select cpp_srvcli

 

在 Windows 上：

 

.. 代码块::控制台

 

colcon build --merge-install --packages-select cpp_srvcli

 

 

.. 组标签:: Python

 

.. 代码块::控制台

 

colcon build --packages-select py_srvcli

 

在 Windows 上：

 

.. 代码块::控制台

 

colcon build --merge-install --packages-select py_srvcli

 

然后打开两个新终端，在每个终端中输入“dev_ws”，然后运行：

 

.. 标签::

 

.. 组标签:: C++

 

.. 代码块::控制台

 

ros2 运行 cpp_srvcli 服务器

 

.. 代码块::控制台

 

ros2 运行 cpp_srvcli 客户端 2 3 1

 

.. 组标签:: Python

 

.. 代码块::控制台

 

ros2 运行 py_srvcli 服务

 

.. 代码块::控制台

 

ros2 运行 py_srvcli 客户端 2 3 1

 

 

概括

-------

 

在本教程中，您学习了如何在自己的包中创建自定义接口以及如何在其他包中利用这些接口。

 

这是一种创建和使用界面的简单方法。

您可以在 <InterfaceConcept> 处了解更多关于接口的信息：ref:`这里。

 

下一步

----------

 

:ref:`下一个教程 <SinglePkgInterface>` 涵盖了在 ROS 2 中使用接口的更多方法。

 
