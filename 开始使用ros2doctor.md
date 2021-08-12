# 开始使用ros2doctor



**目标：** 使用ros2doctor工具识别ROS2设置中的问题。

**教程级别：** 初学者

**时间：** 10分钟

**内容**



- ## 背景

    当你的ROS2设置未按预期运行时，你可以使用ros2doctor工具检查其设置。

    ros2doctor检查ROS2的各个方面，包括平台、版本、网络、环境、运行系统等，并警告您可能的错误和问题的原因。

- ## 先决条件

    ros2doctor是ros2cli包的一部分。 
    只要你安装了ros2cli（任何正常安装都应该有），你就可以使用ros2doctor。

    本教程使用海龟模拟器来说明一些示例。

- ## 任务

    1. ### 检查你的设置
    
        让我们使用ros2doctor来检查你的一般ROS2设置。
        首先，在新终端中输入ROS2，然后输入命令：
        
        ```
        ros2 doctor
        ```
        
        这将对你的所有设置模块进行检查并返回警告和错误。

        如果你的ROS2设置处于完美状态，你将看到类似于以下内容的消息：
        
        ```
        All <n> checks passed
        ```
        
        但是，返回一些警告并不罕见。 
        UserWarning并不意味着你的设置不可用； 
        这更有可能只是表明某些东西的配置方式并不理想。

        如果你确实收到警告，它将如下所示：
        
        ```
        <path>: <line>: UserWarning: <message>
        ```
        
        例如，如果你使用不稳定的ROS2发行版，ros2doctor会发现此警告：
        
        ```
        UserWarning: Distribution <distro> is not fully supported or tested. To get more consistent features, download a stable version at 
        https://index.ros.org/doc/ros2/Installation/
        ```
        
        如果```ros2doctor```只在你的系统中发现警告，
        你仍然会收到所有```<n>```检查已通过的消息。

        大多数检查被归类为警告而不是错误。
        ```ros2doctor```返回的反馈的重要性主要取决于你，用户。
        如果它确实在你的设置中发现了一个罕见的错误，由```UserWarning: ERROR:```指示，则检查被视为失败。
  
        你将在问题反馈列表后看到一条与此类似的消息：
  
        ```
        1/3 checks failed

        Failed modules:  network
        ```
  
        错误表示系统缺少对ROS2至关重要的重要设置或功能。
        应解决错误以确保系统正常运行。

  
  
  2. ### 检查系统

        你还可以检查正在运行的ROS2系统以确定问题的可能原因。
        要查看```ros2doctor```在正在运行的系统上工作，让我们运行海龟模拟器，它的节点之间积极地相互通信。

        通过打开一个新终端，采购ROS2并输入命令来启动系统：

        ```bash
        ros2 run turtlesim turtlesim_node
        ```

        打开另一个终端并获取ROS2以运行Teleop控件：

        ```bash
        ros2 run turtlesim turtle_teleop_key
        ```

        现在在它自己的终端中再次运行```ros2doctor```。
        你将看到上次在设置中运行```ros2doctor```时出现的警告和错误（如果有的话）。
        接下来是一些与系统本身相关的新警告：

        ```
        UserWarning: Publisher without subscriber detected on /turtle1/color_sensor.
        UserWarning: Publisher without subscriber detected on /turtle1/pose.
        ```
    
    似乎```/turtlesim```节点将数据发布到两个未被订阅的主题，```ros2doctor```认为这可能会导致问题。

    如果你运行命令来回显```/color_sensor```和```/pose```主题，这些警告将消失，因为发布者将拥有订阅者。

    你可以通过在海龟模拟器仍在运行时打开两个新终端来尝试此操作，在每个终端中采购ROS2，并在自己的终端中运行以下每个命令：
    
    ```bash
    ros2 topic echo /turtle1/color_sensor
    ros2 topic echo /turtle1/pose
    ```

    
    然后再次在其终端中运行 ros2doctor。没有订阅者警告的发布者将消失。 （确保在运行 echo 的终端中输入 Ctrl+C）。

    现在尝试退出turtlesim窗口或退出teleop并再次运行ros2doctor。现在系统中的一个节点不可用，您将看到更多警告，指示不同主题的发布者没有订阅者或没有发布者的订阅者。

    在具有许多节点的复杂系统中，ros2doctor 对于确定通信问题的可能原因非常有用。

    
    
    3. ### 获取完整报告

        虽然 ros2doctor 会让你知道关于你的网络、系统等的警告，但使用 --report 参数运行它会给你更多的细节来帮助你分析问题。

        如果您收到有关您的网络设置的警告并想确切地找出导致警告的配置部分，您可能需要使用 --report。

        当您需要打开支持工单以获取 ROS 2 帮助时，这也非常有用。您可以将报告的相关部分复制并粘贴到工单中，以便帮助您的人员更好地了解您的环境并提供更好的帮助。

        要获得完整报告，请在终端中输入以下命令：
    
    ```bash
    ros2 doctor --report
    ```
    
    将返回分为五组的信息列表：
    
    ```
    NETWORK CONFIGURATION
    ...

    PLATFORM INFORMATION
    ...

    RMW MIDDLEWARE
    ...

    ROS 2 INFORMATION
    ...

    TOPIC LIST
    ...
    ```
    
    你可以根据运行```ros2 doctor```时收到的警告交叉检查此处的信息。 
    例如，如果```ros2doctor```返回警告（前面提到过）你的发行版“不完全支持或测试”，你可以查看报告的ROS2信息部分：
    
    ```
    distribution name      : <distro>
    distribution type      : ros2
    distribution status    : prerelease
    release platforms      : {'<platform>': ['<version>']}
    ```
    
    
    在这里你可以看到分发状态为预发布，这解释了为什么不完全支持它。

    
    
- ## 总结

    ros2doctor会通知你ROS2设置和运行系统中的问题。
    你可以使用--report参数更深入地了解这些警告背后的信息。

    请记住，ros2doctor不是调试工具；
    它不会帮助你解决代码中或系统实现方面的错误。

    
    
- ## 相关内容

    ros2doctor的README会告诉你更多关于不同参数的信息。 
    你可能还想看看ros2doctor存储库，因为它对初学者非常友好，并且是开始贡献的好地方。

    
    
- ## 下一步

    你已完成初学者级别的教程！
