# 创建你的第一个 ROS 2 包



**目标：** 使用CMake或Python创建一个新包，并运行其可执行文件。

**教程级别：** 初学者

**时间：** 15分钟

**内容**



- ## 背景
  
  1. ### 什么是ROS2包？
  
      一个包可以被认为是你的ROS2代码的容器。 
      如果你希望能够安装你的代码或与他人共享，那么你需要将其组织在一个包中。
      使用包，你可以发布你的ROS2工作，并允许其他人轻松构建和使用它。

      ROS2中的包创建使用ament作为其构建系统和colcon作为其构建工具。
      你可以使用官方支持的CMake或Python创建包，但确实存在其他构建类型。

  2. ### ROS2包由什么组成？
  
      ROS2 Python和CMake包都有自己的最低要求内容：
      
      ```
      package.xml file containing meta information about the package

      CMakeLists.txt file that describes how to build the code within the package
      ```
      
      最简单的包可能具有如下所示的文件结构：
      
      ```
      my_package/
       CMakeLists.txt
       package.xml
      ```
      
      
  3. ### 工作区中的包

      单个工作区可以包含任意数量的包，每个包都在自己的文件夹中。
      你还可以在一个工作区（CMake、Python等）中拥有不同构建类型的包。
      你不能有嵌套的包。

      最佳做法是在你的工作区中有一个src文件夹，并在其中创建你的包。
      这使工作区的顶层保持“干净”。

      一个简单的工作区可能看起来像：
    
      ```
      workspace_folder/
      src/
        package_1/
            CMakeLists.txt
            package.xml

        package_2/
            setup.py
            package.xml
            resource/package_2
        ...
        package_n/
            CMakeLists.txt
            package.xml
      ```
      
      
      
- ## 先决条件

  按照上一教程中的说明操作后，你应该拥有一个ROS2工作区。
  你将在此工作区中创建你的包。
  
  
  
- ## 任务

  1. ### 创建一个包

      首先，获取你的ROS2安装。

      让我们将你在上一教程中创建的工作区dev_ws用于你的新包。`

      在运行包创建命令之前，请确保你位于src文件夹中。 
      
      ```bash 
      cd ~/dev_ws/src 
      ```
      
      在ROS2中创建新包的命令语法是：
      
      ```bash
      ros2 pkg create --build-type ament_cmake <package_name>
      ```
      
      在本教程中，你将使用可选参数--node-name在包中创建一个简单的Hello World类型可执行文件。

      在终端中输入以下命令：
      
      ```bash
      ros2 pkg create --build-type ament_cmake --node-name my_node my_package
      ```
      
      你现在将在工作区的src目录中拥有一个名为my_package的新文件夹。

      运行命令后，您的终端将返回消息：
      
      ```
      going to create a new package
      package name: my_package
      destination directory: /home/user/dev_ws/src
      package format: 3
      version: 0.0.0
      description: TODO: Package description
      maintainer: ['<name> <email>']
      licenses: ['TODO: License declaration']
      build type: ament_cmake
      dependencies: []
      node_name: my_node
      creating folder ./my_package
      creating ./my_package/package.xml
      creating source and include folder
      creating folder ./my_package/src
      creating folder ./my_package/include/my_package
      creating ./my_package/CMakeLists.txt
      creating ./my_package/src/my_node.cpp
      ```
      
      您可以看到为新包自动生成的文件。


  2. ### 构建包

      将包放入工作区特别有价值，因为你可以通过在工作区根目录中运行colcon build一次构建多个包。
      否则，你将不得不单独构建每个包。

      返回到工作区的根目录：

      ```bash
      cd ~/dev_ws
      ```
      
      现在你可以构建你的包：
      
      ```bash
      colcon build
      ```
      
      回忆上一篇教程，你的dev_ws中也有ros_tutorials包。
      你可能已经注意到，运行colcon build还构建了海龟模拟器包。
      当你的工作区中只有几个包时这很好，但是当包很多时，colcon build 可能需要很长时间。

      下次只构建my_package包，你可以运行：
      
      ```bash
      colcon build --packages-select my_package
      ```
      
      
      
  3. ### 读取并执行设置文件中的shell命令

      要使用你的新包和可执行文件，首先打开一个新终端并获取你的主要ROS2安装。

      然后，从dev_ws目录中，运行以下命令来获取你的工作区：
      
      ```bash
      . install/local_setup.bash
      ```
      
      现在你的工作区已添加到你的路径中，你将能够使用新包的可执行文件。
      
      
      
4. ### 使用包

    要运行你在包创建期间使用```--node-name```参数创建的可执行文件，请输入命令：
    
    ```bash
    ros2 run my_package my_node
    ```
    
    将向你的终端返回一条消息：
    
    ```
    hello world my_package package
    ```
    
    
    
5. ### 检查包的内容

    在```dev_ws/src/my_package```里面，你会看到```ros2 pkg```创建的自动生成的文件和文件夹：
    
    ```
    CMakeLists.txt  include  package.xml  src
    ```
    
    my_node.cpp位于src目录中。
    这是你未来所有自定义C++节点的所在。
    
    
    
  6. ### 自定义package.xml

      你可能已经在创建包后的返回消息中注意到，字段描述和许可证包含TODO注释。
      那是因为包描述和许可声明不是自动设置的，但如果你想发布你的包，这是必需的。
      可能还需要填写维护者字段。

      从dev_ws/src/my_package，使用你喜欢的文本编辑器打开package.xml：
      
      ```
       1<?xml version="1.0"?>
       2<?xml-model
       3   href="http://download.ros.org/schema/package_format3.xsd"
       4   schematypens="http://www.w3.org/2001/XMLSchema"?>
       5<package format="3">
       6 <name>my_package</name>
       7 <version>0.0.0</version>
       8 <description>TODO: Package description</description>
       9 <maintainer email="user@todo.todo">user</maintainer>
      10 <license>TODO: License declaration</license>
      11
      12 <buildtool_depend>ament_cmake</buildtool_depend>
      13
      14 <test_depend>ament_lint_auto</test_depend>
      15 <test_depend>ament_lint_common</test_depend>
      16
      17 <export>
      18   <build_type>ament_cmake</build_type>
      19 </export>
      20</package>
      ```
      
      如果没有为你自动填充，请在第7行输入你的姓名和电子邮件。
      然后，编辑第6行的描述以总结包：
      
      ```
      <description>Beginner client libraries tutorials practice package</description>
      ```
      
      然后，在第8行更新许可证。
      你可以在此处阅读有关开源许可证的更多信息。

      由于此软件包仅用于练习，因此使用任何许可证都是安全的。
      我们使用```Apache License 2.0```：
      
      ```
      <license>Apache License 2.0</license>
      ```
      
      完成编辑后不要忘记保存。

      在许可证标签下方，你将看到一些以```_depend```结尾的标签名称。
      这是你的```package.xml```将列出它对其他包的依赖关系的地方，供colcon搜索。
      my_package很简单，没有任何依赖项，但你会在即将到来的教程中看到这个空间被利用。
      
      
      ```
      你现在已经完成了！
      ```
      
      
- ## 总结
    
    你已经创建了一个包来组织你的代码并使其易于他人使用。

    你的包会自动填充必要的文件，然后你使用colcon构建它，以便你可以在本地环境中使用它的可执行文件。

- ## 下一步

    接下来，让我们为包添加一些有意义的东西。
    你将从一个简单的发布者/订阅者系统开始，你可以选择用C++或Python编写。
