---
id: 解析urdf文件
title: 解析URDF文件
---

**说明：** 本教程教你如何使用urdf解析器

**教程级别：** 初学者

1. ### 读取URDF文件

    本教程从上一个结束的地方开始。
    你应该仍然拥有my_robot.urdf文件，其中包含如下所示的机器人描述。
    
    ![alt image](http://wiki.ros.org/urdf/Tutorials/Parse%20a%20urdf%20file?action=AttachFile&do=get&target=link.png)
    
    让我们首先在我们的沙箱中创建一个依赖于urdf解析器的包：
    
    ```bash
    cd ~/galactic_ws/src
    ros2 pkg create --build-type ament_cmake testbot_description urdf
    cd testbot_description
    ```
    
    现在创建一个/urdf文件夹来存储我们刚刚创建的urdf文件：
    
    ```bash
    mkdir urdf
    cd urdf
    ```
    
    这遵循始终将机器人的URDF文件存储在名为MYROBOT_description的ROS包和名为/urdf的子文件夹中的约定。
    机器人描述包的其他标准子文件夹包括/meshes、/media和/cad，如下所示：
    
    ```
    /MYROBOT_description
    package.xml
    CMakeLists.txt
    /urdf
    /meshes
    /materials
    /cad
    ```
    
    接下来，将你的my_robot.urdf文件复制到我们刚刚创建的包和文件夹中。
    
    创建一个文件夹src/并启动你的编辑器来创建一个名为src/parser.cpp的文件：
    
    ```c++
    #include <urdf/model.h>
    #include "ros/ros.h"

    int main(int argc, char** argv){
      ros::init(argc, argv, "my_parser");
      if (argc != 2){
        ROS_ERROR("Need a urdf file as argument");
        return -1;
      }
      std::string urdf_file = argv[1];

      urdf::Model model;
      if (!model.initFile(urdf_file)){
        ROS_ERROR("Failed to parse urdf file");
        return -1;
      }
      ROS_INFO("Successfully parsed urdf file");
      return 0;
    }
    ```
    
    真正的动作发生在第12-13行。
    这里我们创建了一个解析器对象，并通过提供文件名从文件初始化它。
    如果成功解析URDF文件，则initFile方法返回 true。



    现在让我们尝试运行这段代码。 首先将以下行添加到您的 CMakeList.txt 文件中：
    
    ```cmake
    add_executable(parser src/parser.cpp)
    target_link_libraries(parser ${catkin_LIBRARIES})
    ```
    
    构建你的包，并运行它。
    
    ```bash
    ```
    
    输出应如下所示：
    
    ```
    ```
    
    现在查看代码API，了解如何开始使用你刚刚创建的URDF模型。
    正在运行的URDF模型类的一个很好的例子是RViz中的Robot::load()，在文件src/rviz/robot/robot.cpp中。


    
