# 开始使用KDL解析器



**说明：** 本教程教你如何从URDF文件创建KDL树

**教程级别：** 初学者



1. ### 构建KDL解析器

    ```bash
    rosdep install kdl_parser
    ```
    
    这将为kdl_parser安装所有外部依赖项。
    要构建包，请运行：
    
    ```bash
    rosmake kdl_parser
    ```
    
2. ### 在你的代码中使用

    首先，将KDL解析器作为依赖项添加到package.xml（在ROS fuerte或更早版本中为manifest.xml）文件：
    
    ```
    <package>
    ...
    <build_depend package="kdl_parser" />
    ...
    <run_depend package="kdl_parser" />
    ...
    </package>
    ```
    
    要开始在C++代码中使用KDL解析器，请包含以下文件：
    
    ```c++
    #include <kdl_parser/kdl_parser.hpp>
    ```
    
    现在有不同的方法可以继续。
    你可以从urdf以各种形式构建KDL树：
   
    1. #### 从一个文件

        ```c++
        KDL::Tree my_tree;
           if (!kdl_parser::treeFromFile("filename", my_tree)){
              ROS_ERROR("Failed to construct kdl tree");
              return false;
           }
        ```
        
        要创建 PR2 URDF 文件，请运行以下命令：
        
        ```bash
        rosrun xacro xacro.py `rospack find pr2_description`/robots/pr2.urdf.xacro > pr2.urdf
        ```
        
    2. #### 从参数服务器

        ```c++
           KDL::Tree my_tree;
           ros::NodeHandle node;
           std::string robot_desc_string;
           node.param("robot_description", robot_desc_string, std::string());
           if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
              ROS_ERROR("Failed to construct kdl tree");
              return false;
           }
        ```
        
    3. #### 从XML元素

        ```c++
         KDL::Tree my_tree;
         TiXmlDocument xml_doc;
         xml_doc.Parse(xml_string.c_str());
         xml_root = xml_doc.FirstChildElement("robot");
         if (!xml_root){
            ROS_ERROR("Failed to get robot from xml document");
            return false;
         }
         if (!kdl_parser::treeFromXml(xml_root, my_tree)){
            ROS_ERROR("Failed to construct kdl tree");
            return false;
         }
        ```
        
    4. #### 从URDF模型

        ```c++
        KDL::Tree my_tree;
           urdf::Model my_model;
           if (!my_model.initXml(....)){
              ROS_ERROR("Failed to parse urdf robot model");
              return false;
           }
           if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
              ROS_ERROR("Failed to construct kdl tree");
              return false;
           }
        ```
