# 在Gazebo中使用URDF

**描述：** 关于如何在 Gazebo 中生成和控制机器人的初步教程。

**教程级别：** 中级



请从github或使用aptitude下载URDF模拟教程，即ROS kinetic：

```bash
  sudo apt-get install ros-kinetic-urdf-sim-tutorial
```



1. ### 无功能的Gazebo界面

    我们可以使用 Gazebo.launch 将我们已经创建的模型生成到 Gazebo 中

    ```bash
      roslaunch urdf_sim_tutorialgazebo.launch
    ```

    这个启动文件

         - 将宏教程中的urdf加载到参数描述中（和以前一样）
         - 开启一个空的gazebo世界
         - 运行脚本以从参数中读取urdf并将其生成在Gazebo中。
         - 默认情况下，gazebo gui 也会显示，如下所示：

    注意：如果直接从 git 下载包，那么请创建一个工作区并将这两个文件夹放在```yourworkspacefolder/src```下，并使用命令 catkin_make 进行编译。



    ![image alt](http://wiki.ros.org/urdf/Tutorials/Using%20a%20URDF%20in%20Gazebo?action=AttachFile&do=get&target=Gazebo.png)



    但是，它没有做任何事情，并且缺少 ROS 使用此机器人所需的许多关键信息。

    以前我们一直使用joint_state_publisher来指定每个关节的姿势。

    然而，机器人本身应该在现实世界或gazebo中提供这些信息。

    然而，如果没有具体说明，Gazebo 不知道发布该信息。

    为了让机器人能够交互（与你和 ROS），我们需要指定两件事：插件和传输。



2. ### gazebo插件

    为了让 ROS 与 Gazebo 交互，我们必须动态链接到 ROS 库，它会告诉 Gazebo 做什么。
    
    从理论上讲，这允许其他机器人操作系统以通用方式与 Gazebo 交互。
    
    在实践中，它只是 ROS。

    为了链接 Gazebo 和 ROS，我们在 URDF 中指定了插件，就在关闭 </robot> 标签之前：
    
    ```xml
      <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
          <robotNamespace>/</robotNamespace>
        </plugin>
      </gazebo>
    ```
    
    你可以在 [https://github.com/ros/urdf_sim_tutorial/blob/master/urdf/09-publishjoints.urdf.xacro] 中看到这一点，并通过运行
    
    ```roslaunch urdf_sim_tutorialgazebo.launch model:=urdf/09-publishjoints.urdf.xacro```

    但是，这不会做任何新的事情。
    
    为此，我们需要在 URDF 之外指定更多信息。



3. ### 生成控制器

    现在我们已经链接了 ROS 和 Gazebo，我们需要指定一些我们想要在 Gazebo 中运行的 ROS 代码，我们通常称之为控制器。
    
    这些最初加载到 ROS 参数空间中。
    
    我们有一个 yaml 文件 Joints.yaml 指定了我们的第一个控制器。
    
    ```xml
    type: "joint_state_controller/JointStateController"
    publish_rate: 50
    ```
    
    该控制器位于joint_state_controller 包中，可直接从Gazebo 将机器人关节的状态发布到ROS 中。

    在 09-joints.launch 中，你可以看到我们应该如何将这个 yaml 文件加载到 r2d2_joint_state_controller 命名空间中。
    
    然后我们使用该命名空间调用 [[controller_manager]]/spawner 脚本，将其加载到 Gazebo 中。

    你可以启动它，但它仍然不完全存在。
    
    ```bash
    roslaunch urdf_sim_tutorial 09-joints.launch
    ```
    
    这将运行控制器并实际上在 /joint_states 主题上发布......但其中没有任何内容。
    
    ```
    header:
      seq: 652
      stamp:
        secs: 13
        nsecs: 331000000
      frame_id: ''
    name: []
    position: []
    velocity: []
    effort: []
    ```
    
    你还想要什么Gazebo！？ 嗯，它想知道发布信息的关节。



4. ### 传输

    对于每个非固定关节，我们需要指定一个传输，它告诉 Gazebo 如何处理关节。
    
    让我们从头部关节开始。 
    
    将以下内容添加到你的 URDF 中：
    
    ```xml
      <transmission name="head_swivel_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="$head_swivel_motor">
          <mechanicalReduction>1</mechanicalReduction>
        </actuator>
        <joint name="head_swivel">
          <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        </joint>
      </transmission>
    ```
    
    - 出于介绍的目的，只需将大部分代码视为样板。
    - 首先要注意的是关节元素。
        该名称应与之前声明的关节相匹配。
    - 在我们探索插件时，hardwareInterface 将很重要。

    你可以使用我们之前的启动配置运行此 URDF。
    
    ```bash
    roslaunch urdf_sim_tutorial 09-joints.launch model：=urdf/10-firsttransmission.urdf.xacro
    ```
    
    现在，头部在 RViz 中正确显示，因为头部关节在 joint_states 消息中列出。
    
    ```
    header:
      seq: 220
      stamp:
        secs: 4
        nsecs: 707000000
      frame_id: ''
    name: ['head_swivel']
    position: [-2.9051283156888985e-08]
    velocity: [7.575990694887896e-06]
    effort: [0.0]
    ```
    
    我们可以继续为所有非固定关节添加传输（我们会），以便正确发布所有关节。
    
    但是，生活不仅仅是看着机器人。
    
    我们想控制他们。
    
    所以，让我们在这里得到另一个控制器。



5. ### 关节控制

    这是我们要添加的下一个控制器配置。
    
    ```
    type: "position_controllers/JointPositionController"
    joint: head_swivel
    ```
    
    这指定使用来自 position_controllers 包的 JointPositionController 来控制 head_swivel 传输。
    
    请注意，该关节的 URDF 中的硬件接口与控制器类型相匹配。

    现在我们可以像之前一样使用添加的配置启动它 ```roslaunch urdf_sim_tutorial 10-head.launch```

    现在 Gazebo 订阅了一个新的主题，然后你可以通过在 ROS 中发布一个值来控制头部的位置。
    
    ```bash
    rostopic pub /r2d2_head_controller/command std_msgs/Float64 “数据：-0.707”
    ```

    当这个命令发布时，位置会立即改变为指定的值。
    
    这是因为我们没有为 urdf 中的关节指定任何限制。
    
    但是，如果我们改变关节，它会逐渐移动。
    
    ```xml
      <joint name="head_swivel" type="continuous">
        <parent link="base_link"/>
        <child link="head"/>
        <axis xyz="0 0 1"/>
        <origin xyz="0 0 ${bodylen/2}"/>
        <limit effort="30" velocity="1.0"/>
      </joint>
    ```
    
    ```bash
    roslaunch urdf_sim_tutorial 10-head.launch model：=urdf/11-limittransmission.urdf.xacro
    ```



6. ### 另一个控制器

    我们可以以类似的方式更改 Gripper 关节的 URDF。
    
    然而，我们可能希望将它们组合在一起，而不是使用自己的 ROS 主题单独控制抓手的每个关节。
    
    为此，我们只需要在 ROS 参数中指定一个不同的控制器。
    
    ```
    type: "position_controllers/JointGroupPositionController"
    joints:
      - gripper_extension
      - left_gripper_joint
      - right_gripper_joint
    ```
    
    要启动它，```roslaunch urdf_sim_tutorial 12-gripper.launch```

    有了这个，我们可以用一个浮点数组来指定位置。 
    
    打开和关闭：
    
    ```
    rostopic pub  /r2d2_gripper_controller/command std_msgs/Float64MultiArray "layout:
      dim:
      - label: ''
        size: 3
        stride: 1
      data_offset: 0
    data: [0, 0.5, 0.5]"
    ```
    
    关闭和收回：
    
    ```
    rostopic pub  /r2d2_gripper_controller/command std_msgs/Float64MultiArray "layout:
        dim:
        - label: ''
          size: 3
          stride: 1
        data_offset: 0
      data: [-0.4, 0, 0]"
    ```
    
    
    
7. ### 机器人上的轮子一圈又一圈

    为了驱动机器人四处走动，我们从车轮宏中为每个车轮指定了另一个传输。
    
    ```
    <transmission name="${prefix}_${suffix}_wheel_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <actuator name="${prefix}_${suffix}_wheel_motor">
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
      <joint name="${prefix}_${suffix}_wheel_joint">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      </joint>
    </transmission>
    ```
    
    这就像其他传输一样，除了

    - 它使用宏参数来指定名称

    - 它使用 VelocityJointInterface。

    由于轮子实际上会接触地面并因此与地面进行物理交互，因此我们还指定了一些有关轮子材料的附加信息。
    
    ```
    <gazebo reference="${prefix}_${suffix}_wheel">
      <mu1 value="200.0"/>
      <mu2 value="100.0"/>
      <kp value="10000000.0" />
      <kd value="1.0" />
      <material>Gazebo/Grey</material>
    </gazebo>
    ```
    
    有关更多详细信息，请参阅 [http://gazebosim.org/tutorials/?tut=ros_urdf] 。

    我们可以为每个单独的轮子指定控制器，
    
    但这有什么乐趣呢？ 相反，我们希望一起控制所有车轮。
    
    为此，我们将需要更多的 ROS 参数。
    
    ```
      type: "diff_drive_controller/DiffDriveController"
      publish_rate: 50

      left_wheel: ['left_front_wheel_joint', 'left_back_wheel_joint']
      right_wheel: ['right_front_wheel_joint', 'right_back_wheel_joint']

      wheel_separation: 0.44

      # Odometry covariances for the encoder output of the robot. These values should
      # be tuned to your robot's sample odometry data, but these values are a good place
      # to start
      pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.03]
      twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.03]

      # Top level frame (link) of the robot description
      base_frame_id: base_link

      # Velocity and acceleration limits for the robot
      linear:
        x:
          has_velocity_limits    : true
          max_velocity           : 0.2   # m/s
          has_acceleration_limits: true
          max_acceleration       : 0.6   # m/s^2
      angular:
        z:
          has_velocity_limits    : true
          max_velocity           : 2.0   # rad/s
          has_acceleration_limits: true
          max_acceleration       : 6.0   # rad/s^2
    ```
    
    DiffDriveController 订阅标准的 Twist cmd_vel 消息并相应地移动机器人。
    
    ```bash
    roslaunch urdf_sim_tutorial 13-diffdrive.launch
    ```
    
    除了加载上述配置外，这还会打开 RobotSteering 面板，让你可以驱动 R2D2 机器人四处走动，同时还可以观察其实际行为（在 Gazebo 中）和可视化行为（在 RViz 中）：
    
    
    
    ![image alt](http://wiki.ros.org/urdf/Tutorials/Using%20a%20URDF%20in%20Gazebo?action=AttachFile&do=get&target=DrivingInterface.png)
    
    
    
    恭喜！ 现在你正在使用 URDF 模拟机器人。
