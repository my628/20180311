# 使用URDF构建可移动机器人模型

**描述：** 了解如何在URDF中定义可移动关节

**教程级别：** 初学者


在本教程中，我们将修改我们在上一教程中制作的 R2D2 模型，使其具有可移动的关节。

在之前的模型中，所有的关节都是固定的。

现在我们将探讨其他三种重要的关节类型：连续、旋转和棱柱形。

在继续之前，请确保你已安装所有先决条件。

有关所需内容的信息，请参阅上一教程。

同样，本教程中提到的所有机器人模型都可以在urdf_tutorial包中找到。

这是带有灵活关节的新型urdf。 

你可以将其与之前的版本进行比较，以查看发生了哪些变化的所有内容，但我们只关注三个示例关节。

为了可视化和控制这个模型，运行与上一个教程相同的命令：

```bash
roslaunch urdf_tutorial display.launch model:=urdf/06-flexible.urdf
```

但是现在这也会弹出一个 GUI，允许你控制所有的值 非固定关节。

与模型玩一些，看看它是如何移动的。

然后，我们可以看看我们是如何做到这一点的。

![image alt](https://raw.githubusercontent.com/ros/urdf_tutorial/master/images/flexible.png)



1. ### 头部

    ```xml
      <joint name="head_swivel" type="continuous">
        <parent link="base_link"/>
        <child link="head"/>
        <axis xyz="0 0 1"/>
        <origin xyz="0 0 0.3"/>
      </joint>
    ```
    
    身体和头部之间的连接是一个连续的关节，这意味着它可以呈现从负无穷大到正无穷大的任何角度。
    
    轮子也是这样建模的，这样它们就可以永远在两个方向上滚动。

    我们必须添加的唯一附加信息是旋转轴，这里由 xyz 三元组指定，它指定了头部将围绕其旋转的向量。
    
    由于我们希望它绕 z 轴运行，因此我们指定向量“0 0 1”。



2. ### 抓手

    ```xml
      <joint name="left_gripper_joint" type="revolute">
        <axis xyz="0 0 1"/>
        <limit effort="1000.0" lower="0.0" upper="0.548" velocity="0.5"/>
        <origin rpy="0 0 0" xyz="0.2 0.01 0"/>
        <parent link="gripper_pole"/>
        <child link="left_gripper"/>
      </joint>
    ```
    
    左右夹爪关节都被建模为旋转关节。
    
    这意味着它们以与连续关节相同的方式旋转，但它们有严格的限制。
    
    因此，我们必须包含指定关节上限和下限（以弧度为单位）的限制标签。
    
    我们还必须指定此关节的最大速度和作用力，但实际值与我们的目的无关。



3. ### 夹持臂

    ```xml
      <joint name="gripper_extension" type="prismatic">
        <parent link="base_link"/>
        <child link="gripper_pole"/>
        <limit effort="1000.0" lower="-0.38" upper="0" velocity="0.5"/>
        <origin rpy="0 0 0" xyz="0.19 0 0.2"/>
      </joint>
    ```
    
    夹臂是一种不同类型的关节……即棱柱关节。
    
    这意味着它沿着一个轴移动，而不是围绕它移动。
    
    这种平移运动使我们的机器人模型能够伸展和缩回其抓手。

    棱柱臂的极限以与旋转关节相同的方式指定，不同之处在于单位是米，而不是弧度。



4. ### 其他类型的关节

    还有另外两种在空间中移动的关节。
    
    棱柱关节只能沿一个维度移动，而平面关节可以在平面或二维中移动。
    
    此外，浮动关节不受约束，可以在三个维度中的任何一个维度上移动。
    
    这些关节不能仅由一个数字指定，因此不包含在本教程中。



5. ### 指定姿势

    当你在 GUI 中移动滑块时，模型会在 Rviz 中移动。
    
    这是怎么做的？首先，GUI 解析 URDF 并找到所有非固定关节及其限制。
    
    然后，它使用滑块的值来发布 sensor_msgs/JointState 消息。
    
    然后由 robots_state_publisher 使用它们来计算不同部分之间的所有变换。
    
    然后使用生成的变换树在 Rviz 中显示所有形状。



6. ### 下一步

    现在你有了一个明显的功能模型，你可以添加一些物理属性，或者开始使用 xacro 来简化你的代码。
