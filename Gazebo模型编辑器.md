# 模型编辑器

现在我们将构建我们的简单机器人。 

我们将制作一辆轮式车辆并添加一个传感器，使我们能够让机器人跟随一个 blob（人）。

模型编辑器让我们可以直接在图形用户界面 (GUI) 中构建简单的模型。

对于更复杂的模型，你需要学习如何编写 SDF 文件，并查看有关构建机器人的教程。

但是现在，我们可以在 Gazebo GUI 中做任何事情！



## 模型编辑器用户界面

要进入模型编辑器，请单击菜单栏中的编辑并选择模型编辑器。 

或者，使用热键 Ctrl+M。 

一旦你进入模型编辑器，物理和模拟就会暂停。

模型编辑器界面看起来与主要 Gazebo UI 相似，但有一些细微的差别。

左侧面板和顶部工具栏现在仅包含用于编辑和创建模型部件的小部件。

由于模拟现在暂停，显示模拟数据的底部工具栏被隐藏。

![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/gazebo8_model_editor_ui.png)

1. 工具栏 - 包含用于编辑模型的工具
2. 调色板 - 也称为左面板。
    
    有两个用于编辑模型的选项卡。
    
4. 插入选项卡 - 添加链接和嵌套模型的工具
5. 模型选项卡 - 允许编辑模型属性和内容



### 调色板（左面板）

调色板有两个选项卡。

- **插入：** 插入选项卡是您向模型编辑器添加新零件（链接和模型）的地方。
    共有三个部分。
    
    - 简单形状：这些是可以插入以在模型中形成链接的原始几何图形。
    - 自定义形状：添加按钮允许你从模型中的链接导入自定义网格。
    
        它目前支持 COLLADA (.dae)、3D Systems (.stl)、Wavefront (.obj) 和 W3C SVG (.svg) 文件。
        
    - 模型数据库：有一个模型列表。
    
        这些可以像简单形状一样插入到模型编辑器中。
        
        一旦插入，它们就被称为嵌套模型。


- **模型：** 模型选项卡允许你设置正在构建的模型的名称和基本参数。
    
    它显示模型中的链接、关节、嵌套模型和插件的列表。
    
    可以使用链接检查器修改参数。
    
    可以使用这些方法中的任何一种打开它。
    
    1. 双击列表中的项目
    2. 双击场景中的项目
    3. 右键单击列表中的项目并选择打开链接检查器
    4. 右键单击场景中的项目并选择打开链接检查器



### 工具栏

与模拟模式一样，模型编辑器中的主工具栏包括用于与场景中的对象交互的工具（请参阅用户界面教程）。

可用的工具包括选择、平移、缩放、旋转、撤消和重做、复制和粘贴、对齐、捕捉、视图调整和关节创建。



### 限制

模型编辑器支持大多数可以通过编写 SDF 来完成的基本模型构建任务。

但是，还有一些功能尚不可用：

- 编辑嵌套模型和嵌套模型中的链接。
- 添加和编辑某些几何类型，包括平面和折线。
- 支持高度图。
- CAD 功能。



## 车辆构造

### 创建车辆

本节提供有关在模型编辑器中创建简单车辆模型的分步说明。

#### 底盘

1. 首先，我们将创建车辆底盘。
    
    在左侧面板的“插入”选项卡中，单击“框”图标一次，将光标移动到场景中的任意位置，然后再次单击以释放框。
    
    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-editor_box.png)
    
2. 接下来，调整盒子的大小，使其看起来更像汽车底盘的形状。

    我们可以通过选择位于顶部工具栏上的缩放工具来完成此操作。

    选择场景中的框，一个 RGB 颜色的标记应该出现在框上。

    红色标记代表 X 轴，绿色代表 Y，蓝色代表 Z。

    将鼠标移到红色标记上以突出显示它，然后单击并拖动以使机箱沿 X 轴变长。

    缩放底盘，使其长约 2 米。

    你可以通过查看地面上 1x1 米的网格来估计这一点。
    
    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-scale_tool.png)
    
    
3. 现在使用缩放工具展平底盘。 

    单击并向下拖动蓝色标记，使机箱大约为原始尺寸的一半。
    
    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-chassis_scale.png)


4. 我们希望降低底盘离地面更近。

    为了给出准确的测量值，我们将使用 Link Inspector。

    双击该框以显示 Inspector。

    向下滚动到“链接”选项卡的底部以找到“姿势”参数并将 Z 更改为 0.4m，然后在框外单击（不要按 Enter）。

    单击确定以保存更改并关闭检查器。
    
    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-chassis_height.png)
    
    
    
#### 前轮

1. 让我们继续讨论前轮。 

    首先从左侧面板的“插入”选项卡中插入圆柱体。
        
2. 默认方向的圆柱体不会很好地滚动。

    让我们使用链接检查器沿 X 轴旋转它。
    
    双击圆柱体，滚动到底部的姿势部分，将滚动更改为 1.5707 弧度（90 度），然后在框外单击。
    
    暂时不要关闭 Inspector。
    
    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-wheel_rotate.png)
    
    
3. 接下来，通过提供精确的尺寸来调整轮子的大小。

    转到“视觉”选项卡以查看此链接中的视觉效果列表。
    
    应该只有一个。
    
    单击可视文本标签旁边的小箭头展开可视项。
    
    向下滚动到几何部分并将半径更改为 0.3m，将长度更改为 0.25m。
    
    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-wheel_visual.png)
    
    
4. 你现在应该会在较大的圆柱体中看到一个较小的圆柱体。

    这是意料之中的，因为我们只改变了视觉几何，但没有改变碰撞。
    
    “视觉”是链接的图形表示，不会影响物理模拟。
    
    另一方面，物理引擎使用“碰撞”进行碰撞检查。
    
    要同时更新车轮的碰撞，请转到“碰撞”选项卡，展开唯一的碰撞项目，然后输入相同的几何尺寸。
    
    半径：0.3m，长度：0.25m。 单击确定以保存更改并关闭检查器。


5. 现在我们已经创建了我们的第一个轮子，我们将使用它作为模板并制作另一个。

    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-copy_tool.png)
    
    
6. 选择滚轮并单击顶部工具栏中的复制图标。
    
    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-copy_tool.png)
    

    现在让我们通过沿正 X 轴（场景中的红色标记）对齐底盘来确保车辆正确行驶。
    
    在下一步中添加车轮时，请确保它们位于沿 X 正轴延伸的车辆末端。


7. 底盘和车轮目前是自由移动的车身。

    为了限制它们的运动，我们将在每个车轮和底盘之间添加关节。
    
    单击顶部工具栏中的关节图标以显示关节创建对话框。
    
    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-joint_dialog.png)
    
    
8. “关节创建”对话框包含通常为关节指定的关节属性。

    在配置任何属性之前，系统会提示你选择关节的父链接和子链接。
    
    将鼠标移到场景中的底盘上以查看其突出显示，然后单击它以将其设置为关节的父级。

9. 将鼠标移动到左前轮； 一条线现在应该从机箱的原点延伸到鼠标的末端。

    单击滚轮将其设置为关节的子项。
    
    创建了一个新关节。
    
    默认情况下，它是一个旋转关节（如对话框中的关节类型部分所示），恰好是我们想要的关节类型。

    > 注意：你可能会发现此时更改视角很有用。
    > 这可以在上部工具栏中完成； 
    > 单击带有橙色边的立方体图标。

    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-wheel_joint.png)
    
    
10. 接下来，我们需要配置车轮的旋转轴。

    在 Joint Creation 对话框中，找到 Joint axis 部分并将轴更改为 Z (0, 0, 1)。
    
    注意轮子上的RGB关节视觉。
    
    你应该会看到一个黄色环现在出现在关节视觉的蓝色箭头上，表明它是旋转轴。
    
    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-wheel_rotation_axis.png)
    
    
11. 要将车轮与底盘对齐，我们将使用“关节创建”对话框中“对齐链接”部分中的不同对齐选项。

    首先我们将在 X 轴上对齐，所以点击 X Align Max 选项来查看对齐的结果。
    
    圆柱体应突出显示以指示其姿势已更改。
    
    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-wheel_align_x.png)
    
    
12. 在我们的示例中，我们希望将车轮定位在与底盘齐平的位置。

    要使轮子靠近，请单击 Y Align Max 选项。
    
    然而，这还不是我们想要的。
    
    单击 Y 对齐选项旁边的反向选项，将车轮的最小值（最大值的反向）与底盘的最大值对齐。
    
    请注意，反向选项应用于子链接，因为下面下拉列表中显示的默认对齐配置是子到父。
    
    如果设置了父到子配置，则反向选项将应用于父链接。
    
    按创建。
    
    ![](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-wheel_align_y_reverse.png)
    
    
13. 要将轮子放置在地面上方，请双击轮子打开链接检查器。

    我们可以使用对话框底部的姿势部分来移动滚轮。
    
    假设车轮的半径为 0.3m，继续将 Z 位置更改为 0.3m 以放置在地面上，然后按确定。


14. 对另一个前轮重复关节创建过程和轴配置，确保 a) 底盘是关节的父级，车轮是子级，b) 旋转轴设置为 Z，c) 使用 Y Align Min 选项将右轮对齐，因为它在底盘的另一侧。

    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-wheel_joints.png)
    
    
#### 脚轮

1. 要为车辆制作脚轮，请单击左侧面板上的球体按钮并将其插入场景中。

    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-caster_sphere.png)


2. 调整球体的大小，以与为前轮所做的相同的方式为其指定精确的尺寸。

    转到“视觉”选项卡以查看此链接中的视觉对象列表，展开唯一的视觉项目，向下滚动到“几何”部分并将半径更改为 0.2m。

    确保也对碰撞选项卡中的碰撞执行相同的操作。

    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-caster_resize.png)



3. 要在脚轮和底盘之间创建关节，请单击顶部工具栏中的关节图标，打开关节创建对话框。 

    将鼠标移动到场景并选择底盘作为父链接，球体作为子链接。

    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-caster_joint.png)



4. 与前轮接头不同，脚轮可以向各个方向滚动，并且没有特定的旋转轴。

    在 Gazebo 中，这是使用球形接头模拟的。

    因此，在关节类型部分下，选择球关节选项。

    你应该会看到场景中的关节视觉变化颜色以指示已设置不同的关节类型。

    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-caster_joint_ball.png)



5. 接下来，对齐脚轮，使其与底盘居中并位于后端。

    在对齐链接部分中，选择 Y 对齐中心选项以将两个链接在 Y 轴上居中，然后选择 X 对齐最小选项以移动脚轮，使其正好位于车辆的后部。

    按“创建”按钮完成关节创建过程。

    ![image alt](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/guided_b/files/ftu4-caster_align.png)



6. 最后，定位脚轮，使其正好位于地面上方。

    通过打开链接检查器并将 Z 位置设置为 0.2m 来执行此操作。
    
    
    
## 添加传感器

我们将添加到汽车上的传感器是深度摄像头传感器，它将帮助我们检测汽车前方的物体。

在本教程中，我们将从模型数据库中插入一个现有的传感器模型。



1. 转到 Palette（左侧面板）并选择 Insert 选项卡以查看 Model Database 部分中可用的模型列表。

2. 列表中的模型按它们所在的路径进行组织。
        
    如你所见，第一个列表包含本地计算机上可用的模型，如标题中的路径所示。
        
    如果你是第一次使用，你可能不会在列表中看到很多模型。

    当你从在线模型数据库下载它们时，会出现更多。

    找到路径为 http://gazebosim.org/models/ 的列表并展开它以查看在线模型数据库中可用的模型。


3. 模型按字母顺序排列。
        
    在列表中找到深度相机并点击它开始下载模型。

    这可能需要几秒钟的时间，具体取决于网络连接。


4. 下载完成后，你应该会看到深度相机模型出现在场景中。
    
    它看起来像一个小立方体。
        
    将鼠标移到场景上，然后单击汽车前面的空白区域以插入深度相机。

5. 选择顶部工具栏中的平移工具并移动深度相机，使其位于车辆前部的底盘顶部并大致位于 Y 轴的中心。


6. 接下来，将深度摄像头固定到机箱上。
    
    单击顶部工具栏中的关节图标以打开关节创建对话框。

    将鼠标移动到场景并选择机箱作为父链接，选择深度相机作为子链接。


7. 在关节类型部分下的关节创建对话框中，选择固定关节选项，然后单击创建以完成关节的创建。
    
    
    
## 添加插件

到目前为止，我们建造的车辆已配备所有物理和传感器组件。

但是，它实际上不会做太多事情，而是保持静止并在模拟中生成深度数据。

插件是通过允许模型执行传感器数据处理、路径规划和控制等计算来增强具有一定自主权的模型的好方法。

为简单起见，本教程将为我们的车辆使用现有插件。

请注意，可以创建自己的插件，但需要编写代码。请参阅插件教程。



1. 转到左侧面板并选择“模型”选项卡以查看构成你构建的汽车模型的部件。

2. 在模型插件下，你应该会看到一个添加按钮。
单击它会显示一个模型插件检查器，让你可以向模型添加新插件。

3. 首先，给插件一个名字。
    
        在插件名称字段中输入追随者。
        
        插件名称在此模型中必须是唯一的。

4. 我们将要使用的插件名为 libFollowerPlugin.so，因此在文件名字段中输入它。
    
        文件名对应于存储在本地机器上的插件库的实际文件名。
        
        它以动态链接的共享对象库的形式存在，因此命名约定和扩展名 .so（在 Linux 上）。
        
        如果你在其他操作系统上使用 Gazebo，请不要担心，因为扩展名会自动替换为正确的扩展名。

5. 追随者插件不需要任何附加参数，因此你可以将 Innerxml 字段留空。
    
        > 注意：这是一个用于演示目的的简单插件。
        > 插件通常具有相关联的各种参数，例如差速驱动插件需要指定控制左右轮的关节名称，以便它可以将车辆向正确的方向移动。
        > 在跟随器插件的情况下，它对其所连接的模型类型进行了许多假设，并尝试自动找到关节和传感器。

6. 单击“确定”以添加插件。该插件现在应该出现在左侧面板的模型插件下。




## 保存你的模型

1. 通过转到“文件”菜单并选择“另存为”来保存模型。
        
        输入模型的名称，然后单击保存。

2. 通过转到文件并选择退出模型编辑器来退出模型编辑器。
     
        Gazebo 现在应该切换回正常的模拟模式。
        
        点击播放按钮运行模拟。

3. 要测试插件是否正常工作，请在汽车前面插入一个盒子，然后看到汽车慢慢向它移动。




如果你想稍后再次编辑模型，只需右键单击它并在上下文菜单中选择编辑模型。
