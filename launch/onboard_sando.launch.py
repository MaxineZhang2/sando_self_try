"""Onboard SANDO launch: planner node, fake sim, and optional obstacle tracker for hardware or simulation."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml
from math import radians


# 将一些常见的布尔值表示形式（如字符串 "true"、"True" 或数字 1）统一转换为 Python 的布尔值 True，方便后续逻辑处理
def convert_str_to_bool(str):
    return True if (str == "true" or str == "True" or str == 1 or str == "1") else False


# ROS 2 中的一个标准函数，用于生成一个 launch 文件的描述。Launch 文件是 ROS 2 中用来启动多个节点、设置参数和配置环境的文件。
# 返回值是一个 LaunchDescription 对象，包含了所有的启动参数和节点配置。
def generate_launch_description():

    # DeclareLaunchArgument 是 ROS 2 中用来声明启动参数的类，每个参数都有一个名称（如 "x"）、默认值（如 "20.0"）和描述（如 "Initial x position of the quadrotor"）

    # initial position and yaw of the quadrotor，定义无人机的初始位置和朝向
    x_arg = DeclareLaunchArgument(
        "x", default_value="20.0", description="Initial x position of the quadrotor"
    )
    y_arg = DeclareLaunchArgument(
        "y", default_value="9.0", description="Initial y position of the quadrotor"
    )
    z_arg = DeclareLaunchArgument(
        "z", default_value="2.0", description="Initial z position of the quadrotor"
    )
    yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="180", description="Initial yaw angle of the quadrotor"    # 偏航角
    )
    # 定义 ROS 2 节点的命名空间，命名空间可以用来区分不同的机器人实例，避免节点名称冲突
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="NX01", description="Namespace of the nodes"
    )  
    # 定义是否启动障碍物跟踪器节点
    use_obstacle_tracker_arg = DeclareLaunchArgument(
        "use_obstacle_tracker",
        default_value="false",
        description="Flag to indicate whether to start the obstacle tracker node",
    )  
    # 定义用于存储数据的文件名
    data_file_arg = DeclareLaunchArgument(
        "data_file",
        default_value="",
        description="File name to store data",
    )  
    # 定义全局路径规划器的类型
    global_planner_arg = DeclareLaunchArgument(
        "global_planner", default_value="sjps", description="Global planner to use"
    ) 
    # 定义是否启用全局规划器的基准测试 
    use_benchmark_arg = DeclareLaunchArgument(
        "use_benchmark",
        default_value="false",
        description="Flag to indicate whether to use the global planner benchmark",
    ) 
    # 定义是否使用真实硬件 
    use_hardware_arg = DeclareLaunchArgument(
        "use_hardware",
        default_value="false",
        description="Flag to indicate whether to use hardware or simulation",
    )  
    # 定义是否使用板载定位（如 DLIO 里程计）代替外部定位系统（如 Vicon）
    use_onboard_localization_arg = DeclareLaunchArgument(
        "use_onboard_localization",
        default_value="false",
        description="Use onboard localization (DLIO odom) instead of Vicon",
    )
    # 定义仿真环境的类型，字符串，表示仿真环境的类型（如 gazebo 或 fake_sim），默认值为空字符串（使用配置文件中的值）
    sim_env_arg = DeclareLaunchArgument(
        "sim_env",
        default_value="",
        description="Simulation environment (gazebo, fake_sim). Empty string uses value from config file.",
    )  
    # 定义环境假设的类型
    environment_assumption_arg = DeclareLaunchArgument(
        "environment_assumption",
        default_value="",
        description="Override environment_assumption from config (static, dynamic, dynamic_worst_case). Empty string uses value from config file.",
    )
    # 定义与里程计相关的参数
    publish_odom_arg = DeclareLaunchArgument("publish_odom", default_value="true")    # 布尔值，表示是否发布里程计数据，默认值为 true
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic", default_value="visual_slam/odom"    
    )    # 表示里程计数据的主题名称，默认值为 "visual_slam/odom"
    odom_frame_id_arg = DeclareLaunchArgument("odom_frame_id", default_value="map")   # 表示里程计数据的参考坐标系，默认值为 "map"
    # 定义最大速度和避障相关的参数
    v_max_arg = DeclareLaunchArgument(
        "v_max",
        default_value="",
        description="Override v_max from config. Empty string uses value from config file.",
    )
    hover_avoidance_enabled_arg = DeclareLaunchArgument(
        "hover_avoidance_enabled",
        default_value="",
        description="Override hover_avoidance_enabled from config (true/false). Empty string uses value from config file.",
    )   # 表示是否启用悬停避障，默认值为空字符串。
    # 用于控制是否忽略其他轨迹（true 表示忽略，false 表示不忽略）
    # 如果设置为 true，机器人可能会忽略其他机器人或障碍物的轨迹，仅关注自身的轨迹规划
    # 如果设置为 false，机器人会考虑其他轨迹的影响，可能用于多机器人协作或避障
    ignore_other_trajs_arg = DeclareLaunchArgument(
        "ignore_other_trajs",
        default_value="",
        description="Override ignore_other_trajs from config (true/false). Empty string uses value from config file.",
    )
    # 用于控制是否跳过初始的偏航调整阶段（YAWING 状态）
    # 如果机器人已经面向目标方向，可以设置为 true，跳过偏航调整，直接开始移动。
    # 如果机器人需要调整方向以面向目标，可以设置为 false，确保机器人在正确的方向上开始移动
    skip_initial_yawing_arg = DeclareLaunchArgument(
        "skip_initial_yawing",
        default_value="",
        description="Skip YAWING state and go directly to TRAVELING (true/false). Empty string uses value from config file.",
    )

    # 定义地图大小，Need to be the same as simulartor.launch.py
    map_size_x_arg = DeclareLaunchArgument("map_size_x", default_value="20.0")
    map_size_y_arg = DeclareLaunchArgument("map_size_y", default_value="20.0")
    map_size_z_arg = DeclareLaunchArgument("map_size_z", default_value="6.0")
    odometry_topic_arg = DeclareLaunchArgument(
        "odometry_topic", default_value="visual_slam/odom"
    )

    # launch_setup 函数是 ROS 2 的一个启动配置函数，用于根据传入的参数动态生成节点配置，并启动相关的 ROS 2 节点
    # Opaque function to launch nodes
    def launch_setup(context, *args, **kwargs):
        
        # 获取启动参数的实际值
        x = LaunchConfiguration("x").perform(context)    # **LaunchConfiguration**：ROS 2 中用于获取启动参数的类；**perform(context)**：执行上下文，获取参数的实际值。
        y = LaunchConfiguration("y").perform(context)
        z = LaunchConfiguration("z").perform(context)
        yaw = LaunchConfiguration("yaw").perform(context)
        namespace = LaunchConfiguration("namespace").perform(context)
        use_obstacle_tracker = convert_str_to_bool(
            LaunchConfiguration("use_obstacle_tracker").perform(context)
        )
        data_file = LaunchConfiguration("data_file").perform(context)
        global_planner = LaunchConfiguration("global_planner").perform(context)
        use_benchmark = convert_str_to_bool(
            LaunchConfiguration("use_benchmark").perform(context)
        )
        use_hardware = convert_str_to_bool(
            LaunchConfiguration("use_hardware").perform(context)
        )
        use_onboard_localization = convert_str_to_bool(
            LaunchConfiguration("use_onboard_localization").perform(context)
        )
        sim_env_override = LaunchConfiguration("sim_env").perform(context)
        environment_assumption_override = LaunchConfiguration(
            "environment_assumption"
        ).perform(context)
        publish_odom = convert_str_to_bool(
            LaunchConfiguration("publish_odom").perform(context)
        )
        odom_topic = LaunchConfiguration("odom_topic").perform(context)
        odom_frame_id = LaunchConfiguration("odom_frame_id").perform(context)
        v_max_override = LaunchConfiguration("v_max").perform(context)
        hover_avoidance_enabled_override = LaunchConfiguration(
            "hover_avoidance_enabled"
        ).perform(context)
        ignore_other_trajs_override = LaunchConfiguration("ignore_other_trajs").perform(
            context
        )
        skip_initial_yawing_override = LaunchConfiguration(
            "skip_initial_yawing"
        ).perform(context)
        # 设置基础坐标系 ID（base_frame_id）
        base_frame_id = namespace + "/base_link"
        map_size_x = float(LaunchConfiguration("map_size_x").perform(context))
        map_size_y = float(LaunchConfiguration("map_size_y").perform(context))
        map_size_z = float(LaunchConfiguration("map_size_z").perform(context))
        odometry_topic = LaunchConfiguration("odometry_topic").perform(context)

        # The path to the urdf file
        urdf_path = PathJoinSubstitution(    # **PathJoinSubstitution**：拼接路径
            [FindPackageShare("sando"), "urdf", "quadrotor.urdf.xacro"]    # **FindPackageShare**：查找 ROS 2 包的共享目录
        )
        if use_hardware:
            parameters_path = os.path.join(
                get_package_share_directory("sando"),
                "config",
                "sando_hw_quadrotor.yaml",
            )
        else:
            parameters_path = os.path.join(
                get_package_share_directory("sando"), "config", "sando.yaml"
            )

        # Get the dict of parameters from the yaml file
        with open(parameters_path, "r") as file:
            parameters = yaml.safe_load(file)

        # Extract specific node parameters
        parameters = parameters["sando_node"]["ros__parameters"]

        # Override sim_env if provided (hardware configs may not have sim_env)，如果提供了仿真环境参数，则覆盖配置文件中的值。
        if sim_env_override:
            parameters["sim_env"] = sim_env_override
        elif "sim_env" not in parameters:
            parameters["sim_env"] = ""

        # Override environment_assumption if provided
        if environment_assumption_override:
            parameters["environment_assumption"] = environment_assumption_override

        # Override v_max if provided
        if v_max_override:
            parameters["v_max"] = float(v_max_override)

        # Override hover_avoidance_enabled if provided
        if hover_avoidance_enabled_override:
            parameters["hover_avoidance_enabled"] = convert_str_to_bool(
                hover_avoidance_enabled_override
            )

        # Override ignore_other_trajs if provided
        if ignore_other_trajs_override:
            parameters["ignore_other_trajs"] = convert_str_to_bool(
                ignore_other_trajs_override
            )

        # Override skip_initial_yawing if provided
        if skip_initial_yawing_override:
            parameters["skip_initial_yawing"] = convert_str_to_bool(
                skip_initial_yawing_override
            )

        # Update parameters for benchmarking，设置基准测试相关参数，包括数据文件路径和全局规划器。

        parameters["file_path"] = data_file
        parameters["use_benchmark"] = bool(use_benchmark)
        if use_benchmark:
            parameters["global_planner"] = global_planner

        # Create a Sando node，创建主节点 sando_node，加载配置参数并设置日志级别
        sando_node = Node(
            package="sando",    # 节点所属的 ROS 2 包名
            executable="sando",    # 可执行文件的名称
            name="sando_node",    # 节点的名称
            namespace=namespace,    # 节点的命名空间，通常用于区分不同的机器人实例
            output="screen",    # 将节点的输出打印到终端
            emulate_tty=True,    # 模拟终端输出，确保日志输出格式正确
            parameters=[parameters],    # 加载配置文件中的参数
            # prefix='xterm -e gdb -q -ex run --args', # gdb debugging
            arguments=["--ros-args", "--log-level", "error"],    # 设置 ROS 2 的日志级别为 error，减少不必要的日志输出
        )

        # Robot state publisher node，创建 robot_state_publisher 节点，用于发布机器人状态，使用 URDF 文件（通过 xacro 生成）来描述机器人模型
        robot_state_publisher_node = Node(
            package="robot_state_publisher",    # ROS 2 提供的标准包，用于发布机器人状态
            executable="robot_state_publisher",    # 可执行文件的名称
            name="robot_state_publisher",    # 节点的名称
            output="screen",    # 将节点的输出打印到终端
            namespace=namespace,    # 节点的命名空间，通常用于区分不同的机器人实例
            parameters=[
                {
                    "robot_description": ParameterValue(    #**ParameterValue**：用于设置参数的值，支持多种类型，包括字符串、整数、浮点数等
                        Command(    # Command：用于执行命令并获取输出结果
                            [
                                # 运行 xacro，用 urdf_path 这个模板文件，把里面所有 ${namespace} 替换成当前命名空间，把 ${d435_range_max_depth} 替换成深度相机量程。
                                "xacro ",    # 这是一个字符串，表示要调用的命令是 xacro，xacro 是 ROS 中的一个工具，用于处理 .xacro 文件（XML 宏文件），并将其转换为标准的 URDF 文件
                                urdf_path,    # 指定要处理的 .xacro 文件的位置
                                " namespace:=",    # 这是传递给 xacro 的一个参数，表示设置 namespace 的值
                                namespace,    # 这是一个变量，表示命名空间的具体值
                                " d435_range_max_depth:=",    # 这是传递给 xacro 的另一个参数，表示设置 d435_range_max_depth 的值
                                str(parameters["depth_camera_depth_max"]),    # 这是一个变量，表示深度相机的最大深度范围的具体值
                            ]
                        ),
                        value_type=str,
                    ),
                    "use_sim_time": False,    # 是否使用仿真时间，False 表示禁用仿真时间，使用系统时间
                    "frame_prefix": namespace + "/",    # 为所有 TF 帧添加命名空间前缀
                }
            ],
            arguments=["--ros-args", "--log-level", "error"],
        )

        # Spawn entity node for Gazebo，该节点用于在 Gazebo 仿真环境中生成机器人实体，从 robot_description 主题中获取机器人模型，并将其放置在指定的位置和朝向
        # Get the start position and yaw from the parameters
        yaw = str(radians(float(yaw)))
        spawn_entity_node = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_entity",
            namespace=namespace,
            parameters=[
                {
                    "use_sim_time": False,
                }
            ],
            arguments=[
                "-topic",    # 指定要使用的主题，这里是 robot_description，表示从该主题获取机器人模型（一对一对的）
                "robot_description",    
                "-entity",    # 指定要生成的实体的名称，这里使用命名空间作为实体名称，确保在多机器人场景中不会冲突
                namespace,
                "-x",
                x,
                "-y",
                y,
                "-z",
                z,
                "-Y",
                yaw,
            ],
        )

        # Create an obstacle tracker node，该节点负责跟踪障碍物，处理来自深度相机的点云数据。
        obstacle_tracker_node = Node(
            package="sando",
            executable="obstacle_tracker_node",
            namespace=namespace,
            name="obstacle_tracker_node",
            emulate_tty=True,    # 模拟终端输出，确保日志输出格式正确
            parameters=[parameters],
            # prefix='xterm -e gdb -ex run --args', # gdb debugging
            output="screen",
            remappings=[("point_cloud", "d435/depth/color/points")],    # **("point_cloud", "d435/depth/color/points")**：将点云数据的输入主题重映射为深度相机的点云输出
        )

        # Convert pose and twist (from Vicon) to ，该节点将来自 Vicon 系统的位姿（Pose）和速度（Twist）数据转换为机器人状态（State）
        pose_twist_to_state_node = Node(
            package="sando",
            executable="convert_vicon_to_state",
            name="convert_vicon_to_state",
            namespace=namespace,
            remappings=[
                ("world", "world"),  # Remap incoming PoseStamped topic，重映射世界坐标系的主题
                ("twist", "twist"),  # Remap incoming TwistStamped topic，重映射速度数据的主题
                ("state", "state"),  # Remap outgoing State topic，映射输出的状态主题
            ],
            emulate_tty=True,
            output="screen",
            # prefix='xterm -e gdb -ex run --args', # gdb debugging
            # arguments=['--ros-args', '--log-level', 'error']
        )

        # Convert odom (from DLIO) to global-frame state and pose via TF2，将来自 DLIO 的里程计数据转换为全局坐标系下的状态和位姿
        odom_to_global_state_node = Node(
            package="sando",
            executable="odom_to_global_state",
            name="odom_to_global_state",
            namespace=namespace,
            remappings=[
                ("odom", "dlio/odom_node/odom"),    # **("odom", "dlio/odom_node/odom")**：重映射里程计数据的输入主题
                ("state", "state"),    # **("state", "state")**：重映射输出的状态主题
                ("global_pose", "global_pose"),    # **("global_pose", "global_pose")**：重映射输出的全局位姿主题
            ],
            emulate_tty=True,
            output="screen",
        )

        # 该节点模拟机器人在仿真环境中的行为。它可以发布里程计数据，并与 Gazebo 交互
        # When using ground robot, we don't need to send the exact state to gazebo - the state will be taken care of by wheel controllers
        # send_state_to_gazebo = False if use_ground_robot else True
        # Create a fake sim node
        fake_sim_node = Node(
            package="sando",
            executable="fake_sim",
            name="fake_sim",
            namespace=namespace,
            emulate_tty=True,
            parameters=[
                {
                    "start_pos": [float(x), float(y), float(z)],
                    "start_yaw": float(yaw),
                    "send_state_to_gazebo": parameters["sim_env"] == "gazebo",    # 如果仿真环境是 Gazebo，则发送状态到 Gazebo，否则不发送
                    "visual_level": parameters["visual_level"],    # 设置视觉效果的级别，可能用于调试或性能优化
                    "publish_odom": publish_odom,    # 布尔值，表示是否发布里程计数据
                    "odom_topic": odom_topic,    # 字符串，表示里程计数据的主题名称
                    "odom_frame_id": odom_frame_id,    # 字符串，表示里程计数据的参考坐标系
                    "base_frame_id": base_frame_id,    # 字符串，表示机器人底盘的参考坐标系
                }
            ],
            # prefix='xterm -e gdb -q -ex run --args', # gdb debugging
            output="screen",
        )

        camera_file = os.path.join(
            get_package_share_directory("local_sensing"), "config", "camera.yaml"    # get_package_share_directory是一个 ROS 2 提供的函数，用于获取指定 ROS 2 包的共享目录路径
        )

        # 该节点负责渲染点云数据，并生成局部感知信息
        pcl_render_node = Node(
            package="local_sensing",
            executable="pcl_render_node",
            namespace=namespace,
            name="pcl_render_node",
            output="screen",
            parameters=[
                {"sensing_horizon": 5.0},    # 设置感知范围为 5.0 米，表示机器人可以感知到的最大距离
                {"sensing_rate": 30.0},   # 设置感知频率为 30.0 Hz，表示每秒钟处理 30 帧点云数据
                {"estimation_rate": 30.0},    # 设置估计频率为 30.0 Hz，表示每秒钟进行 30 次状态估计
                {"map/x_size": map_size_x},
                {"map/y_size": map_size_y},
                {"map/z_size": map_size_z},
                camera_file,
            ],
            remappings=[
                ("global_map", "/map_generator/global_cloud"),    # 重映射全局地图的主题，表示从 /map_generator/global_cloud 获取全局点云数据
                ("odometry", odometry_topic),    # 重映射里程计数据的主题，表示从 odometry_topic 获取里程计数据
                ("depth", "pcl_render_node/depth"),    # 重映射深度图的主题，表示从 pcl_render_node/depth 获取深度图数据
            ],
            # prefix='xterm -e gdb -q -ex run --args', # gdb debugging
        )

        # Return launch description，根据不同的条件动态决定需要启动的 ROS 2 节点，并将这些节点添加到 nodes_to_start 列表中，最后返回该列表！！！
        nodes_to_start = [sando_node]    # 初始化一个列表 nodes_to_start，并将主节点 sando_node 添加到列表中
        if use_hardware and use_onboard_localization:    # 检查是否启用了硬件 (use_hardware) 且启用了板载定位 (use_onboard_localization)
            nodes_to_start.append(odom_to_global_state_node)    # 将 odom_to_global_state_node 添加到 nodes_to_start 列表中，该节点负责将里程计数据转换为全局状态，通常用于硬件环境下的定位
        elif use_hardware:
            nodes_to_start.append(pose_twist_to_state_node)    # 该节点负责将位姿（Pose）和速度（Twist）数据转换为机器人状态，通常用于外部定位系统（如 Vicon 或 Motion Capture 系统）
        else:
            nodes_to_start.append(fake_sim_node)    # 该节点用于模拟机器人在仿真环境中的行为，通常用于开发和测试
            if parameters["sim_env"] == "gazebo":
                nodes_to_start.append(robot_state_publisher_node)    # 发布机器人状态（如关节状态、TF 坐标变换）的节点
                nodes_to_start.append(spawn_entity_node)    # 在 Gazebo 仿真环境中生成机器人实体的节点
            elif parameters["sim_env"] == "fake_sim":    # 表示使用自定义的仿真环境（非 Gazebo）
                nodes_to_start.append(pcl_render_node)    # 该节点负责渲染点云数据，并生成局部感知信息
        nodes_to_start.append(obstacle_tracker_node) if use_obstacle_tracker else None

        return nodes_to_start

    # Create launch description，带有后缀 _arg 的变量通常表示 Launch 参数（Launch Arguments），不带后缀的可能是普通变量
    return LaunchDescription(
        [
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            namespace_arg,
            use_obstacle_tracker_arg,
            data_file_arg,
            global_planner_arg,
            use_benchmark_arg,
            use_hardware_arg,
            use_onboard_localization_arg,
            sim_env_arg,
            environment_assumption_arg,
            publish_odom_arg,
            odom_topic_arg,
            odom_frame_id_arg,
            map_size_x_arg,
            map_size_y_arg,
            map_size_z_arg,
            odometry_topic_arg,
            v_max_arg,
            hover_avoidance_enabled_arg,
            ignore_other_trajs_arg,
            skip_initial_yawing_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
