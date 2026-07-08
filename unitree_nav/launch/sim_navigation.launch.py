"""
统一仿真导航启动文件
启动 Gazebo + Go1 + SLAM (slam_toolbox) + Nav2

使用方法:
  建图模式 (默认):
    ros2 launch unitree_nav sim_navigation.launch.py

  纯定位导航模式 (需要已有地图):
    ros2 launch unitree_nav sim_navigation.launch.py slam:=false map:=/path/to/map.yaml

  不启动 Rviz:
    ros2 launch unitree_nav sim_navigation.launch.py use_rviz:=false

注意: 需要先在另一个终端手动启动 unitree_guide2 控制器:
    ros2 run unitree_guide2 junior_ctrl
    按 2 = 站立, 按 5 = move_base 模式 (接受 cmd_vel)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ==== 参数声明 ====
    use_slam = LaunchConfiguration("slam")
    use_slam_arg = DeclareLaunchArgument(
        "slam", default_value="true", choices=["true", "false"],
        description="Run SLAM (true) or use pre-built map (false)"
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", choices=["true", "false"],
        description="Launch RViz2 for visualization"
    )

    world_file = LaunchConfiguration("world_file")
    world_file_arg = DeclareLaunchArgument(
        "world_file", default_value="test_latest.world",
        description="Gazebo world file name"
    )

    map_file = LaunchConfiguration("map_file")
    map_file_arg = DeclareLaunchArgument(
        "map_file", default_value="",
        description="Path to saved map YAML file (used when slam:=false)"
    )

    start_gazebo = LaunchConfiguration("start_gazebo")
    start_gazebo_arg = DeclareLaunchArgument(
        "start_gazebo", default_value="true", choices=["true", "false"],
        description="Start Gazebo (set to false if Gazebo is running separately on NVIDIA)"
    )

    # ==== 1. Gazebo + Go1 仿真 ====
    spawn_go1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("go1_gazebo"), "launch", "spawn_go1.launch.py"
            ])
        ]),
        condition=IfCondition(start_gazebo),
        launch_arguments={
            "world_file_name": world_file,
        }.items(),
    )

    # ==== 2. SLAM (slam_toolbox) ====
    slam_toolbox_node = Node(
        condition=IfCondition(use_slam),
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "base_frame": "base_link",
            "odom_frame": "odom",
            "map_frame": "map",
            "scan_topic": "/scan",
            "mode": "mapping",
            "map_file_name": "/tmp/go1_map",
            "map_start_pose": [0.0, 0.0, 0.0],
        }],
    )

    # ==== 3. Nav2 导航（使用仿真专用参数） ====
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": PathJoinSubstitution([
                FindPackageShare("unitree_nav"), "config", "nav2_params_sim.yaml"
            ]),
        }.items(),
    )

    # ==== 4. RViz ====
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("unitree_nav"), "config", "nav.rviz"
            ]),
        ],
    )

    # ==== 5. 提示信息 ====
    reminder = LogInfo(
        msg="\n\n========== 别忘了在另一个终端启动控制器 ==========\n"
            "  ros2 run unitree_guide2 junior_ctrl\n"
            "  按键: 2 = 站立, 5 = move_base 模式\n"
            "==================================================\n"
    )

    # ==== 组合启动 ====
    ld = LaunchDescription()
    ld.add_action(use_slam_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(world_file_arg)
    ld.add_action(map_file_arg)
    ld.add_action(start_gazebo_arg)

    # Gazebo 仿真
    ld.add_action(spawn_go1)
    # SLAM (延迟等 Gazebo 启动完成)
    ld.add_action(TimerAction(period=5.0, actions=[slam_toolbox_node]))
    # Nav2 (延迟等 SLAM 和 TF 就绪)
    ld.add_action(TimerAction(period=8.0, actions=[nav2_bringup]))
    # RViz (最后启动)
    ld.add_action(TimerAction(period=10.0, actions=[rviz_node]))
    # 提示
    ld.add_action(TimerAction(period=2.0, actions=[reminder]))

    return ld
