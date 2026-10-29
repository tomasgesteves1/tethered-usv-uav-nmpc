import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('bringup')
    pkg_description = get_package_share_directory('description')
    
    # 1. Initial poses
    # USV
    boat_x, boat_y, boat_z = -0.288249, 0.0, 0.2  # base_link origin = system CG (0.288 m aft of the old hull origin)
    # UAV, resting on the landing pad
    drone_world_x, drone_world_z = boat_x, 1.65  # centre of the landing pad, above the tether winch

    # PX4 paths
    px4_dir = os.environ.get('PX4_DIR', os.path.expanduser('~/PX4-Autopilot'))
    px4_build_dir = os.path.join(px4_dir, 'build', 'px4_sitl_default')

    # Gazebo resource paths
    local_models_dir = os.path.join(pkg_description, 'models')
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_resource_path = f"{local_models_dir}:{gz_resource_path}"

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_gz_resource_path
    )

    # Headless argument
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without its graphical interface'
    )

    # Tether argument
    use_tether_arg = DeclareLaunchArgument(
        'use_tether',
        default_value='true',
        description='Enable the simulation of the tether'
    )

    # 1. World (starts paused)
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'world.launch.py')
        ),
        launch_arguments={'world': 'wamv_world', 'paused': 'true', 'headless': LaunchConfiguration('headless')}.items()
    )

    # 2. Spawn the vehicles
    spawn_boat = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'wamv_world', '-file', os.path.join(local_models_dir, 'wamv', 'model.sdf'),
                   '-name', 'wamv', '-x', str(boat_x), '-y', str(boat_y), '-z', str(boat_z)],
        output='screen'
    )

    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'wamv_world', '-file', os.path.join(local_models_dir, 'x500', 'model.sdf'),
                   '-name', 'x500', '-x', str(drone_world_x), '-y', str(boat_y), '-z', str(drone_world_z)],
        output='screen'
    )

    # 3. Resume the physics
    unpause_gz = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/wamv_world/control', 
             '--reqtype', 'gz.msgs.WorldControl', 
             '--reptype', 'gz.msgs.Boolean', 
             '--timeout', '2000', 
             '--req', 'pause: false'],
        output='screen'
    )

    # Unpause 2 s after the vehicles are spawned
    unpause_physics_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_drone,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[unpause_gz]
                )
            ]
        )
    )

    # 4. PX4 SITL attached to the UAV
    px4_env = os.environ.copy()
    px4_env.update({
        'PX4_GZ_WORLD': 'wamv_world', 
        'PX4_GZ_MODEL_NAME': 'x500',
        'PX4_SYS_AUTOSTART': '4001',
        'GZ_SIM_RESOURCE_PATH': new_gz_resource_path
    })

    px4_sitl_process = TimerAction(
        period=5.0, 
        actions=[
            ExecuteProcess(
                cmd=[os.path.join(px4_build_dir, 'bin', 'px4'), '-i', '1'],
                cwd=px4_build_dir,
                output='screen',
                env=px4_env
            )
        ]
    )

    # 5. Micro XRCE-DDS Agent and ROS-Gazebo bridge
    micro_xrce_agent_path = os.path.join(
        get_package_prefix('micro_xrce_vendor'), 'lib', 'micro_xrce_vendor', 'MicroXRCEAgent'
    )
    micro_ros_agent = ExecuteProcess(
        cmd=[micro_xrce_agent_path, 'udp4', '-p', '8888'],
        output='log'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/left/pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/wamv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/x500/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/wamv_world/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            '/world/wamv_world/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            '/world/wamv_world/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity',
            '/world/wamv_world/stats@ros_gz_interfaces/msg/WorldStatistics[gz.msgs.WorldStatistics',
        ],
        remappings=[
            ('/wamv/thrusters/left/thrust', '/boat/thrusters/left/thrust'),
            ('/wamv/thrusters/right/thrust', '/boat/thrusters/right/thrust'),
            ('/wamv/thrusters/left/pos', '/boat/thrusters/left/pos'),
            ('/wamv/thrusters/right/pos', '/boat/thrusters/right/pos'),
            ('/model/wamv/odometry', '/boat/ground_truth/odometry'),
            ('/model/x500/odometry', '/drone/ground_truth/odometry'),
            ('/world/wamv_world/stats', '/simulation/stats'),
        ],
        output='screen'
    )

    tether_node = Node(
        package='moordyn_tether',
        executable='moordyn_tether_node',
        name='moordyn_tether_node',
        condition=IfCondition(LaunchConfiguration('use_tether')),
        parameters=[
            os.path.join(
                get_package_share_directory('moordyn_tether'),
                'config', 'moordyn_tether.yaml'),
            {'use_sim_time': True},
        ],
        output='screen'
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'use_sim_time': True,
            'asset_uri_allowlist': ['.*']
        }]
    )

    frame_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('frame_manager'), 'launch', 'frame_manager.launch.py')
        )
    )

    mocap_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mocap_bridge'), 'launch', 'mocap_bridge.launch.py')
        )
    )

    # Kill the PX4 processes left behind on shutdown
    force_kill_zombies = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                ExecuteProcess(cmd=['pkill', '-9', '-f', 'px4'], name='kill_px4')
            ]
        )
    )

    return LaunchDescription([
        headless_arg,
        use_tether_arg,
        set_gz_resource_path,
        world_launch,
        spawn_boat,
        spawn_drone,
        unpause_physics_handler,
        px4_sitl_process,
        micro_ros_agent,
        bridge,
        foxglove_bridge,
        frame_manager_launch,
        mocap_bridge_launch,
        tether_node,
        force_kill_zombies
    ])

