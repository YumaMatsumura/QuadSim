from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Get the launch file directory
    display_launch_path = PathJoinSubstitution(
        [FindPackageShare('quad_description_launch'), 'launch', 'display.launch.py']
    )
    gzserver_path = PathJoinSubstitution(
        [FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py']
    )
    gzclient_path = PathJoinSubstitution(
        [FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py']
    )

    # Create the launch configuration variables
    gui = LaunchConfiguration('gui')
    server = LaunchConfiguration('server')
    robot = LaunchConfiguration('robot')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pose = {
        'x': LaunchConfiguration('x_pose', default='0.0'),
        'y': LaunchConfiguration('y_pose', default='0.0'),
        'z': LaunchConfiguration('z_pose', default='0.5'),
        'R': LaunchConfiguration('roll', default='0.0'),
        'P': LaunchConfiguration('pitch', default='0.0'),
        'Y': LaunchConfiguration('yaw', default='0.0'),
    }
    declare_gui_cmd = DeclareLaunchArgument(
        'gui', default_value='true', description='Set to "false" to run headless.'
    )
    declare_server_cmd = DeclareLaunchArgument(
        'server', default_value='true', description='Set to "false" not to run gzserver.'
    )
    declare_robot_cmd = DeclareLaunchArgument(
        'robot', default_value='simple_quad', description='Robot name to start.'
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to use rviz'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )

    # Create nodes
    def create_gz_node():
        gz_arguments = []

        def add_gz_node_arg(key: str, value: str) -> list:
            gz_arguments.append(key)
            gz_arguments.append(value)

        add_gz_node_arg('-entity', robot)
        add_gz_node_arg('-x', pose['x'])
        add_gz_node_arg('-y', pose['y'])
        add_gz_node_arg('-z', pose['z'])
        add_gz_node_arg('-R', pose['R'])
        add_gz_node_arg('-P', pose['P'])
        add_gz_node_arg('-Y', pose['Y'])
        add_gz_node_arg('-topic', '/robot_description')
        node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=gz_arguments,
            output='screen',
        )
        return node

    def load_controller(controller_name: str):
        def set_to_active_cmd():
            _cmd = ['ros2 control load_controller --set-state active']
            _cmd.append(controller_name)
            return _cmd

        process = ExecuteProcess(
            cmd=set_to_active_cmd(),
            shell=True,
            output='screen',
        )
        return process

    spawn_entity = create_gz_node()
    load_joint_state_broadcaster = load_controller('joint_state_broadcaster')
    load_position_controller = load_controller('position_controller')

    return LaunchDescription(
        [
            declare_gui_cmd,
            declare_server_cmd,
            declare_robot_cmd,
            declare_use_rviz_cmd,
            declare_use_sim_time_cmd,
            # ===== display launch (RViz2) ===== #
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(display_launch_path),
                launch_arguments={
                    'robot': robot,
                    'use_rviz': use_rviz,
                    'use_ignition': 'false',
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
            # ===== Gazebo ===== #
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gzserver_path), condition=IfCondition(server)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gzclient_path), condition=IfCondition(gui)
            ),
            spawn_entity,
            # ===== Ros2 Control ===== #
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity, on_exit=[load_joint_state_broadcaster]
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster, on_exit=[load_position_controller]
                )
            ),
        ]
    )
