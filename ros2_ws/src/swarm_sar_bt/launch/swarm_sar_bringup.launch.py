"""Bring up a decentralized search-and-rescue swarm on top of Nav2.

Reuses nav2_bringup's own cloned_multi_tb3_simulation_launch.py to spawn N
identical robots in Gazebo, each with its own full Nav2 stack in its own
namespace (exactly nav2's documented multi-robot pattern - see "Setting Up
a Fleet" in the Nav2 docs). On top of that this file starts, per robot
namespace, one mission_bt_node running sar_mission_tree.xml, plus one
global victim_ground_truth_node.

Example:
  ros2 launch swarm_sar_bt swarm_sar_bringup.launch.py

mission_bt_node's own params (config/mission_bt_params.yaml) use a bare
"/**:" wildcard root key, so - unlike nav2_multirobot_params_all.yaml - it
does not need nav2_common's RewrittenYaml(root_key=namespace) rewriting to
apply under each robot's namespace; it is handed to every robot as-is, with
only `robot_id` overridden per instance.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from nav2_common.launch import ParseMultiRobotPose


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    swarm_dir = get_package_share_directory('swarm_sar_bt')

    robots = LaunchConfiguration('robots')
    world = LaunchConfiguration('world')
    map_yaml_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    mission_params_file = LaunchConfiguration('mission_params_file')
    scenario_params_file = LaunchConfiguration('scenario_params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    bringup_delay = LaunchConfiguration('bringup_delay')

    declare_robots_cmd = DeclareLaunchArgument(
        'robots',
        # 3 ground rovers around the same victim layout as config/sar_scenario.yaml
        default_value=(
            'robot0={x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}; '
            'robot1={x: -8.0, y: 8.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}; '
            'robot2={x: 8.0, y: -8.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}'
        ),
        description='Robot namespaces and initial poses, nav2_bringup ParseMultiRobotPose format')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'world_only.model'),
        description='Full path to the Gazebo world file to load. Swap this (and the robot '
                     'model used by cloned_multi_tb3_simulation_launch.py) to move from the '
                     'TB3 stand-in to an actual ground-rover model/world.')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to the map yaml file to load')

    declare_nav2_params_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_all.yaml'),
        description="Nav2 stack params, identical for every robot (nav2's own default)")

    declare_mission_params_cmd = DeclareLaunchArgument(
        'mission_params_file',
        default_value=os.path.join(swarm_dir, 'config', 'mission_bt_params.yaml'),
        description='mission_bt_node params, identical for every robot except robot_id')

    declare_scenario_params_cmd = DeclareLaunchArgument(
        'scenario_params_file',
        default_value=os.path.join(swarm_dir, 'config', 'sar_scenario.yaml'),
        description='victim_ground_truth_node params (victim positions)')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description="Automatically bring each robot's Nav2 stack to the active state")

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RViz per robot')

    declare_bringup_delay_cmd = DeclareLaunchArgument(
        'bringup_delay', default_value='15.0',
        description='Seconds to wait for each Nav2 stack (specifically bt_navigator, whose '
                     'navigate_to_pose action server the NavigateToPose BT node blocks on '
                     'while the tree is being built) before starting mission_bt_node. A '
                     'production setup would instead gate this on a lifecycle state event.')

    start_nav2_swarm_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'cloned_multi_tb3_simulation_launch.py')),
        launch_arguments={
            'robots': robots,
            'world': world,
            'map': map_yaml_file,
            'params_file': nav2_params_file,
            'autostart': autostart,
            'use_rviz': use_rviz,
        }.items())

    start_ground_truth_cmd = Node(
        package='swarm_sar_bt',
        executable='victim_ground_truth_node',
        name='victim_ground_truth_node',
        output='screen',
        parameters=[scenario_params_file])

    bt_xml_path = os.path.join(swarm_dir, 'bt_xml', 'sar_mission_tree.xml')
    robots_list = ParseMultiRobotPose('robots').value()

    mission_bt_nodes = []
    for robot_id, robot_name in enumerate(robots_list):
        group = GroupAction([
            PushRosNamespace(robot_name),
            Node(
                package='swarm_sar_bt',
                executable='mission_bt_node',
                name='mission_bt_node',
                output='screen',
                parameters=[
                    mission_params_file,
                    {'robot_id': robot_id, 'bt_xml_filename': bt_xml_path},
                ]),
        ])
        mission_bt_nodes.append(group)

    start_mission_layer_cmd = TimerAction(period=bringup_delay, actions=mission_bt_nodes)

    ld = LaunchDescription()
    ld.add_action(declare_robots_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_nav2_params_cmd)
    ld.add_action(declare_mission_params_cmd)
    ld.add_action(declare_scenario_params_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_bringup_delay_cmd)

    ld.add_action(start_nav2_swarm_cmd)
    ld.add_action(start_ground_truth_cmd)
    ld.add_action(start_mission_layer_cmd)

    return ld
