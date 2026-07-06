from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # env and path setup
    share_dir = get_package_share_directory("vfh_algo_plan")
    rviz_config_file = os.path.join(share_dir, "rviz/default_view.rviz")
    nav_params_file = os.path.join(share_dir, "nav_params/local_costmap_params.yaml")

    param_substitutions = {
        'use_sim_time': LaunchConfiguration("use_sim_time"),
        'autostart': LaunchConfiguration("autostart")
    }

    # launch args
    declare_use_sim_time_cmd = DeclareLaunchArgument(
       'use_sim_time',
        default_value='True',
        description='Use simulator clock if true'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the lifecycle node'
    )

    # override some certain param field values
    configured_params = RewrittenYaml(
        source_file=nav_params_file,
        root_key="",
        param_rewrites=param_substitutions,
        convert_types=True)


    # rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        output='screen'    
    )

    # managed lifecycle node names required
    lifecycle_nodes = ["local_costmap"]
    # local costmap managed node 
    costmap_node = Node(
        package="vfh_algo_plan",
        executable="local_costmap_viewer_node",
        name="local_costmap",
        parameters=[configured_params]
    )
    # lifecycle manager node (unused for now) --> make a custom one tbh
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': LaunchConfiguration("use_sim_time"),
            'autostart': LaunchConfiguration("autostart"),
            'node_names': lifecycle_nodes
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(rviz_node)
    ld.add_action(costmap_node)

    # ld.add_action(lifecycle_manager_node)



    return ld