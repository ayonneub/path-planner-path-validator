import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # get package directory
    pkg_create3_cbo = get_package_share_directory('path_planner_node')
    # Load parameters from YAML file
    with open(f'{pkg_create3_cbo}/config/planner.yaml', 'r') as f:
        planner_config = yaml.safe_load(f)
    
    # Create the node for the path planner
    planner_node = Node(
        package='path_planner_node',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[
            planner_config
        ]
    )
    # Create the node for the path validator
    validator_node = Node(
        package='path_planner_node',
        executable='path_validator',
        name='path_validator',
        output='screen',
        parameters=[
            planner_config
        ]
    )
    ld = LaunchDescription()
    ld.add_action(planner_node)
    ld.add_action(validator_node)
    return ld