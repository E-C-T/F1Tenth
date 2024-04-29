import launch
import launch.actions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='safety_node',  
            executable='safety_node',
            name='safety_node',
            output='screen',
        ),

    ])
