import launch
import launch.actions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='wall_follow',  
            executable='wall_follow_node',
            name='wall_follow_node',
            output='screen',
        ),

    ])
