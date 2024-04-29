import launch
import launch.actions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='gap_follow',  
            executable='reactive_node',
            name='reactive_node',
            output='screen',
        ),

    ])
