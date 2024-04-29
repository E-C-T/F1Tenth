import launch
import launch.actions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='pure_pursuit',  
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
        ),

    ])
