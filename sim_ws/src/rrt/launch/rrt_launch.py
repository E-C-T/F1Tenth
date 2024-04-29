import launch
import launch.actions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rrt',  
            executable='rrt_node',
            name='rrt_node',
            output='screen',
        ),

    ])
