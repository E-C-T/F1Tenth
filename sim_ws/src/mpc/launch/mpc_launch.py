import launch
import launch.actions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='mpc',  
            executable='mpc_node',
            name='mpc_node',
            output='screen',
        ),

    ])
