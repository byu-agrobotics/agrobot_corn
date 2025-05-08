import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='example',
            executable='talker',
            output='screen', # This lets you see print() or cout messages in the terminal
        ),
        launch_ros.actions.Node(
            package='example',
            executable='listener',
        ),
    ])