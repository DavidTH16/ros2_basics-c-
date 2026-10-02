import launch
import launch_ros.actions

def generate_launch_description():

    #nodes
    callback_function_node = launch_ros.actions.Node(
        package = "logs_test",
        executable = "logger_example",
        arguments = [],
        emulate_tty = True,
        output = "screen"
    )

    return launch.LaunchDescription([
        callback_function_node
    ])