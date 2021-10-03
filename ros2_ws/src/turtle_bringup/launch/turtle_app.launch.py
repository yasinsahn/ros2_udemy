from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtle_controller_node = Node(
        package='turtle_game',
        executable='turtle_controller_node',
        name='turtle_controller',
        parameters = [
            {'lin_vel_x_gain': 2.0},
            {'lin_vel_y_gain': 1.0},
            {'ang_vel_z_gain': 5.0},
            {'control_loop_frequency': 100}
            ]
            )

    turtle_spawner_node = Node(
        package='turtle_game',
        executable='turtle_spawner_node',
        name='turtle_spawner',
        parameters = [
            {'turtle_spawn_second': 0.8}
            ]
            )
    
    turtlesim_node=Node(
        package = 'turtlesim',
        executable = 'turtlesim_node'
        )


    ld.add_action(turtle_controller_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtlesim_node)

    return ld
