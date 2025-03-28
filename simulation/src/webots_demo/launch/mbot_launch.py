import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix
def get_ros2_nodes(*args):
    package_dir = get_package_share_directory('webots_demo')
    # mbot_description_path = os.path.join(package_dir, 'resource', 'mbot.urdf')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'mbot.urdf')).read_text()
    mbot_controller = Node(
            package='webots_ros2_driver',
            executable='driver',
            output='screen',
            additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'mbot_car'},
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': True},
                {'webots_node_name': 'mbot_car'}
            ],
            remappings=[
                ('/mbot_car/cmd_vel', '/cmd_vel')
            ]
        )
    obstacle_avoider = Node(
        package='webots_demo',
        executable='obstacle_avoider',
    )


    return [
        mbot_controller,
        obstacle_avoider,
    ]

def generate_launch_description():
    package_dir = get_package_share_directory('webots_demo')
    # world = LaunchConfiguration('world')
    # 使用 webots_ros2_driver.webots_launcher.WebotsLauncher 来启动 webots，加载 my_world.wbt
    # my_world.wbt 内有一个 mbot_car 机器人
    # 补充：如何制作 my_world.wbt，可以参考：
    # https://cyberbotics.com/doc/guide/tutorials
    # webots_world = WebotsLauncher(
    #     world=os.path.join(package_dir, 'worlds', 'my_world.wbt'),
    #     ros2_supervisor=True
    # )

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'sense6.wbt'),
        ros2_supervisor=True
    )

    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes,
        )
    )


    # 使用 webots_ros2_driver.webots_controller.WebotsController 来启动 mbot_car 机器人的控制器
    # 参数 robot_description 为 mbot.urdf，里面指定了 webots_demo.mbot_driver.MbotDriver 为机器人的控制器
    # mbot_controller = WebotsController(
    #     robot_name='mbot_car',
    #     parameters=[
    #         {'robot_description': mbot_description_path},
    #     ], 
    # )
    # 使用 webots_ros2_driver.driver 来启动 mbot_car 机器人的控制器

    # 额外启动一个避障控制器，订阅 mbot_car 发来的左右距离传感器数据，通过 cmd_vel，控制 mbot_car 机器人的运动


    return LaunchDescription([
        webots,
        webots._supervisor,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.UnregisterEventHandler(
                        event_handler=reset_handler.event_handler
                    ),
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
        reset_handler
    ]+ get_ros2_nodes())
