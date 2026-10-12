from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    map_name = LaunchConfiguration("map")
    fault_yaw_deg = LaunchConfiguration("fault_yaw_deg")
    fault_delay_sec = LaunchConfiguration("fault_delay_sec")
    demo_duration_sec = LaunchConfiguration("demo_duration_sec")

    return LaunchDescription(
        [

            DeclareLaunchArgument(
                "host",
                default_value="127.0.0.1",
                description="Windows CARLA server IP",
            ),

            DeclareLaunchArgument(
                "port",
                default_value="2000",
            ),

            DeclareLaunchArgument(
                "map",
                default_value="Town04",
            ),

            # 0.0 means random:
            # 2, 4, 6, 8, 10 or 12 degrees
            DeclareLaunchArgument(
                "fault_yaw_deg",
                default_value="0.0",
            ),

            DeclareLaunchArgument(
                "fault_delay_sec",
                default_value="12.0",
            ),

            DeclareLaunchArgument(
                "demo_duration_sec",
                default_value="60.0",
            ),

            LogInfo(
                msg=[
                    "\n",
                    "=" * 60,
                    "\nTAV | TOYOTA ADAS VERIFICATION - PROJECT PROTOTYPE",
                    "\nV2X-HEAL FAIL-OPERATIONAL CARLA DEMONSTRATION",
                    "\nCARLA Host: ",
                    host,
                    "\nMap: ",
                    map_name,
                    "\nFault Yaw: 0 = RANDOM",
                    "\n",
                    "=" * 60,
                ]
            ),

            Node(
                package="v2x_heal",
                executable="fault_injector",
                name="v2x_heal_fault_injector",
                output="screen",
                parameters=[
                    {
                        "fault_yaw_deg": fault_yaw_deg,
                        "fault_delay_sec": fault_delay_sec,
                    }
                ],
            ),

            Node(
                package="v2x_heal",
                executable="carla_visualizer",
                name="v2x_heal_carla_visualizer",
                output="screen",
                parameters=[
                    {
                        "host": host,
                        "port": port,
                        "map_name": map_name,
                        "demo_duration_sec": demo_duration_sec,
                    }
                ],
            ),
        ]
    )