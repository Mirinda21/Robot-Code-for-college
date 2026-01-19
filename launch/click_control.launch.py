from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch:
    - (opcjonalnie) sterownik kamery v4l2_camera
    - node interfejsu: click_control_node
    """

    nodes = []

    # --- Kamera (jeśli na danej maszynie jest zainstalowany v4l2_camera) ---
    # Jeśli nie ma tej paczki, możesz zakomentować ten Node.
    #camera_node = Node(
    #    package='v4l2_camera',          # albo np. 'usb_cam' jeśli używacie innego drivera
    #    executable='v4l2_camera_node',
    #    name='camera',
    #    output='screen',
    #    parameters=[{
            # tu możesz dodać parametry kamery, np. device: '/dev/video0'
    #    }]
    #)

    #nodes.append(camera_node)

    # --- Twój node interfejsu ---
    click_control_node = Node(
        package='my_robot_interface',
        executable='click_control_node',
        name='click_control_node',
        output='screen'
    )

    nodes.append(click_control_node)

    return LaunchDescription(nodes)

