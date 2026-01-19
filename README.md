# my_robot_interface

Interfejs do sterowania robotem mobilnym (np. TurtleBot) z wykorzystaniem ROS 2 Jazzy.
Użytkownik klika w obraz z kamery, a robot porusza się do przodu lub do tyłu
w zależności od tego, czy kliknięty punkt jest powyżej czy poniżej środka obrazu.

## Wymagania

- ROS 2 Jazzy (Ubuntu 24.04)
- Python 3
- Paczki ROS2:
  - `rclpy`
  - `sensor_msgs`
  - `geometry_msgs`
  - `cv_bridge`
  - `image_transport`
- OpenCV dla Pythona (`python3-opencv`)
- Sterownik kamery publikujący obraz na topicu `/image_raw`
- Robot (lub symulator) subskrybujący topic `/cmd_vel` (np. TurtleBot)

## Instalacja

W katalogu `ros2_ws`:

```bash
cd ~/ros2_ws/src
git clone https://github.com/TWOJ_LOGIN/my_robot_interface.git
cd ..
colcon build --symlink-install
source install/setup.bash
