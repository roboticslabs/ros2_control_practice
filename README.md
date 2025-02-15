# ros2_control practice
Use the main branch as the diffbot ros2_control demo startup package

## Quick Start

* Create a ros workspace
    ```bash   
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
* Download the package
    ```bash    
    git clone -b main https://github.com/roboticslabs/ros2_control_practice.git
    ```
* Compile the package
    ```bash
    cd ~/ros2_ws
    sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
    colcon build
    ```
* Try it
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch diffbot_description view_robot.launch.py
    ```
