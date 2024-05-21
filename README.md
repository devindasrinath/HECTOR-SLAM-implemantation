# Hector SLAM Implementation in C++

## Overview

This project provides a complete C++ implementation of the Hector SLAM (Simultaneous Localization and Mapping) algorithm. It uses a web socket to connect to a ROS (Robot Operating System) bridge and retrieve LiDAR data published on the `/scan` topic. The visual representation of the SLAM process is managed using the SFML graphics library.

## Features

- **Hector SLAM Algorithm:** Full implementation of the Hector SLAM algorithm in C++.
- **WebSocket Communication:** Utilizes WebSockets to connect to the ROS bridge and retrieve LiDAR data.
- **LiDAR Compatibility:** Compatible with any LiDAR sensor, with configurable sample number defined in a header file.
- **Visualization:** Real-time visualization using the SFML graphics library running in a separate thread.
- **Configurable Window Size:** The window size for visualization can be changed by modifying the `WIDTH` and `HEIGHT` parameters in a header file.
- **Occupancy Grid:** Supports the use of multiple occupancy grids to avoid local minima.
- **Easy Build and Run:** Simple build and run process using any C++ compiler, given that all required libraries are installed.

## Requirements

- **Libraries:** Ensure the following libraries are installed:
  - SFML (Simple and Fast Multimedia Library)
  - WebSocket++
  - ROS and rosbridge_server
- **Tools:** C++ compiler (e.g., g++ or clang++)

## Installation

1. **Clone the Repository:**
   ```sh
   git clone https://github.com/your-username/hector-slam-cpp.git
   cd hector-slam-cpp
2. **Install Dependencies:**
   Make sure SFML, WebSocket++, and ROS (with rosbridge_server) are installed on your system.
3. **Build the Project:**
   ```sh
   make

## Configuration

-**LiDAR Configuration:**
    Define the number of samples from your LiDAR in the appropriate header file:
    ```sh
    #define NUM_SAMPLES 360  // Example value

-**WebSocket Port:**
    The default WebSocket port is set to 9090. This can be changed in the configuration file if needed.

-**Window Size:**
    Modify the WIDTH and HEIGHT parameters in the header file to change the visualization window size:
    ```sh
    #define WIDTH 800
    #define HEIGHT 600

## Running the Code

1. **Start ROS Bridge:**
    Before running the code, start the ROS bridge to enable WebSocket communication:
    ```sh
    roslaunch rosbridge_server rosbridge_websocket.launch

2. **Run the SLAM Algorithm:**
    After building the project, run the executable:
    ```sh
    ./hector_slam

## Visualization

The SLAM process will be visualized in a window created using the SFML library. The window size and other visualization parameters can be adjusted as described in the configuration section.

## Demonstration

- YouTube Demo: [Link to YouTube video demonstrating the project](https://youtu.be/B0aX1W8MLuA?si=9Lh70ry8uCzIiK7L)
<iframe width="560" height="315" src="https://www.youtube.com/embed/B0aX1W8MLuA?si=9Lh70ry8uCzIiK7L" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Additional Resources


- References:
  - [Hector SLAM Paper](https://ieeexplore.ieee.org/abstract/document/6106777/)
  - [SFML Documentation](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://www.sfml-dev.org/documentation/2.6.1/&ved=2ahUKEwj766_vi5-GAxU8klYBHWWmBD8QFnoECBQQAQ&usg=AOvVaw32azKBGsgAc5zJiIr9-BED)
  - [WebSocket++ Documentation](https://docs.websocketpp.org/)

# License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

# Acknowledgments

- Thanks to the authors of the Hector SLAM algorithm.
- The ROS and SFML communities for their invaluable tools and support.

Feel free to contribute to this project by submitting issues or pull requests. Your feedback and contributions are greatly appreciated!
