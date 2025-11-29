AZcrab is a prototype of a rescue robot developed for participation in the RoboCup Rescue Maze competition. Creating robots capable of locating and rescuing people in hard-to-reach areas is a relevant challenge in modern robotics. The robot is designed to autonomously navigate an unknown maze with obstacles, detect victims, and read color markers on the walls.

Official competition website: [RoboCup Junior Rescue Maze](https://junior.robocup.org/rcj-rescue-maze/)

Project concept
The robot’s concept can be summarized in three key points:

Distributed control architecture — two microcontrollers are used to separate tasks: an ESP32‑S3 handles low‑level motion control, while a Raspberry Pi 5 performs high‑level data processing and navigation algorithms.

Specialized inertial module — a GY25 IMU processes data separately to accurately determine the robot’s orientation and detect inclined surfaces (ramps).

Optimized design — a compact chassis with a quick‑release battery mount and an efficient rescue kit deployment system.

Technical specifications
Hardware

Chassis: Custom-built chassis with motor suspension

Motors: 4 × MF4010V2

Main processors:

ESP32‑S3 N16R8 — low‑level control

Raspberry Pi 5 — high‑level processing

Navigation system:

Range sensors: VL53L0X (for wall distance measurement)

Inertial module: GY25 (for rotation and slope detection)

Color sensor: APDS9960 (for wall marker reading)

Vision system: Two CSI cameras connected to the Raspberry Pi

Power supply:

Custom 4S 18650 battery pack with BMS

Charging module: IP2369 (45 W)

Software
Programming languages: C++, Python

Frameworks and tools:

PlatformIO ESP‑IDF (for ESP32‑S3)

YOLO (object detection)

OpenCV (image processing)

Picamera 2(getting an image)

Navigation algorithm: DFS (Depth‑First Search) for maze exploration

