# 2. Service & Client

[🔙 Back to `main`](https://github.com/Woolfrey/tutorial_ros2/blob/main/README.md#ros2-c-tutorials)

This coding tutorial demonstrates how to implement a simple ROS2 service in C++, and a client.

### Contents:
- 2.1 Defining a Service
  - 2.1.1 Create the Service File
  - 2.1.2 Edit the Configuration Files
  - 2.1.3 Compile and Inspect the Service
- 2.2 Creating a Service
  - 2.2.1 Create the Header File
  - 2.2.2 Create the Source File
  - 2.2.3 Create the Executable
  - 2.2.4 Edit the Configuration Files
  - 2.2.5 Compiling & Running the Package
- 2.3 Creating a Client
  - 2.3.1 Create the Header File
  - 2.3.2 Create the Source File
  - 2.3.3 Create the Executable
  - 2.3.4 Edit the Configuration Files
  - 2.3.5 Compiling & Running the Package
 
The folder structure for our package will look like this:
```
ros2_workspace/
├─ build/
├─ install/
├─ log/
└──src/
    └── tutorial_ros2/
        ├─ include/
        |   ├─  HaikuClient.h
        |   └── HaikuService.h
        ├─ src/
        |   ├─  HaikuClient.cpp
        |   ├─  HaikuService.cpp
        |   ├─  client.cpp
        |   └── service.cpp
        ├─ srv/
        |   └── Haiku.srv
        ├── CMakeLists.txt
        └── package.xml
```

## 1.1 Defining a Service

A service is declared using a `.srv` file in ROS2. This tells both the server & client what data is being sent and returned during communication. It is composed of two parts:
1. A request, which defines what data the client will send, and
2. A response, which defines what the server will return:

```
# Request
package/Type request
---
# Response
package/Type response
```
Both the request and response fields can be composed of any number of fields and data types.

### 1.1.1. Create the Service File

In `srv/Haiku.srv` we will request the line number, and in return the server will return a `std_msgs::msg::String` object:
```
int64 line_number
---
std_msgs/String line
```

### 1.1.2 Edit the Configuration Files

ROS2 will convert the `.srv` service definition in to useable code. We must give it instructions to do so in the `CMakeLists.txt` file:
```
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs)

```
We need:
- `rclcpp` for the ROS2 C++ client libraries,
- `rosidl_default_generators` to tell it how to build the `.srv` file in to useable code, and
- `std_msgs` which contains the `String` field
  
Then beneath it we add:
```
rosidl_generate_interfaces(${PROJECT_NAME} "srv/Haiku.srv"
                           DEPENDENCIES std_msgs)
```
which then tells ROS2 to compile the file we just created under the current project name.

Simultaneously we must modify the `package.xml` file to match:
```
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### 1.2.3 Compile & Inspect the Service

Navigate to the ROS2 working directory:
```
cd ~/ros2_workspace
```
then build the package:
```
colcon build --packages-select tutorial_ros2
```
Make sure you source the changes if you haven't added this to your `.bashrc` file:
```
source ./install.setup.bash
```
Now we can check the details of the newly defined service:
```
ros2 interface show tutorial_ros2/srv/Haiku
```
<p align="center">
  <img src="doc/interface_show_haiku.png" width="700" height="auto" alt="Show an interface"/>
  <br>
  <em> Figure 1: We can show the request and response fields of services in ROS2.</em>
</p>

## 1.2 Creating a Service

### 1.2.1 Create the Header File :page_facing_up:

### 1.2.2 Create the Source File :page_facing_up:

### 1.2.3 Create the Executable :gear:

### 1.2.4 Edit the Configuration Files :hammer_and_wrench:

### 1.2.5 Compiling & Running the Package :computer:

## 1.3 Creating a Service

### 1.3.1 Create the Header File :page_facing_up:

### 1.3.2 Create the Source File :page_facing_up:

### 1.3.3 Create the Executable :gear:

### 1.3.4 Edit the Configuration Files :hammer_and_wrench:

### 1.3.5 Compiling & Running the Package :computer:

[🔙 Back to `main`](https://github.com/Woolfrey/tutorial_ros2/blob/main/README.md#ros2-c-tutorials)
