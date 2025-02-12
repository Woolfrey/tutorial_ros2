# 2. Service & Client

[🔙 Back to `main`](https://github.com/Woolfrey/tutorial_ros2/blob/main/README.md#ros2-c-tutorials)

This coding tutorial demonstrates how to implement a simple ROS2 service in C++, and a client.

### Contents:

- 2.2 Creating a Service
  - 2.2.1 Create the Header File
  - 2.2.2 Create the Source File
  - 2.2.3 Create the Executable
  - 2.2.4 Edit the Configuration File
  - 2.2.5 Compiling & Running the Package
- 2.3 Creating a Client
  - 2.3.1 Create the Header File
  - 2.3.2 Create the Source File
  - 2.3.3 Create the Executable
  - 2.3.4 Edit the Configuration File
  - 2.3.5 Compiling & Running the Package
 
The folder structure for our package will look like this:
```
ros2_workspace/
├─ build/
├─ install/
├─ log/
├─ src/
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

```
package/Type request
---
package/Type response
```

In `srv/Haiku.srv' we will request the line number, and in return the server will return a string:
```
int64 line_number
---
std_msgs/String line
```

Modify the `CMakeLists.txt` file:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
                           "srv/Haiku.srv")
```

Modify the `package.xml` file:
```
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

Navigate to the root:
```
cd ~/ros2_workspace
colcon build --packages-select tutorial_ros2
```
Source:
```
source ./install.setup.bash
```
Check:
```
ros2 interface show tutorial_ros2/srv/Haiku
```
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
