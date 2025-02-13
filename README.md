# Action Servers & Action Clients

[🔙 Back to `main`](https://github.com/Woolfrey/tutorial_ros2/blob/main/README.md#ros2-c-tutorials)

This coding tutorial demonstrates how to implement a simple ROS2 action server & client.

### Contents:
- [1 Defining an Action](#1-defining-an-action)
  - [1.1 Create the Action File](#11-create-the-action-file-card_index)
  - [1.2 Edit the Configuration Files](#12-edit-the-configuration-files-hammer_and_wrench)
  - [1.3 Compile and Inspect the Action](#13-compile--inspect-the-action-computer)
- [2 Creating an Action Server](#2-creating-an-action-server)
  - [2.1 Create the Header File](#21-create-the-header-file-page_facing_up)
  - [2.2 Create the Source File](#22-create-the-source-file-page_facing_up)
  - [2.3 Create the Executable](#23-create-the-executable-gear)
  - [2.4 Edit the Configuration Files](#24-edit-the-configuration-files-hammer_and_wrench)
  - [2.5 Compiling & Running the Package](#25-compiling--running-the-package-computer)
- [3 Creating an Action Client](#3-creating-an-action-client)
  - [3.1 Create the Header File](31-create-the-header-file-page_facing_up)
  - [3.2 Create the Source File](#32-create-the-source-file-page_facing_up)
  - [3.3 Create the Executable](#33-create-the-executable-gear)
  - [3.4 Edit the Configuration Files](#34-edit-the-configuration-files-hammer_and_wrench)
  - [3.5 Compiling & Running the Package](#35-compiling--running-the-package-computer)
 
The folder structure for our package will look like this:
```
ros2_workspace/
├─ build/
├─ install/
├─ log/
└──src/
    └── tutorial_ros2/
        ├─  action/
        |   └── Haiku.action
        ├─ include/
        |   ├─  HaikuActionClient.h
        |   └── HaikuActionServer.h
        ├─ src/
        |   ├─  HaikuActionClient.cpp
        |   ├─  HaikuActionServer.cpp
        |   ├─  action_client.cpp
        |   └── action_server.cpp
        ├── CMakeLists.txt
        └── package.xml
```

## 1 Defining an Action

Communication between an action server and client is defined by a `.action` file:
```
package/Type goal
---
package/Type result
---
package/Type feedback
```
It contains three parts:
1. A goal component providing data about what the server should achieve,
2. A result component providing data on the outcome of the action, and
3. Feedback that provides regular updates as the action is executing.

The goal & response function similarly to the client & service protocol, whereas the feedback will quite literally be published like the publisher protocol.

Whilst a service should provide a near-instantaneous response, an action should be used when a request will take several seconds or more to complete.

### 1.1 Create the Action File :card_index:

Create the file `action/Haiku.action` and insert the following code:
```
int32 number_of_lines
---
std_msgs/String poem
---
int32 line_number
std_msgs/String current_line
```
- The goal for the action is to read a given `number_of_lines` of an haiku.
- The result will be a `std_msgs/String` of the entire number of lines, and
- The feedback will be the current `line_number` and the `current_line`.


[:arrow_up: Back to top.](#action-servers--action-clients)

### 1.2 Edit the Configuration Files :hammer_and_wrench:

We need to edit the configuration files so the compiler knows to turn the `.action` definition in to useable code.

First add the following to  `CMakeLists.txt`:
```
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
```
- We need `std_msgs` for the `std_msgs/String` in `Haiku.action`,
- We need the `rosidl_default_generators` for converting the `.action` in to code.

Now we need to update the `package.xml` file to match. First add these:
```
<depend>std_msgs</depend>
<depend>action_msgs</depend>
```
Here we need the additional `action_msgs` not found in the `CMakeLists.txt` file. Then add:
```
<build_depend>rosidl_default_generators</build_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

[:arrow_up: Back to top.](#action-servers--action-clients)

### 1.3 Compile & Inspect the Action :computer:

Navigate back to the root of your ROS2 workspace `cd ~/ros2_workspace` then build the package:
```
colcon build --packages-select tutorial_ros2
```
Make sure to source the changes if you haven't already told `.bashrc` to do so:
```
source ./install/setup.bash
```
Now we can check that it has compiled correctly with:
```
ros2 interface show tutorial_ros2/action/Haiku
```
which should print the following:
<p align="center">
  <img src="doc/interface_show_action.png" width="600" height="auto" alt="Screenshot of ros2 interface show for Haiku action."/>
  <br>
  <em> Figure 1: ROS2 showing the action definition for the Haiku with the goal, result, and feedback fields.</em>
</p>

[:arrow_up: Back to top.](#action-servers--action-clients)

## 2 Creating an Action Server

### 2.1 Create the Header File :page_facing_up:

[:arrow_up: Back to top.](#action-servers--action-clients)

### 2.2 Create the Source File :page_facing_up:

[:arrow_up: Back to top.](#action-servers--action-clients)

### 2.3 Create the Executable :gear:

[:arrow_up: Back to top.](#action-servers--action-clients)

### 2.4 Edit the Configuration Files :hammer_and_wrench:

[:arrow_up: Back to top.](#action-servers--action-clients)

### 2.5 Compiling & Running the Package :computer:

[:arrow_up: Back to top.](#action-servers--action-clients)

## 3 Creating an Action Client

### 3.1 Create the Header File :page_facing_up:

[:arrow_up: Back to top.](#action-servers--action-clients)

### 3.2 Create the Source File :page_facing_up:

[:arrow_up: Back to top.](#action-servers--action-clients)

### 3.3 Create the Executable :gear:

[:arrow_up: Back to top.](#action-servers--action-clients)

### 3.4 Edit the Configuration Files :hammer_and_wrench:

[:arrow_up: Back to top.](#action-servers--action-clients)

### 3.5 Compiling & Running the Package :computer:

[:arrow_up: Back to top.](#action-servers--action-clients)

[🔙 Back to `main`](https://github.com/Woolfrey/tutorial_ros2/blob/main/README.md#ros2-c-tutorials)
