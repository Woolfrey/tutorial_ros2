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

[:arrow_up: Back to top.](#action-servers--action-clients)

### 1.2 Edit the Configuration Files :hammer_and_wrench:

[:arrow_up: Back to top.](#action-servers--action-clients)

### 1.3 Compile & Inspect the Action :computer:

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
