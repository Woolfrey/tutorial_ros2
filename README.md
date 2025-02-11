# 1. Publishers & Subscribers

[🔙 Back to `main`](https://github.com/Woolfrey/tutorial_ros2/blob/main/README.md#ros2-c-tutorials)

### Contents:
- [1.1 Creating a Publisher](https://github.com/Woolfrey/tutorial_ros2/blob/publisher/README.md#11-creating-a-publisher)
- [1.2 Creating a Subscriber](https://github.com/Woolfrey/tutorial_ros2/blob/publisher/README.md#12-creating-a-subscriber)


The `Publisher` and `Subcriber` protocol is analogous to the role of a news agency, or book store. A printing press will publish magazines and/or books that are sent to a store. They are made publically available for people to purchase of their own volition. The type of data being communicated is fast, frequent, and numerous.

<p align="center">
  <img src="assets/PublisherSubscriberModel.png" width="400" height="auto" alt="Publisher/Subscriber Model."/>
  <br>
  <em>Figure 1: Publishers make data publicly available for any number of subscribers.</em>
</p>

Folder structure :file_folder:
```
ros2_workspace/src/
├─ include/
    ├─  HaikuPublisher.h
    └── HaikuSubscriber.h
├─ src/
    ├─  HaikuPublisher.cpp
    ├─  HaikuSubscriber.cpp
    ├─  publisher.cpp
    └── subscriber.cpp
├── CMakeLists.txt
└── package.xml
```

## 1.1 Creating a Publisher

### 1.1.1 Create the Header File :page_facing_up:

### 1.1.2 Create the Source File :page_facing_up:

### 1.1.3 Create the Executable :gear:

### 1.1.4 Create Configuration Files :hammer_and_wrench:

### 1.1.5 Compiling & Running the Package :computer:

### 1.1.6 The Code Explained 🔎

[⬆️ Return to top.](https://github.com/Woolfrey/tutorial_ros2/blob/publisher/README.md#1-publishers--subscribers)

## 1.2 Creating a Subscriber

### 1.2.1 Create the Header File :page_facing_up:

### 1.2.2 Create the Source File :page_facing_up:

### 1.2.3 Create the Executable :gear:

### 1.2.4 Create Configuration Files :hammer_and_wrench:

### 1.2.5 Compiling & Running the Package :computer:

### 1.2.6 The Code Explained 🔎

[⬆️ Return to top.](https://github.com/Woolfrey/tutorial_ros2/blob/publisher/README.md#1-publishers--subscribers)

[🔙 Back to `main`](https://github.com/Woolfrey/tutorial_ros2/blob/main/README.md#ros2-c-tutorials)
