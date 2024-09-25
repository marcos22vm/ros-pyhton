# tech challenge
Candidate technical challenge


# ROS TurtleSim with Docker and Angular

This project propose a solution to connect a backen in ROS with turtlesim and frontend on Angular.  


# Requirements

## install

[Docker Engine](https://docs.docker.com/engine/install/ubuntu/) must be installed on you sistem.

[Docker Compose](https://docs.docker.com/compose/install/) must be installed on you sistem.



## configure
To allow docker to build the catkin project, you must set the maintenance user (usr email and user name) in the file "./backend/catkin_ws/src/ros_backend/package.xml".

```bash
...
...
 <maintainer email="user@mail.com">user_name</maintainer>
...
...

```

The ports 8080 (frontend) and 9090 (backend) must be available. You can modify the ports used in the following files:

    - ./docker-compose.yml
    - ./frontend/src/app/_services/api-ros-turtle-.service.ts


# Run the project

Open a terminal on the project folder and run the next command:

```bash
$ sudo docker-compose up -d

```
On the first run, Docker will be installing all upgrades and libraries necessary to compile the projects. The operation can take several minutes


