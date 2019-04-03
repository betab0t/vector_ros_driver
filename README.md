# vector_ros_driver
This repository contains a ROS package to control physical Anki Vector home robot. For more information please visit [vector_ros](https://github.com/betab0t/vector_ros) main repository.

# Setup
## Docker Image
It's highly recommended to use the supplied Dockerfile instead of installing directly on your machine mainly because of the tricky setup required to run Python 3 properly on ROS. If you wish to do this setup by yourself then [I wrote a blog post explaining how](https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674) that you can use, else follow these instructions:
1. Install [Docker](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-16-04) and [docker-compose](https://docs.docker.com/compose/install/) if you dont have it already installed
```sh
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update
sudo apt-get install -y docker-ce
```

2. Run using pre-built image from DockerHub, params are passed view environment variables:
```sh
sudo docker run -e ANKI_USER_EMAIL=<EMAIL> -e ANKI_USER_PASSWORD=<PASSWORD> -e VECTOR_IP=<VECTOR_IP> -e VECTOR_SERIAL=<VECTOR_SERIAL> -e VECTOR_NAME=<VECTOR_NAME> --network host -it betab0t/vector-ros-driver
```
*Notice! Use your [Anki Developer](https://developer.anki.com/) username and password*

# Topics
* `/vector/camera`  *(sensor_msgs/Image)*

Vector camera feed.

* `/lwheel_ticks` *(std_msgs/Int32)*

Cumulative encoder ticks of the left wheel. used by [diff_drive](https://github.com/merose/diff_drive) package.

* `/rwheel_ticks`  *(std_msgs/Int32)*

Cumulative encoder ticks of the right wheel. used by [diff_drive](https://github.com/merose/diff_drive) package.

* `/lwheel_rate`  *(std_msgs/Int32)*

Left wheel rotation rate. used by [diff_drive](https://github.com/merose/diff_drive) package.

* `/rwheel_rate`  *(std_msgs/Int32)*

Right wheel rotation rate. used by [diff_drive](https://github.com/merose/diff_drive) package.

# Services

* `/vector/battery_state`

* `/vector/set_head_angle`

* `/vector/set_lift_height`

* `/vector/anim_list`

* `/vector/say_text`

# Actions

* `/vector/play_animation`

Play animation by name.

# Examples
## View single image from camera
```sh
beta_b0t@home:~$ rosrun image_view image_saver image:=/vector/camera
[ INFO] [1550425113.646567813]: Saved image left0000.jpg
[ INFO] [1550425113.752592532]: Saved image left0001.jpg
[ INFO] [1550425113.848999553]: Saved image left0002.jpg
...
(Ctrl+C)
...
beta_b0t@home:~$ eog left0000.jpg
```

## Set head angle
```sh
beta_b0t@home:~$ rosservice call /vector/set_head_angle "deg: 45.0"
```

## Say text
```sh
beta_b0t@home:~$ rosservice call /vector/say_text "text: 'hello world'"
```

## Play animation 
```sh
beta_b0t@home:~$ rostopic pub /vector/play_animation/goal vector_ros_driver/PlayAnimationActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  anim: 'anim_turn_left_01'"
```

# FAQ
- **[How do i find Vector's IP address?](https://developer.anki.com/vector/docs/troubleshooting.html#can-t-find-vector-s-ip-address)**

- **[How do i find Vector's name?](https://developer.anki.com/vector/docs/troubleshooting.html#can-t-find-robot-name)**

- **[How do i find Vector's serial number?](https://developer.anki.com/vector/docs/troubleshooting.html#can-t-find-serial-number)**

- **Why isn't this XX from Vector SDK supported?** Well, I didn't wrap all the functions from the SDK - only the main ones as i see it. Yet, if you found a missing function that you need/would like to see as part of vector_ros_driver, please consider opening a [new issue](https://github.com/betab0t/vector_ros_driver/issues/new) with your proposal.
