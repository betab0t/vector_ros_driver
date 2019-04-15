# vector_ros_driver
This repository contains a ROS package to control physical Anki Vector home robot. For more information please visit [vector_ros](https://github.com/betab0t/vector_ros) main repository.

# Setup
## Docker Image
It's highly recommended to use the supplied Dockerfile instead of installing directly on your machine mainly because of the tricky setup required to run Python 3 properly on ROS. If you wish to do this setup by yourself then [I wrote a blog post explaining how](https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674) that you can use, else follow these instructions:
1. Install [Docker](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-16-04) if you dont have it already installed
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

3. You can now execute ```rostopic list``` on your host machine the verify everything works

# FAQ
- **[How do i find Vector's IP address?](https://developer.anki.com/vector/docs/troubleshooting.html#can-t-find-vector-s-ip-address)**

- **[How do i find Vector's name?](https://developer.anki.com/vector/docs/troubleshooting.html#can-t-find-robot-name)**

- **[How do i find Vector's serial number?](https://developer.anki.com/vector/docs/troubleshooting.html#can-t-find-serial-number)**
