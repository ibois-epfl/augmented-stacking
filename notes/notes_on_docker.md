# Notes on docker installation

- to kknow the linx release you have:
´´´bash
$ cat /etc/*release*
´´´
- installation via *"Install using the convenience script"*:
https://docs.docker.com/engine/install/ubuntu/


- check version of docker
´´´bash
$ sudo sh get-docker.sh
´´´

- to test if docker is functionning let's open an image that displays a wheel and say something as our input, and stop running.
´´´bash
$ sudo docker run docker/whalesay cowsay Hello-World!
´´´
