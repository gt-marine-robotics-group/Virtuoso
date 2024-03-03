# Docker Installation

## Images
The Docker images for Virtuoso can be found in `utils/docker`. There is currently a `Dockerfile` for the on-board computer (found in the `usv` directory) made, and we are working on creating a `Dockerfile` for normal development.

## USV Image
The USV image sets up an Docker environment containing
1. Ubuntu 20.04
2. ROS Foxy
3. The foxy branch of Virtuoso (and relevant dependencies)
4. Bag-of-Tricks (and relevant dependencies)
5. Vim (the superior text editor)

To build the container, from the directory containing the `Dockerfile`, run 
```
docker build -t virtuoso .
```

To run the container, run
```
docker run -v /dev:/dev --privileged --net=host --name virtuoso_pare -it virtuoso
```

Once the container is started, you can enter the running container from a different bash session by running
```
docker exec -it virtuoso_pare bash
```

To remove the container when done using it (necessary to rerun later), run
```
docker rm virtuoso_pare
```