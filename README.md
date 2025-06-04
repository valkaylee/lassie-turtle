# lassie-traveler
Software distribution:
1. docker - control the environment. 
2. highlevel - handle video collection, data saving
3. src/lowlevel - handle can communication & handle trajectory generating. 


development procedure:
1. go to docker folder, and run `make_docker.sh`
2. run `make_traveler.sh` will create a container from the image and enter into the container automatically, and you can exit by type exit
3. once, exit, the contained will be stopped, but is still here. you can use `./start_traveler.sh` to enter the container
4. if you want to remove the container, use `./rm_travler.sh`
5. Note that, once you install some software, in the container, if you run `./rm_travler.sh`, the container will be deleted. next time, you will need to run `make_traveler.sh` again, and the installed software won't be keep.

you will first need to create a docker(in windows(wsl), or mac, or linux) in your computer and tested. 
and then you could run `./deploy_traveler.sh` to copy the folder docker, highlevel, src, into the raspberry pi

THE can communication(repo) and traveler high controller should go to lowlevel folder. 


Run procedure: 
once you setup the repo,
write a launch code in highlevel, main.py, so that you only need to run main.py, that will launch all stuff we need. 

IF you are using another raspberry pi to test, you might need to change the username, host, and remote_folder in the deploy_traveler bash and enter_traveler.sh. 

In the computer, download the foxglove studio to control the robot, link: https://foxglove.dev/studio

and open the layout with lassie-traveler.json, should allow you to control the robot on your commputer, if the computer and rasp are in the same network. 