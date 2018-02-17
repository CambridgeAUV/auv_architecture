# Monolithic Repo for AUV Architecture

To install first create a new catkin workspace
```
mkdir auv_catkin_workspace
cd auv_catkin_workspace
mkdir src
catkin_make
```
Then go into the src directory and clone this repo 
```
cd src
git clone https://github.com/CambridgeAUV/auv_architecture
```
Now your directory should look like this:
```
auv_catkin_workspace
    --build
    --devel
    --src
        --CMakeLists.txt
        --auv_architecture
            --README.md
            --pid_controller
            .
            .
            .
```
Go back to the top directory `auv_catkin_workspace` and build the packages
```
cd auv_catkin_workspace
catkin_make
```
To run a package use a command such as
```
rosrun sonar_cv script.py
```
If you get an error perhaps you haven't sourced the setup file
```
cd auv_catkin_workspace
source devel/setup.bash
```

