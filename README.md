# my_gen2
ROS1 package for Choreonoid. 

## 1. Install
### Kinovarobotics/kinova-ros
```
cd ~/catkin_ws/src
git clone https://github.com/Kinovarobotics/kinova-ros.git
```

### my_gen2
```
cd ~/catkin_ws/src
git clone https://github.com/k38-suzuki/my_gen2.git
```

## 2. Build
```
cd ~/catkin_ws
catkin build -j8
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. Run
### Run (Joint Position Control)
```
roslaunch my_gen2 gen2_joint.launch
```

### Run (Cartesian Position Control)
```
roslaunch my_gen2 gen2_cartesian.launch
```
