# rosplan_dorna

## Installation
Firstly install the ROSPlan software from https://github.com/KCL-Planning/ROSPlan.

Move into the packages folder and clone this package
```sh
cd ROSPlan/src
git clone https://github.com/christka1/rosplan_dorna
```

Compile again
```sh
cd ..
catkin build rosplan_dorna
```

Make sure that the bridge tries to connect to the correct ROS master node

## Running
Start the simulation environment and the bridge.
Source the correct workspace

Start the ROSPlan tool and sensing interface:
```sh
roslaunch rosplan_dorna rosplan_sensing.launch
```

Once that is running, start the action interface:
```sh
rosrun rosplan_dorna action_interface.py
```

To run one of the scenarios, choose scenario 1-3:
```sh
rosrun rosplan_dorna scenarios.py 3
```

1 = scanning test
2 = pick-up test
3 = sorting test (complete run-through)
