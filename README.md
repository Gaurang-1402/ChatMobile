# ChatMobile

Run commands in this order

```
ros2 launch articubot_one launch_robot.launch.py
```
```
rviz2 -d src/articubot_one/config/bot.rviz

```

```
ros2 launch articubot_one launch_sim.launch.py world:=src/articubot_one/worlds/obstacles.world
```

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 


```

```
source install/setup.sh
ros2 run rosgpt rosgpt 
```

```
source install/setup.sh
ros2 run rosgpt rosgpt_client_node 
```
```
source install/setup.sh
ros2 run rosgpt rosgptparser_mobile
```


## Credits
Simulation adapted from: https://github.com/joshnewans/articubot_one

```
@article{koubaa2023rosgpt,
  title={ROSGPT: Next-Generation Human-Robot Interaction with ChatGPT and ROS},
  author={Koubaa, Anis},
  journal={Preprints.org},
  year={2023},
  volume={2023},
  pages={2023040827},
  doi={10.20944/preprints202304.0827.v2}
}

```
I am deeply appreciative of these individuals for sharing their work to build on top of!
