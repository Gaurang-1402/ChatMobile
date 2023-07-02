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
