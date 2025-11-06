<TRANSCRIPTION>
Now that we have the bring-up package and have completed some configuration, the first thing I’m going to do is start everything from the terminal. This way, you can see the different nodes, configurations, and launch files we need to start for our application. Then, we’ll be able to write the launch file for it.

It’s always easier to write a launch file once we’ve confirmed that everything works properly when started from the terminal.

So, let’s go ahead. The first thing we need to do is start the **robot_state_publisher**.

If you go back to the code in the *my_robot_description* package, you can see that in the launch file we start the **robot_state_publisher** with the `robot_description` parameter, and we use the `xacro` command on the URDF. That’s what I’m going to do now from the terminal.

You can follow along or just watch — what we do here will help you in the next video. The main goal is to write the launch file later.

I’ll run:

```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro /home/user/ros2_workspace/src/my_robot_description/urdf/my_robot.urdf.xacro)"
```

I’m using the absolute path here. You can also find it by searching in another terminal and copying the path.

If there are no typos, you should see something like “robot initialized.” Great — that’s the first step done.

---

### Step 2: Start ros2_control

Next, open a new terminal. We need to start **ros2_control**, specifically the **controller_manager**.
We can do this with:

```bash
ros2 run controller_manager ros2_control_node \
  --ros-args --params-file /home/user/ros2_workspace/src/my_robot_bringup/config/ros2_controllers.yaml
```

You should see logs like “Loading hardware interface,” “Arm initialized,” etc.

If you don’t see this, make sure that you correctly added the `ros2_control` tags in your URDF file.

At this point, we have:

1. The **robot_state_publisher**
2. The **controller_manager** with **ros2_control**
3. The hardware interface started

Now, we need to spawn each controller individually.

---

### Step 3: Spawn Controllers

We’ll use the **spawner** from `controller_manager` to start each controller.
Run:

```bash
ros2 run controller_manager spawner joint_state_broadcaster
```

You should see logs showing that it’s configured and activated.

Then do the same for the other controllers listed in your config file — for example:

```bash
ros2 run controller_manager spawner arm_controller
ros2 run controller_manager spawner gripper_controller
```

So far:

* Step 1: robot_state_publisher
* Step 2: controller_manager
* Step 3: launch controllers

---

### Step 4: Start MoveIt

Now that the robot, controllers, and hardware interface are running, we can start MoveIt.

Run:

```bash
ros2 launch my_robot_moveit_config move_group.launch.py
```

You’ll see many logs. If everything is correct, you’ll get a message like:

```
You can start planning now.
```

Great! The application is started.

---

### Step 5: Visualize in RViz

To visualize everything, we can start RViz with a configuration file:

```bash
ros2 run rviz2 rviz2 -d /home/user/ros2_workspace/src/my_robot_description/rviz/my_robot.rviz
```

You should see all the TFs — everything working properly. If something is missing, click **Add** → **Motion Planning** to enable MoveIt’s motion planning plugin.

After a few seconds, logs should appear, and you’ll be able to select the arm group and choose a pose (for example, *pose1*).

If it doesn’t plan correctly, go to **Context** and switch the planner to **OMPL**. Then plan and execute again.

You might also notice that the velocity scaling is very low — you can adjust it and plan again to see smoother motion.

You can test the gripper as well — for example, **gripper close**, **plan**, and **execute**. Again, make sure OMPL is selected.

As you can see, the motion is quite smooth — even smoother than the MoveIt demo.

---

### Recap

To summarize:

1. Start **robot_state_publisher**
2. Start **controller_manager**
3. Spawn all controllers (joint_state_broadcaster, arm_controller, gripper_controller)
4. Launch **MoveIt**
5. Start **RViz** with a configuration file to visualize everything

With this setup, your robot and all its controllers are fully running from the terminal.

</TRANSCRIPTION>