**General description:**

There are two types of commands that you can send via serial communication (rate baud). First (normal) mode is used when everything works correctly and there is no need for detailed feedback. Normal mode's commands are initiated with uppercase letters. On the other hand the second (debug) mode's commands start with lowercase latter and give detailed feedback with many important values. 

**possible commands:**<br>

**Direct drive**
     Direct drive works by setting angle (in degrees) directly to each axis. This is the fastest way to control the robotic arm, because no complex math has to be calculated. Point where the end of the arm will appear after sending this command can be write like this: P = [a1, a2, a3, a4, a5, a6]. Where "a1" stands for axis one which is located at the bottom of the robotic arm and "a6" stands for axis six which is located at the end of the robotic arm. 

Normal mode:<br>
      **Dxxx/xxx/xxx/xxx/xxx/xxx/;**<br>
Debug mode:<br>
      **dxxx/xxx/xxx/xxx/xxx/xxx/;**<br>

**Invers kinematics drive**<br>
     Inverse kinematics drive wokrs by computing send coordinates and than controling angle of each axis. Math's being done according to integrated kinematic models with set parameters for this specific arm. Zero point of the coordinates system is located at the center of the base.  Point where the end of the arm will appear after sending this command can be write like this: 𝑃[𝑋; 𝑌; 𝑍; 𝑦; 𝑝; 𝑟].Where "X, Y, Z" stands for x, y and z coordinates and "y, p, r" stands for pitch roll and yaw of the end of the arm.

Normal mode:<br>
      **Ixxx/xxx/xxx/xxx/xxx/xxx/;**<br>
Debug mode:<br>
      **ixxx/xxx/xxx/xxx/xxx/xxx/;**<br>

**Gripper commands**<br>
This gripper can be controlled by sending percentage of opening, where 100% represents close state and 0% represents open state.

Normal mode:<br>
      **Sxxx;**<br>
Debug mode:<br>
      **sxxx;**<br>


Note that not every position is possible so there are some software limits implemented to prevent movement problems or even damage to the robotic arm itself.

For more information about the inverse kinematics, movement limits you can look true basic schematics or contact me directly. :)
