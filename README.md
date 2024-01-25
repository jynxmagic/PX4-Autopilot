# PX4 Autopilot with direct motor control

Works in both SITL testing and on a real drone.

List of file changes required to add custom mode with direct motor control:
<ul>
  <li>msg/VehicleControlMode.msg</li>
  <ul>
    <li>Lines 17, 18 (Added custom modes)</li>
  </ul>
  <li>msg/VehicleStatus.msg</li>
  <ul>
    <li>Lines 61 (Added custom navigation state)</li>
  </ul>
  <li>src/modules/commander/Commander.cpp</li>
  <ul>
    <li>Lines 410-412, Set vehicle mode during SITL</li>
    <li>Lines 807-809, Set navigation state during SITL </li>
  </ul>
  <li>src/modules/commander/ModeUtil/custom_mode.cpp</li>
  <ul>
    <li>Lines 56, 63, 73, 85, 89: Disabled custom mode if default navigation states enabled. Custom mode also enabled for ACRO flight mode (For real drone).</li>
    <li>Lines 172-194, added various modes which will be available during custom navigation state. </li>
  </ul>
  <li>src/modules/commander/px4_custom_mode.h</li>
  <ul>
  <li>Line 54, added new custom main mode</li>
  <li>Line 176-180, changed into custom main mode if inside custom navigation state</li>
  </ul>
  <li>src/modules/ControlAllocator.hpp</li>
  <ul>
  <li>line 79, 196, 226, Added vehicle control mode dependenciy</li>
  </ul>
  <li>src/modules/ControlAllocator.cpp</li>
  <ul>
  <li>Lines 800-803, update control mode if changes</li>
  <li>Lines 843-858, send normal commands to motors if not within custom mode</li>
  <li>Lines 860-1011, generate and send custom commands to motors</li>
  </ul>
</ul>


To build the code for Gazebo, first run

```./Tools/setup/ubuntu.sh```

To launch SITL

```make px4_sitl gazebo```

To launch the custom mode, in this case the custom mode is called "ai". First, launch the drone from the ground:

```commander takeoff```

Whilst drone is in the air, you can enable custom mode by typing

```commander mode ai```

For deployment onto a real drone make for the correct target. For example, a Hexacube black v2

```make px4_fmu-v3```

Then within QGroundControl upload the generated file to the drone using a USB cable.

Finally, ensure that within QGroundControl you can change the drone to "ACCRO" mode. QGroundControl does not allow custom modes. However, you can trick QGC and use an existing mode. This code currently Hijakcs "ACCRO" mode. If the drone is changed into accro mode mid flight, it will enable the custom mode within the code. If any other mode is enabled, custom mode will turn off.
