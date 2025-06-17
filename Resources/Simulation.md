<h1 align="center">
Robotics Simulation Overview
</h1>
</br>
<p>This section is just to give you a quick overview of the simulation and visualization tools in ROS. </p>
<blockquote>
<p><strong>NOTE:</strong> This is only to give <strong>quick overview</strong> of what these terms are. There is a lot to explore and learn in each of the following sub-titles, and we strongly recommend you explore these further.</p>
</blockquote>
<h2 id="rviz"><a class="header" href="#rviz">RViz</a></h2>
<ul>
<li>Visualizing sensor information is an important part of developing and debugging controllers. </li>
<li>Rviz is a powerful 3D visualization tool in ROS that will help you do it.</li>
<li>It allows the user to view the simulated robot model, log sensor information from the robot's sensors, and replay the logged sensor information.</li>
</ul>
<img style="margin-top: 10px;" height="200px" src="https://raw.githubusercontent.com/ros-visualization/rviz/noetic-devel/images/splash.png" />
<h3 id="reference"><a class="header" href="#reference">Reference</a></h3>
<ol>
<li><a href="http://wiki.ros.org/rviz">ROS Wiki: Rviz</a></li>
<li><a href="http://gazebosim.org/tutorials?tut=drcsim_visualization&amp;cat=drcsim">Gazebo: Visualization and logging</a></li>
</ol>
</br>
<hr />
<h2 id="gazebo"><a class="header" href="#gazebo">Gazebo</a></h2>
<ul>
<li>Robot simulation is an essential tool in every roboticist's toolbox.</li>
<li>A robust physics engine, high-quality graphics, and the convenient programmatic and graphical interface make Gazebo a top choice for 3D Simulator.</li>
</ul>
<p><strong>.World</strong> File: This file is used to describe a collection of objects (such as buildings, tables, and lights), and global parameters including the sky, ambient light, and physics properties.</p>
<img style="margin-top: 10px;" height="120px" src="https://classic.gazebosim.org/assets/logos/gazebo_horz_neg_small-78655f82d6486fc939ff1da1ba6f48af952fa7194f8c04d459dae5544348e413.png" />
<h3 id="reference-1"><a class="header" href="#reference-1">Reference</a></h3>
<ol>
<li><a href="http://gazebosim.org/tutorials">Gazebo tutorials</a></li>
</ol>
</br>
<hr />
<h2 id="urdf"><a class="header" href="#urdf">URDF</a></h2>
<ul>
<li>The Unified Robot Description Format (URDF) contains a number of XML specifications for robot models, sensors, scenes, etc. </li>
<li>It describes the position of all the joints, sensors, type of joints, structure of the robot base, arm etc. </li>
</ul>
<h3 id="reference-2"><a class="header" href="#reference-2">Reference</a></h3>
<ol>
<li><a href="http://wiki.ros.org/urdf">ROS Wiki: URDF overview</a> </li>
<li><a href="http://wiki.ros.org/urdf/Tutorials">ROS Wiki: URDF Tutorials</a> </li>
</ol>
</br>
<hr />
<h2 id="xacro"><a class="header" href="#xacro">XACRO</a></h2>
<ul>
<li>Xacro (XML Macros) is a XML macro language. </li>
<li>With Xacro, you can construct shorter and more readable XML files by using macros that expand to larger XML expressions.</li>
<li>Xacro is useful when the structure of the robot is complex. So instead of describing the whole structure in an URDF file, we can divide the structure into small parts and call those macro files in the main Xacro file.</li>
<li>Xacros also make it easier to define common structures. For example, let's say the robot has 2 wheels. We just need to make macros of a cylindrical structure (wheels), call it in the main Xacro file, and then define 2 different joints using the same structure but giving different joint locations. </li>
</ul>
<h3 id="reference-3"><a class="header" href="#reference-3">Reference</a></h3>
<ol>
<li><a href="http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File">ROS Wiki: Using Xacro to Clean Up a URDF File</a></li>
<li><a href="http://wiki.ros.org/xacro">ROS Wiki: Xacro overview</a></li>
</ol>
</br>
<hr />
<h2 id="ros--gazebo"><a class="header" href="#ros--gazebo">ROS &amp; Gazebo</a></h2>
<ul>
<li>ROS and Gazebo together are a great combination to simulate how your algorithm would work in real-time scenarios. </li>
</ul>
<h3 id="gazebo-plugins"><a class="header" href="#gazebo-plugins">Gazebo Plugins</a></h3>
<ul>
<li>In addition to the transmission tags, a Gazebo plugin needs to be added to your URDF that actually parses the transmission tags and loads the appropriate hardware interfaces and controller manager. </li>
<li>Plugins basically replicate the exact architecture of the sensors in use or the control system used to control the movement of the robot. </li>
</ul>
<h3 id="reference-4"><a class="header" href="#reference-4">Reference</a></h3>
<ol>
<li><a href="http://gazebosim.org/tutorials/?tut=ros_control#Aboutros_control">Gazebo tutorials: ROS Control</a></li>
</ol>
</br>
<hr />

<h1 align="left">
<a href="Action.md">⬅️</a>
</h1>