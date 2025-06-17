<h1 align="center">
ROS 2 Node
</h1>
</br>
<p>A node is a participant in the ROS 2 graph, which uses a <a href="https://docs.ros.org/en/humble/Concepts/Basic/About-Client-Libraries.html">client library</a> to communicate with other nodes. Nodes can communicate with other nodes within the same process, in a different process, or on a different machine. Nodes are typically the unit of computation in a ROS graph; each node should do one logical thing.</p>
<p>Nodes can <a href="https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html">publish</a> to named topics to deliver data to other nodes, or <a href="https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html">subscribe</a> to named topics to get data from other nodes. They can also act as a <a href="https://docs.ros.org/en/humble/Concepts/Basic/About-Services.html">service client</a> to have another node perform a computation on their behalf, or as a <a href="https://docs.ros.org/en/humble/Concepts/Basic/About-Services.html">service server</a> to provide functionality to other nodes. For long-running computations, a node can act as an <a href="https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html">action client</a> to perform it, or as an <a href="https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html">action server</a> to have another node perform it. Nodes can provide configurable <a href="https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html">parameters</a> to change behavior during run-time.</p>
<p>Connections between nodes are established through a distributed <a href="https://docs.ros.org/en/humble/Concepts/Basic/About-Discovery.html">discovery</a> process.</p>
<p>Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.</p>
<img src="https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif">
<p>A full robotic system is comprised of many nodes working in concert. In ROS 2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.</p>
<h2 id="ros-2-node-commands"><a class="header" href="#ros-2-node-commands">ROS 2 Node Commands:</a></h2>
<h3 id="ros2-run"><a class="header" href="#ros2-run">ros2 run</a></h3>
<p>The command <code>ros2 run</code> launches an executable from a package.</p>
<pre><code class="language-sh">ros2 run &lt;package_name&gt; &lt;executable_name&gt;
</code></pre>
<p>To run turtlesim, open a new terminal, and enter the following command:</p>
<pre><code class="language-sh">ros2 run turtlesim turtlesim_node
</code></pre>
<p>The turtlesim window will open, where the package name is <code>turtlesim</code> and the executable name is <code>turtlesim_node</code>.</p>
<p>We still don’t know the node name, however. You can find node names by using <code>ros2 node list</code></p>
<h3 id="ros2-node-list"><a class="header" href="#ros2-node-list">ros2 node list</a></h3>
<p><code>ros2 node list</code> will show you the names of all running nodes. This is especially useful when you want to interact with a node, or when you have a system running many nodes and need to keep track of them.</p>
<p>Open a new terminal while turtlesim is still running in the other one, and enter the following command:</p>
<pre><code class="language-sh">ros2 node list
</code></pre>
<p>The terminal will return the node name:</p>
<pre><code class="language-sh">/turtlesim
</code></pre>
<p>Open another new terminal and start the teleop node with the command:</p>
<pre><code class="language-sh">ros2 run turtlesim turtle_teleop_key
</code></pre>
<p>Here, we are referring to the <code>turtlesim</code> package again, but this time we target the executable named <code>turtle_teleop_key</code>.</p>
<p>Return to the terminal where you ran <code>ros2 node list</code> and run it again. You will now see the names of two active nodes:</p>
<pre><code class="language-sh">/turtlesim
/teleop_turtle
</code></pre>
<h3 id="ros2-node-info"><a class="header" href="#ros2-node-info">ros2 node info</a></h3>
<p>Now that you know the names of your nodes, you can access more information about them with:</p>
<pre><code class="language-sh">ros2 node info &lt;node_name&gt;
</code></pre>
<p>To examine your latest node, <code>my_turtle</code>, run the following command:</p>
<pre><code class="language-sh">ros2 node info /my_turtle
</code></pre>
<p><code>ros2 node info</code> returns a list of subscribers, publishers, services, and actions. i.e. the ROS graph connections that interact with that node. The output should look like this:</p>
<pre><code class="language-sh">/my_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
</code></pre>
<p>Now try running the same command on the <code>/teleop_turtle</code> node, and see how its connections differ from <code>my_turtle</code>.</p>
<h2 id="references"><a class="header" href="#references">References:</a></h2>
<ul>
<li>
<p><a href="https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html">About Nodes</a></p>
</li>
<li>
<p><a href="https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html">Understanding ROS 2 Nodes</a></p>
</li>
</ul>
</br>
<hr />

<h1 align="left">
<a href="Package.md">⬅️</a>
</h1>
<h1 align="right">
<a href="Launch.md">➡️</a>
</h1>