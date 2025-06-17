<h1 align="center">
ROS 2 Topic
</h1>
<h2 id="description"><a class="header" href="#description">Description:</a></h2>
<p>ROS 2 breaks complex systems down into many modular nodes. Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.</p>
<img src="https://docs.ros.org/en/humble/_images/Topic-SinglePublisherandSingleSubscriber.gif" />
<p>A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics.</p>
<img src="https://docs.ros.org/en/humble/_images/Topic-MultiplePublisherandMultipleSubscriber.gif" />
<p>Topics are one of the main ways in which data is moved between nodes and therefore between different parts of the system.</p>
<h2 id="using-ros-topics"><a class="header" href="#using-ros-topics">Using ROS Topics:</a></h2>
<p>By now you should be comfortable starting up turtlesim.</p>
<p>Open a new terminal and run:</p>
<pre><code class="language-sh">ros2 run turtlesim turtlesim_node
</code></pre>
<p>Open another terminal and run:</p>
<pre><code class="language-sh">ros2 run turtlesim turtle_teleop_key
</code></pre>
<p>Recall from the tutorial <a href="ros_nodes.html">ROS 2 Node</a> that the names of these nodes are <code>/turtlesim</code> and <code>/teleop_turtle</code> by default.</p>
<h3 id="rqt_graph"><a class="header" href="#rqt_graph">rqt_graph</a></h3>
<p>Throughout this tutorial, we will use <code>rqt_graph</code> to visualize the changing nodes and topics, as well as the connections between them.</p>
<p>The <a href="https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html">turtlesim tutorial</a> tells you how to install rqt and all its plugins, including <code>rqt_graph</code>.</p>
<p>To run rqt_graph, open a new terminal and enter the command:</p>
<pre><code class="language-sh">rqt_graph
</code></pre>
<p>You can also open rqt_graph by opening <code>rqt</code> and selecting <strong>Plugins &gt; Introspection &gt; Node Graph</strong>.</p>
<img src="https://docs.ros.org/en/humble/_images/rqt_graph.png" />
<p>You should see the above nodes and topic, as well as two actions around the periphery of the graph (let’s ignore those for now). If you hover your mouse over the topic in the center, you’ll see the color highlighting like in the image above.</p>
<p>The graph is depicting how the <code>/turtlesim</code> node and the <code>/teleop_turtle</code> node are communicating with each other over a topic. The <code>/teleop_turtle</code> node is publishing data (the keystrokes you enter to move the turtle around) to the <code>/turtle1/cmd_vel</code> topic, and the <code>/turtlesim</code> node is subscribed to that topic to receive the data.</p>
<p>The highlighting feature of rqt_graph is very helpful when examining more complex systems with many nodes and topics connected in many different ways.</p>
<p>rqt_graph is a graphical introspection tool. Now we’ll look at some command line tools for introspecting topics.</p>
<h3 id="ros2-topic-list"><a class="header" href="#ros2-topic-list">ros2 topic list</a></h3>
<p>Running the <code>ros2 topic list</code> command in a new terminal will return a list of all the topics currently active in the system:</p>
<pre><code class="language-sh">/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
</code></pre>
<p><code>ros2 topic list -t</code> will return the same list of topics, this time with the topic type appended in brackets:</p>
<pre><code class="language-sh">/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
</code></pre>
<p>These attributes, particularly the type, are how nodes know they’re talking about the same information as it moves over topics.</p>
<p>If you’re wondering where all these topics are in rqt_graph, you can uncheck all the boxes under <strong>Hide:</strong></p>
<img src="https://docs.ros.org/en/humble/_images/unhide.png" />
<p>For now, though, leave those options checked to avoid confusion.</p>
<h3 id="ros2-topic-echo"><a class="header" href="#ros2-topic-echo">ros2 topic echo</a></h3>
<p>To see the data being published on a topic, use:</p>
<pre><code class="language-sh">ros2 topic echo &lt;topic_name&gt;
</code></pre>
<p>Since we know that <code>/teleop_turtle</code> publishes data to <code>/turtlesim</code> over the <code>/turtle1/cmd_vel</code> topic, let’s use <code>echo</code> to introspect that topic:</p>
<pre><code class="language-sh">ros2 topic echo /turtle1/cmd_vel
</code></pre>
<p>At first, this command won’t return any data. That’s because it’s waiting for <code>/teleop_turtle</code> to publish something.</p>
<p>Return to the terminal where <code>turtle_teleop_key</code> is running and use the arrows to move the turtle around. Watch the terminal where your <code>echo</code> is running at the same time, and you’ll see position data being published for every movement you make:</p>
<pre><code class="language-sh">linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
</code></pre>
<p>Now return to rqt_graph and uncheck the <strong>Debug</strong> box.</p>
<img src="https://docs.ros.org/en/humble/_images/debug.png" />
<p><code>/_ros2cli_26646</code> is the node created by the <code>echo</code> command we just ran (the number might be different). Now you can see that the publisher is publishing data over the <code>cmd_vel</code> topic, and two subscribers are subscribed to it.</p>
<h3 id="ros2-topic-info"><a class="header" href="#ros2-topic-info">ros2 topic info</a></h3>
<p>Topics don’t have to only be one-to-one communication; they can be one-to-many, many-to-one, or many-to-many.</p>
<p>Another way to look at this is running:</p>
<pre><code class="language-sh">ros2 topic info /turtle1/cmd_vel
</code></pre>
<p>Which will return:</p>
<pre><code class="language-sh">Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 2
</code></pre>
<h3 id="ros2-interface-show"><a class="header" href="#ros2-interface-show">ros2 interface show</a></h3>
<p>Nodes send data over topics using messages. Publishers and subscribers must send and receive the same type of message to communicate.</p>
<p>The topic types we saw earlier after running <code>ros2 topic list -t</code> let us know what message type is used on each topic. Recall that the <code>cmd_vel</code> topic has the type:</p>
<pre><code class="language-sh">geometry_msgs/msg/Twist
</code></pre>
<p>This means that in the package <code>geometry_msgs</code> there is a <code>msg</code> called <code>Twist</code>.</p>
<p>Now we can run <code>ros2 interface show &lt;msg type&gt;</code> on this type to learn its details. Specifically, what structure of data the message expects.</p>
<pre><code class="language-sh">ros2 interface show geometry_msgs/msg/Twist
</code></pre>
<p>For the message type from above it yields:</p>

```sh
# This expresses velocity in free space broken into its linear and angular parts.

    Vector3  linear
            float64 x
            float64 y
            float64 z
    Vector3  angular
            float64 x
            float64 y
            float64 z
```
<p>This tells you that the <code>/turtlesim</code> node is expecting a message with two vectors, <code>linear</code> and <code>angular</code>, of three elements each. If you recall the data we saw <code>/teleop_turtle</code> passing to <code>/turtlesim</code> with the <code>echo</code> command, it’s in the same structure:</p>
<pre><code class="language-sh">linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
</code></pre>
<h3 id="ros2-topic-pub"><a class="header" href="#ros2-topic-pub">ros2 topic pub</a></h3>
<p>Now that you have the message structure, you can publish data onto a topic directly from the command line using:</p>
<pre><code class="language-sh">ros2 topic pub &lt;topic_name&gt; &lt;msg_type&gt; '&lt;args&gt;'
</code></pre>
<p>The <code>'&lt;args&gt;'</code> argument is the actual data you’ll pass to the topic, in the structure you just discovered in the previous section.</p>
<p>It’s important to note that this argument needs to be input in YAML syntax. Input the full command like so:</p>
<pre><code class="language-sh">ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist &quot;{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}&quot;
</code></pre>
<p><code>--once</code> is an optional argument meaning “publish one message then exit”.</p>
<p>You will see the following output in the terminal:</p>
<pre><code class="language-sh">publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.8))
</code></pre>
<p>And you will see your turtle move like so:</p>
<img src="https://docs.ros.org/en/humble/_images/pub_once.png" />
<p>The turtle (and commonly the real robots which it is meant to emulate) require a steady stream of commands to operate continuously. So, to get the turtle to keep moving, you can run:</p>
<pre><code class="language-sh">ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist &quot;{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}&quot;
</code></pre>
<p>The difference here is the removal of the <code>--once</code> option and the addition of the <code>--rate 1</code> option, which tells <code>ros2 topic pub</code> to publish the command in a steady stream at 1 Hz.</p>
<img src="https://docs.ros.org/en/humble/_images/pub_stream.png" />
<p>You can refresh rqt_graph to see what’s happening graphically. You will see that the <code>ros2 topic pub ...</code> node (<code>/_ros2cli_30358</code>) is publishing over the <code>/turtle1/cmd_vel</code> topic, which is being received by both the <code>ros2 topic echo ...</code> node (<code>/_ros2cli_26646</code>) and the <code>/turtlesim</code> node now.</p>
<img src="https://docs.ros.org/en/humble/_images/rqt_graph2.png" />
<p>Finally, you can run <code>echo</code> on the <code>pose</code> topic and recheck rqt_graph:</p>
<pre><code class="language-sh">ros2 topic echo /turtle1/pose
</code></pre>
<img src="https://docs.ros.org/en/humble/_images/rqt_graph3.png" />
<p>You can see that the <code>/turtlesim</code> node is also publishing to the <code>pose</code> topic, which the new <code>echo</code> node has subscribed to.</p>
<h3 id="ros2-topic-hz"><a class="header" href="#ros2-topic-hz">ros2 topic hz</a></h3>
<p>For one last introspection on this process, you can view the rate at which data is published using:</p>
<pre><code class="language-sh">ros2 topic hz /turtle1/pose
</code></pre>
<p>It will return data on the rate at which the <code>/turtlesim</code> node is publishing data to the <code>pose</code> topic.</p>
<pre><code class="language-sh">average rate: 59.354
  min: 0.005s max: 0.027s std dev: 0.00284s window: 58
</code></pre>
<p>Recall that you set the rate of <code>turtle1/cmd_vel</code> to publish at a steady 1 Hz using <code>ros2 topic pub --rate 1</code>. If you run the above command with <code>turtle1/cmd_vel</code> instead of <code>turtle1/pose</code>, you will see an average reflecting that rate.</p>
<h2 id="summary"><a class="header" href="#summary">Summary:</a></h2>
<p>Nodes publish information over topics, which allows any number of other nodes to subscribe to and access that information. In this tutorial you examined the connections between several nodes over topics using rqt_graph and command line tools. You should now have a good idea of how data moves around a ROS 2 system.</p>
<h2 id="references"><a class="header" href="#references">References:</a></h2>
<ul>
<li>
<p><a href="https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html">Understanding ROS 2 Topics</a></p>
</li>
<li>
<p><a href="https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html">About Topics</a></p>
</li>
</ul>
</br>
<hr />


<h1 align="left">
<a href="Launch.md">⬅️</a>
</h1>
<h1 align="right">
<a href="Service.md">➡️</a>
</h1>
