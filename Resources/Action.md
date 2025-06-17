<h1 align="center">
ROS 2 Action
</h1>
<h2 id="description"><a class="header" href="#description">Description:</a></h2>
<p>Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.</p>
<p>Actions are built on topics and services. Their functionality is similar to services, except actions can be canceled. They also provide steady feedback, as opposed to services which return a single response.</p>
<p>Actions use a client-server model, similar to the publisher-subscriber model (described in the topics tutorial). An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.</p>
<img src="https://docs.ros.org/en/humble/_images/Action-SingleActionClient.gif" />
<h2 id="using-ros-actions"><a class="header" href="#using-ros-actions">Using ROS Actions:</a></h2>
<p>Start up the two turtlesim nodes, <code>/turtlesim</code> and <code>/teleop_turtle</code>.</p>
<p>Open a new terminal and run:</p>
<pre><code class="language-sh">ros2 run turtlesim turtlesim_node
</code></pre>
<p>Open another terminal and run:</p>
<pre><code class="language-sh">ros2 run turtlesim turtle_teleop_key
</code></pre>
<p>When you launch the <code>/teleop_turtle</code> node, you will see the following message in your terminal:</p>
<pre><code class="language-sh">Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
</code></pre>
<p>Let’s focus on the second line, which corresponds to an action. (The first instruction corresponds to the “cmd_vel” topic, discussed previously in the <a href="ros_topic.html">topics</a> tutorial.)</p>
<p>Notice that the letter keys <code>G|B|V|C|D|E|R|T</code> form a “box” around the <code>F</code> key on a US QWERTY keyboard (if you are not using a QWERTY keyboard, see <a href="https://upload.wikimedia.org/wikipedia/commons/d/da/KB_United_States.svg">this link</a> to follow along). Each key’s position around <code>F</code> corresponds to that orientation in turtlesim. For example, the <code>E</code> will rotate the turtle’s orientation to the upper left corner.</p>
<p>Pay attention to the terminal where the <code>/turtlesim</code> node is running. Each time you press one of these keys, you are sending a goal to an action server that is part of the <code>/turtlesim</code> node. The goal is to rotate the turtle to face a particular direction. A message relaying the result of the goal should display once the turtle completes its rotation:</p>
<pre><code class="language-sh">[INFO] [turtlesim]: Rotation goal completed successfully
</code></pre>
<p>The <code>F</code> key will cancel a goal mid-execution.</p>
<p>Try pressing the <code>C</code> key, and then pressing the <code>F</code> key before the turtle can complete its rotation. In the terminal where the <code>/turtlesim</code> node is running, you will see the message:</p>
<pre><code class="language-sh">[INFO] [turtlesim]: Rotation goal canceled
</code></pre>
<p>Not only can the client-side (your input in the teleop) stop a goal, but the server-side (the <code>/turtlesim</code> node) can as well. When the server-side chooses to stop processing a goal, it is said to “abort” the goal.</p>
<p>Try hitting the <code>D</code> key, then the <code>G</code> key before the first rotation can complete. In the terminal where the <code>/turtlesim</code> node is running, you will see the message:</p>
<pre><code class="language-sh">[WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal
</code></pre>
<p>This action server chose to abort the first goal because it got a new one. It could have chosen something else, like reject the new goal or execute the second goal after the first one finished. Don’t assume every action server will choose to abort the current goal when it gets a new one.</p>
<h3 id="ros2-action-list"><a class="header" href="#ros2-action-list">ros2 action list</a></h3>
<p>To identify all the actions in the ROS graph, run the command:</p>
<pre><code class="language-sh">ros2 action list
</code></pre>
<p>Which will return:</p>
<pre><code class="language-sh">/turtle1/rotate_absolute
</code></pre>
<p>This is the only action in the ROS graph right now. It controls the turtle’s rotation, as you saw earlier. You also already know that there is one action client (part of <code>/teleop_turtle</code>) and one action server (part of <code>/turtlesim</code>) for this action from using the <code>ros2 node info &lt;node_name&gt;</code> command.</p>
<h3 id="ros2-action-list--t"><a class="header" href="#ros2-action-list--t">ros2 action list -t</a></h3>
<p>Actions have types, similar to topics and services. To find <code>/turtle1/rotate_absolute</code>’s type, run the command:</p>
<pre><code class="language-sh">ros2 action list -t
</code></pre>
<p>Which will return:</p>
<pre><code class="language-sh">/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
</code></pre>
<p>In brackets to the right of each action name (in this case only <code>/turtle1/rotate_absolute</code>) is the action type, <code>turtlesim/action/RotateAbsolute</code>. You will need this when you want to execute an action from the command line or from code.</p>
<h3 id="ros2-action-info"><a class="header" href="#ros2-action-info">ros2 action info</a></h3>
<p>You can further introspect the <code>/turtle1/rotate_absolute</code> action with the command:</p>
<pre><code class="language-sh">ros2 action info /turtle1/rotate_absolute
</code></pre>
<p>Which will return</p>
<pre><code class="language-sh">Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
</code></pre>
<p>This tells us what we learned earlier from running <code>ros2 node info</code> on each node: The <code>/teleop_turtle</code> node has an action client and the <code>/turtlesim</code> node has an action server for the <code>/turtle1/rotate_absolute</code> action.</p>
<h3 id="ros2-interface-show"><a class="header" href="#ros2-interface-show">ros2 interface show</a></h3>
<p>One more piece of information you will need before sending or executing an action goal yourself is the structure of the action type.</p>
<p>Recall that you identified <code>/turtle1/rotate_absolute</code>’s type when running the command <code>ros2 action list -t</code>. Enter the following command with the action type in your terminal:</p>
<pre><code class="language-sh">ros2 interface show turtlesim/action/RotateAbsolute
</code></pre>
<p>Which will return:</p>
<pre><code class="language-sh"># The desired heading in radians
float32 theta
---
The angular displacement in radians to the starting position
float32 delta
---
The remaining rotation in radians
float32 remaining
</code></pre>
<p>The section of this message above the first <code>---</code> is the structure (data type and name) of the goal request. The next section is the structure of the result. The last section is the structure of the feedback.</p>
<h3 id="ros2-action-send_goal"><a class="header" href="#ros2-action-send_goal">ros2 action send_goal</a></h3>
<p>Now let’s send an action goal from the command line with the following syntax:</p>
<pre><code class="language-sh">ros2 action send_goal &lt;action_name&gt; &lt;action_type&gt; &lt;values&gt;
</code></pre>
<p><code>&lt;values&gt;</code> need to be in YAML format.</p>
<p>Keep an eye on the turtlesim window, and enter the following command into your terminal:</p>
<pre><code class="language-sh">ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute &quot;{theta: 1.57}&quot;
</code></pre>
<p>You should see the turtle rotating, as well as the following message in your terminal:</p>
<pre><code class="language-sh">Waiting for an action server to become available...
Sending goal:
   theta: 1.57

Goal accepted with ID: f8db8f44410849eaa93d3feb747dd444

Result:
  delta: -1.568000316619873

Goal finished with status: SUCCEEDED
</code></pre>
<p>All goals have a unique ID, shown in the return message. You can also see the result, a field with the name <code>delta</code>, which is the displacement to the starting position.</p>
<p>To see the feedback of this goal, add <code>--feedback</code> to the <code>ros2 action send_goal</code> command:</p>
<pre><code class="language-sh">ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute &quot;{theta: -1.57}&quot; --feedback
</code></pre>
<p>Your terminal will return the message:</p>
<pre><code class="language-sh">Sending goal:
   theta: -1.57

Goal accepted with ID: e6092c831f994afda92f0086f220da27

Feedback:
  remaining: -3.1268222332000732

Feedback:
  remaining: -3.1108222007751465

…

Result:
  delta: 3.1200008392333984

Goal finished with status: SUCCEEDED
</code></pre>
<h2 id="summary"><a class="header" href="#summary">Summary:</a></h2>
<p>You will continue to receive feedback, the remaining radians, until the goal is complete.</p>
<p>Actions are like services that allow you to execute long running tasks, provide regular feedback, and are cancelable.</p>
<p>A robot system would likely use actions for navigation. An action goal could tell a robot to travel to a position. While the robot navigates to the position, it can send updates along the way (i.e. feedback), and then a final result message once it’s reached its destination.</p>
<p>Turtlesim has an action server that action clients can send goals to for rotating turtles. In this tutorial, you introspected that action, <code>/turtle1/rotate_absolute</code>, to get a better idea of what actions are and how they work.</p>
<h2 id="references"><a class="header" href="#references">References:</a></h2>
<ul>
<li>
<p><a href="https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html">Understanding ROS 2 Actions</a></p>
</li>
<li>
<p><a href="https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html">About Actions</a></p>
</li>
</ul>
</br>
<hr />

<h1 align="left">
<a href="Service.md">⬅️</a>
</h1>
<h1 align="right">
<a href="Simulation.md">➡️</a>
</h1>