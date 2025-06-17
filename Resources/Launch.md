<h1 align="center">
ROS 2 Launch
</h1>
<h2 id="description"><a class="header" href="#description">Description:</a></h2>
<p>A ROS 2 system typically consists of many nodes running across many different processes (and even different machines). While it is possible to run each of these nodes separately, it gets cumbersome quite quickly.</p>
<p>The launch system in ROS 2 is meant to automate the running of many nodes with a single command. It helps the user describe the configuration of their system and then executes it as described. The configuration of the system includes what programs to run, where to run them, what arguments to pass them, and ROS-specific conventions which make it easy to reuse components throughout the system by giving them each a different configuration. It is also responsible for monitoring the state of the processes launched, and reporting and/or reacting to changes in the state of those processes.</p>
<p>All of the above is specified in a launch file, which can be written in Python, XML, or YAML. This launch file can then be run using the <code>ros2 launch</code> command, and all of the nodes specified will be run.</p>
<h2 id="motivation"><a class="header" href="#motivation">Motivation:</a></h2>
<p>In most of the introductory tutorials, you have been opening new terminals for every new node you run. As you create more complex systems with more and more nodes running simultaneously, opening terminals and reentering configuration details becomes tedious.</p>
<p>Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.</p>
<p>Running a single launch file with the ros2 launch command will start up your entire system - all nodes and their configurations - at once.</p>
<h3 id="running-a-launch-file"><a class="header" href="#running-a-launch-file">Running a Launch File:</a></h3>
<p>Open a new terminal and run:</p>
<pre><code class="language-sh">ros2 launch turtlesim multisim.launch.py
</code></pre>
<p>This command will run the following launch file:</p>

```sh
  # turtlesim/launch/multisim.launch.py

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= &quot;turtlesim1&quot;, package='turtlesim', executable='turtlesim_node', output='screen'),
        launch_ros.actions.Node(
            namespace= &quot;turtlesim2&quot;, package='turtlesim', executable='turtlesim_node', output='screen'),
    ])
```
<blockquote>
<p><strong>Note:</strong> The launch file above is written in Python, but you can also use XML and YAML to create launch files. You can see a comparison of these different ROS 2 launch formats in <a href="https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html">Using Python, XML, and YAML for ROS 2 Launch Files</a>.</p>
</blockquote>
<p>This will run two turtlesim nodes:</p>
<img src="https://docs.ros.org/en/humble/_images/turtlesim_multisim.png" />
<p>You can find more information on ROS 2 launch in the <a href="https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html">ROS 2 launch tutorials</a>.</p>
<p>The significance of what you’ve done so far is that you’ve run two turtlesim nodes with one command. Once you learn to write your own launch files, you’ll be able to run multiple nodes - and set up their configuration - in a similar way, with the <code>ros2 launch</code> command.</p>
<h3 id="write-the-launch-file"><a class="header" href="#write-the-launch-file">Write the launch file</a></h3>
<p>Let’s put together a ROS 2 launch file using the <code>turtlesim</code> package and its executables. As mentioned above, this can either be in Python, XML, or YAML.</p>
<ul>
<li>
<p>launch/turtlesim_mimic_launch.py</p>
  
```sh
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```
</li>
<li>
<p>launch/turtlesim_mimic_launch.xml</p>

```sh
<launch>
<node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim1"/>
<node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim2"/>
<node pkg="turtlesim" exec="mimic" name="mimic">
    <remap from="/input/pose" to="/turtlesim1/turtle1/pose"/>
    <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel"/>
</node>
</launch>
```
</li>
<li>
<p>launch/turtlesim_mimic_launch.yaml</p>
  
```sh
launch:

- node:
    pkg: "turtlesim"
    exec: "turtlesim_node"
    name: "sim"
    namespace: "turtlesim1"

- node:
    pkg: "turtlesim"
    exec: "turtlesim_node"
    name: "sim"
    namespace: "turtlesim2"

- node:
    pkg: "turtlesim"
    exec: "mimic"
    name: "mimic"
    remap:
    -
        from: "/input/pose"
        to: "/turtlesim1/turtle1/pose"
    -
        from: "/output/cmd_vel"
        to: "/turtlesim2/turtle1/cmd_vel"

```
</code></pre>
</li>
</ul>
<h3 id="examine-the-launch-file"><a class="header" href="#examine-the-launch-file">Examine the Launch File</a></h3>
<p>We will look for explanation in python, you can refer <a href="https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html">this tutorial</a> for other formats.</p>
<p>These import statements pull in some Python <code>launch</code> modules.</p>

```sh
from launch import LaunchDescription
from launch_ros.actions import Node
```
<p>Next, the launch description itself begins:</p>

```sh
def generate_launch_description():
   return LaunchDescription([

   ])
```
<p>The first two actions in the launch description launch the two turtlesim windows:</p>

```sh
Node(
    package='turtlesim',
    namespace='turtlesim1',
    executable='turtlesim_node',
    name='sim'
),
Node(
    package='turtlesim',
    namespace='turtlesim2',
    executable='turtlesim_node',
    name='sim'
),
```
<p>The final action launches the mimic node with the remaps:</p>

```sh
Node(
    package='turtlesim',
    executable='mimic',
    name='mimic',
    remappings=[
      ('/input/pose', '/turtlesim1/turtle1/pose'),
      ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
    ]
)
```
<h3 id="ros2-launch"><a class="header" href="#ros2-launch">ros2 launch</a></h3>
<p>To run the launch file created above, enter into the directory you created earlier and run the following command:</p>
<ul>
<li>
<p>Python</p>
<pre><code class="language-sh">cd launch
ros2 launch turtlesim_mimic_launch.py
</code></pre>
</li>
<li>
<p>XML</p>
<pre><code class="language-sh">cd launch
ros2 launch turtlesim_mimic_launch.xml
</code></pre>
</li>
<li>
<p>YAML</p>
<pre><code class="language-sh">cd launch
ros2 launch turtlesim_mimic_launch.yaml
</code></pre>
</li>
</ul>
<blockquote>
<p><strong>Note:</strong> It is possible to launch a launch file directly (as we do above), or provided by a package. When it is provided by a package, the syntax is:</p>
<pre><code class="language-sh">ros2 launch &lt;package_name&gt; &lt;launch_file_name&gt;
</code></pre>
</blockquote>
<blockquote>
<p><strong>Note:</strong> For packages with launch files, it is a good idea to add an <code>exec_depend</code> dependency on the <code>ros2launch</code> package in your package’s <code>package.xml</code>:</p>
<pre><code class="language-sh">&lt;exec_depend&gt;ros2launch&lt;/exec_depend&gt;
</code></pre>
<p>This helps make sure that the <code>ros2 launch</code> command is available after building your package. It also ensures that all <a href="https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html">launch file formats</a> are recognized.</p>
</blockquote>
<h2 id="references"><a class="header" href="#references">References:</a></h2>
<ul>
<li>
<p><a href="https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html">Creating Launch Files</a></p>
</li>
<li>
<p><a href="https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html">Launching Multiple Nodes</a></p>
</li>
</ul>
</br>
<hr />

<h1 align="left">
<a href="Node.md">⬅️</a>
</h1>
<h1 align="right">
<a href="Topic.md">➡️</a>
</h1>