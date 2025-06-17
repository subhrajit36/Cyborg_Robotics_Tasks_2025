<h1 align="center">
ROS 2 Service
</h1>
<h2 id="description"><a class="header" href="#description">Description:</a></h2>
<p>Services are another method of communication for nodes in the ROS graph. Services are based on a call-and-response model versus the publisher-subscriber model of topics. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.</p>
<img src="https://docs.ros.org/en/humble/_images/Service-SingleServiceClient.gif" />
<img src="https://docs.ros.org/en/humble/_images/Service-MultipleServiceClient.gif" />
<h2 id="using-ros-services"><a class="header" href="#using-ros-services">Using ROS Services:</a></h2>
<p>Start up the two turtlesim nodes, <code>/turtlesim</code> and <code>/teleop_turtle</code>.</p>
<p>Open a new terminal and run:</p>
<pre><code class="language-sh">ros2 run turtlesim turtlesim_node
</code></pre>
<p>Open another terminal and run:</p>
<pre><code class="language-sh">ros2 run turtlesim turtle_teleop_key
</code></pre>
<h3 id="ros2-service-list"><a class="header" href="#ros2-service-list">ros2 service list</a></h3>
<p>Running the <code>ros2 service list</code> command in a new terminal will return a list of all the services currently active in the system:</p>
<pre><code class="language-sh">/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
</code></pre>
<p>You will see that both nodes have the same six services with <code>parameters</code> in their names. Nearly every node in ROS 2 has these infrastructure services that parameters are built off of. There will be more about parameters in the next tutorial. In this tutorial, the parameter services will be omitted from the discussion.</p>
<p>For now, let’s focus on the turtlesim-specific services, <code>/clear</code>, <code>/kill</code>, <code>/reset</code>, <code>/spawn</code>, <code>/turtle1/set_pen</code>, <code>/turtle1/teleport_absolute</code>, and <code>/turtle1/teleport_relative</code>. You may recall interacting with some of these services using rqt in the <a href="https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html">Use turtlesim, ros2, and rqt</a> tutorial.</p>
<h3 id="ros2-service-type"><a class="header" href="#ros2-service-type">ros2 service type</a></h3>
<p>Services have types that describe how the request and response data of a service is structured. Service types are defined similarly to topic types, except service types have two parts: one message for the request and another for the response.</p>
<p>To find out the type of a service, use the command:</p>
<pre><code class="language-sh">ros2 service type &lt;service_name&gt;
</code></pre>
<p>Let’s take a look at turtlesim’s <code>/clear</code> service. In a new terminal, enter the command:</p>
<pre><code class="language-sh">ros2 service type /clear
</code></pre>
<p>Which will return:</p>
<pre><code class="language-sh">std_srvs/srv/Empty
</code></pre>
<p>The <code>Empty</code> type means the service call sends no data when making a request and receives no data when receiving a response.</p>
<h3 id="ros2-service-list--t"><a class="header" href="#ros2-service-list--t">ros2 service list -t</a></h3>
<p>To see the types of all the active services at the same time, you can append the <code>--show-types</code> option, abbreviated as <code>-t</code>, to the <code>list</code> command:</p>
<pre><code class="language-sh">ros2 service list -t
</code></pre>
<p>Which will return:</p>
<pre><code class="language-sh">/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
...
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
...
</code></pre>
<h3 id="ros2-service-find"><a class="header" href="#ros2-service-find">ros2 service find</a></h3>
<p>If you want to find all the services of a specific type, you can use the command:</p>
<pre><code class="language-sh">ros2 service find &lt;type_name&gt;
</code></pre>
<p>For example, you can find all the <code>Empty</code> typed services like this:</p>
<pre><code class="language-sh">ros2 service find std_srvs/srv/Empty
</code></pre>
<p>Which will return:</p>
<pre><code class="language-sh">/clear
/reset
</code></pre>
<h3 id="ros2-interface-show"><a class="header" href="#ros2-interface-show">ros2 interface show</a></h3>
<p>You can call services from the command line, but first you need to know the structure of the input arguments.</p>
<pre><code class="language-sh">ros2 interface show &lt;type_name&gt;
</code></pre>
<p>Try this on the <code>/clear</code> service’s type, <code>Empty</code>:</p>
<pre><code class="language-sh">ros2 interface show std_srvs/srv/Empty
</code></pre>
<p>Which will return:</p>
<pre><code class="language-sh">---
</code></pre>
<p>The <code>---</code> separates the request structure (above) from the response structure (below). But, as you learned earlier, the <code>Empty</code> type doesn’t send or receive any data. So, naturally, its structure is blank.</p>
<p>Let’s introspect a service with a type that sends and receives data, like <code>/spawn</code>. From the results of <code>ros2 service list -t</code>, we know <code>/spawn</code>’s type is <code>turtlesim/srv/Spawn</code>.</p>
<p>To see the request and response arguments of the <code>/spawn</code> service, run the command:</p>
<pre><code class="language-sh">ros2 interface show turtlesim/srv/Spawn
</code></pre>
<p>Which will return:</p>
<pre><code class="language-sh">float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
</code></pre>
<p>The information above the <code>---</code> line tells us the arguments needed to call <code>/spawn</code>. <code>x</code>, <code>y</code> and <code>theta</code> determine the 2D pose of the spawned turtle, and <code>name</code> is clearly optional.</p>
<p>The information below the line isn’t something you need to know in this case, but it can help you understand the data type of the response you get from the call.</p>
<h3 id="ros2-service-call"><a class="header" href="#ros2-service-call">ros2 service call</a></h3>
<p>Now that you know what a service type is, how to find a service’s type, and how to find the structure of that type’s arguments, you can call a service using:</p>
<pre><code class="language-sh">ros2 service call &lt;service_name&gt; &lt;service_type&gt; &lt;arguments&gt;
</code></pre>
<p>The <code>&lt;arguments&gt;</code> part is optional. For example, you know that <code>Empty</code> typed services don’t have any arguments:</p>
<pre><code class="language-sh">ros2 service call /clear std_srvs/srv/Empty
</code></pre>
<p>This command will clear the turtlesim window of any lines your turtle has drawn.</p>
<img src="https://docs.ros.org/en/humble/_images/clear.png" />
<p>Now let’s spawn a new turtle by calling <code>/spawn</code> and setting arguments. Input <code>&lt;arguments&gt;</code> in a service call from the command-line need to be in YAML syntax.</p>
<p>Enter the command:</p>
<pre><code class="language-sh">ros2 service call /spawn turtlesim/srv/Spawn &quot;{x: 2, y: 2, theta: 0.2, name: ''}&quot;
</code></pre>
<p>You will get this method-style view of what’s happening, and then the service response:</p>
<pre><code class="language-sh">requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.2, name='')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
</code></pre>
<p>Your turtlesim window will update with the newly spawned turtle right away:</p>
<img src="https://docs.ros.org/en/humble/_images/spawn1.png" />
<h2 id="summary"><a class="header" href="#summary">Summary:</a></h2>
<p>Nodes can communicate using services in ROS 2. Unlike a topic - a one way communication pattern where a node publishes information that can be consumed by one or more subscribers - a service is a request/response pattern where a client makes a request to a node providing the service and the service processes the request and generates a response.</p>
<p>You generally don’t want to use a service for continuous calls; topics or even actions would be better suited.</p>
<p>In this tutorial you used command line tools to identify, introspect, and call services.</p>
<h2 id="references"><a class="header" href="#references">References:</a></h2>
<ul>
<li>
<p><a href="https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html">Understanding ROS 2 Services</a></p>
</li>
<li>
<p><a href="https://docs.ros.org/en/humble/Concepts/Basic/About-Services.html">About Services</a></p>
</li>
</ul>
</br>
<hr />

<h1 align="left">
<a href="Topic.md">⬅️</a>
</h1>
<h1 align="right">
<a href="Action.md">➡️</a>
</h1>