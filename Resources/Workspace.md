<h1 align="center">
ROS 2 Workspace
</h1>
<h2 id="prerequisites"><a class="header" href="#prerequisites">Prerequisites:</a></h2>
<ul>
<li>
<p><em><strong>Configure Environment:</strong></em></p>
<p>The ROS 2 development environment needs to be correctly configured before use. This can be done in two ways: either sourcing the setup files in every new shell you open, or adding the source command to your startup script.
You can refer the tutorial to <a href="https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html">configure ros 2 environment.</a></p>
</li>
<li>
<p><em><strong>Install colcon:</strong></em></p>
<p><code>colcon</code> is an iteration on the ROS build tools <em>catkin_make, catkin_make_isolated, catkin_tools</em> and <em>ament_tools</em>. For more information on the design of colcon see <a href="https://design.ros2.org/articles/build_tool.html">this document</a>.</p>
<pre><code class="language-sh">sudo apt install python3-colcon-common-extensions
</code></pre>
</li>
</ul>
<h2 id="basics"><a class="header" href="#basics">Basics:</a></h2>
<p>A workspace is a directory containing ROS 2 packages <em>(ROS 2 is version name and not two packages)</em>. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace in the terminal you plan to work in. This makes ROS 2’s packages available for you to use in that terminal.</p>
<p>A ROS workspace is a directory with a particular structure. Commonly there is a <code>src</code> subdirectory. Inside that subdirectory is where the source code of ROS packages will be located. Typically the directory starts otherwise empty.</p>
<p>colcon does out of source builds. By default it will create the following directories as peers of the <code>src</code> directory:</p>
<ul>
<li>
<p>The <code>build</code> directory will be where intermediate files are stored. For each package a subfolder will be created in which e.g. CMake is being invoked.</p>
</li>
<li>
<p>The <code>install</code> directory is where each package will be installed to. By default each package will be installed into a separate subdirectory.</p>
</li>
<li>
<p>The <code>log</code> directory contains various logging information about each colcon invocation.</p>
</li>
</ul>
<blockquote>
<p><strong>Note:</strong> Compared to <em>catkin</em> there is no <code>devel</code> directory.</p>
</blockquote>
<h3 id="create-a-workspace"><a class="header" href="#create-a-workspace">Create a workspace</a></h3>
<p>First, create a directory (<code>ros2_ws</code>) to contain our workspace:</p>
<pre><code class="language-sh">mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
</code></pre>
<p>At this point the workspace contains a single empty directory <code>src</code>:</p>
<pre><code class="language-sh">.
└── src

1 directory, 0 files
</code></pre>
</br>
<blockquote>
<p><em><strong>NOTE: Further if you want to explore more on ROS workspace, happy learning...</strong></em></p>
</blockquote>
<hr />
<h3 id="add-some-sources"><a class="header" href="#add-some-sources">Add some sources</a></h3>
<p>Let’s clone the <a href="https://github.com/ros2/examples">examples</a> repository into the <code>src</code> directory of the workspace:</p>
<pre><code class="language-sh">git clone https://github.com/ros2/examples src/examples -b humble
</code></pre>
<p>Now the workspace should have the source code to the ROS 2 examples:</p>
<pre><code class="language-sh">.
└── src
    └── examples
        ├── CONTRIBUTING.md
        ├── LICENSE
        ├── rclcpp
        ├── rclpy
        └── README.md

4 directories, 3 files
</code></pre>
<h3 id="source-an-underlay"><a class="header" href="#source-an-underlay">Source an underlay</a></h3>
<p>It is important that we have sourced the environment for an existing ROS 2 installation that will provide our workspace with the necessary build dependencies for the example packages. This is achieved by sourcing the setup script provided by a binary installation or a source installation, ie. another colcon workspace (see <a href="https://docs.ros.org/en/humble/Installation.html">Installation</a>). We call this environment an <strong>underlay</strong>.</p>
<h3 id="build-the-workspace"><a class="header" href="#build-the-workspace">Build the workspace</a></h3>
<p>In the root of the workspace, run <code>colcon build</code>. Since build types such as <code>ament_cmake</code> do not support the concept of the <code>devel</code> space and require the package to be installed, colcon supports the option <code>--symlink-install</code>. This allows the installed files to be changed by changing the files in the <code>source</code> space (e.g. Python files or other non-compiled resources) for faster iteration.</p>
<pre><code class="language-sh">colcon build --symlink-install
</code></pre>
<p>After the build is finished, we should see the <code>build</code>, <code>install</code>, and <code>log</code> directories:</p>
<pre><code class="language-sh">.
├── build
├── install
├── log
└── src

4 directories, 0 files
</code></pre>
<h3 id="run-tests-optional"><a class="header" href="#run-tests-optional">Run tests (Optional)</a></h3>
<p>To run tests for the packages we just built, run the following:</p>
<pre><code class="language-sh">colcon test
</code></pre>
<blockquote>
<p><em><strong>NOTE: This may fail sometimes due to non-updated packages. You may ignore this command and no need to run <code>colcon test</code> if your <code>colcon build</code> has ran successfully.</strong></em></p>
</blockquote>
<h3 id="source-the-environment"><a class="header" href="#source-the-environment">Source the environment</a></h3>
<p>When colcon has completed building successfully, the output will be in the <code>install</code> directory. Before you can use any of the installed executables or libraries, you will need to add them to your path and library paths. colcon will have generated bash/bat files in the <code>install</code> directory to help set up the environment. These files will add all of the required elements to your path and library paths as well as provide any bash or shell commands exported by packages.</p>
<pre><code class="language-sh">source install/setup.bash
</code></pre>
<h2 id="create-your-own-package"><a class="header" href="#create-your-own-package">Create your own package:</a></h2>
<p>colcon uses the <code>package.xml</code> specification defined in <a href="https://www.ros.org/reps/rep-0149.html">REP 149</a> (<a href="https://www.ros.org/reps/rep-0140.html">format</a> 2 is also supported).</p>
<p>colcon supports multiple build types. The recommended build types are <code>ament_cmake</code> and <code>ament_python</code>. Also supported are pure <code>cmake</code> packages.</p>
<p>An example of an <code>ament_python</code> build is the <a href="https://github.com/ament/ament_index/tree/humble/ament_index_python">ament_index_python package</a> , where the <em>setup.py</em> is the primary entry point for building.</p>
<p>A package such as <a href="https://github.com/ros2/demos/tree/humble/demo_nodes_cpp">demo_nodes_cpp</a> uses the <code>ament_cmake</code> build type, and uses CMake as the build tool.</p>
<p>For convenience, you can use the tool <code>ros2 pkg create</code> to create a new package based on a template.</p>
<h2 id="setup-colcon_cd"><a class="header" href="#setup-colcon_cd">Setup <code>colcon_cd</code>:</a></h2>
<p>The command <code>colcon_cd</code> allows you to quickly change the current working directory of your shell to the directory of a package. As an example <code>colcon_cd some_ros_package</code> would quickly bring you to the directory <code>~/ros2_ws/src/some_ros_package</code>.</p>
<pre><code class="language-sh">echo &quot;source /usr/share/colcon_cd/function/colcon_cd.sh&quot; &gt;&gt; ~/.bashrc
echo &quot;export _colcon_cd_root=/opt/ros/humble/&quot; &gt;&gt; ~/.bashrc
</code></pre>
<p>Depending on the way you installed <code>colcon_cd</code> and where your workspace is, the instructions above may vary, please refer to the <a href="https://colcon.readthedocs.io/en/released/user/installation.html#quick-directory-changes">documentation</a> for more details. To undo this in Linux and macOS, locate your system’s shell startup script and remove the appended source and export commands.</p>
<h2 id="tips"><a class="header" href="#tips">Tips:</a></h2>
<ul>
<li>
<p>If you do not want to build a specific package place an empty file named <code>COLCON_IGNORE</code> in the directory and it will not be indexed.</p>
</li>
<li>
<p>If you want to avoid configuring and building tests in CMake packages you can pass: <code>--cmake-args -DBUILD_TESTING=0</code>.</p>
</li>
<li>
<p>If you want to run a single particular test from a package:</p>
<pre><code class="language-sh">colcon test --packages-select YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG
</code></pre>
</li>
</ul>
<h2 id="references"><a class="header" href="#references">References:</a></h2>
<ul>
<li>
<p><a href="https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html">Configuring ROS 2 Environment</a></p>
</li>
<li>
<p><a href="https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html">Creating a ROS 2 Workspace</a></p>
</li>
</ul>
</br>
<hr />

<h1 align="left">
<a href="Linux_Resources.md">⬅️</a>
</h1>
<h1 align="right">
<a href="Package.md">➡️</a>
</h1>
