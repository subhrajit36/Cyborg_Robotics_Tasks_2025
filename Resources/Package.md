<h1 align="center">
ROS 2 Package
</h1>
</br>
<p>This tutorial will teach you how to create your first ROS 2 application. It is intended for developers who want to learn how to create custom packages in ROS 2, not for people who want to use ROS 2 with its existing packages.</p>
<h2 id="creating-a-package"><a class="header" href="#creating-a-package">Creating a package:</a></h2>
<p>All ROS 2 packages begin by running the command</p>
<pre><code class="language-sh">ros2 pkg create &lt;pkg-name&gt; --dependencies [deps]
</code></pre>
<p>in your workspace (usually <code>~/ros2_ws/src</code>).</p>
</br>
<p>To create a package for a specific client library:</p>
<ul>
<li>For C++:
<pre><code class="language-sh">ros2 pkg create &lt;pkg-name&gt; --dependencies [deps] --build-type ament_cmake
</code></pre>
</li>
<li>For Python:
<pre><code class="language-sh">ros2 pkg create &lt;pkg-name&gt; --dependencies [deps] --build-type ament_python
</code></pre>
</li>
</ul>
<p>You can then update the <code>package.xml</code> with your package info such as dependencies, descriptions, and authorship.</p>
<h3 id="c-packages"><a class="header" href="#c-packages">C++ Packages</a></h3>
<p>You will mostly use the <code>add_executable()</code> CMake macro along with</p>
<pre><code class="language-sh">ament_target_dependencies(&lt;executable-name&gt; [dependencies])
</code></pre>
<p>to create executable nodes and link dependencies.</p>
<p>To install your launch files and nodes, you can use the <code>install()</code> macro placed towards the end of the file but before the <code>ament_package()</code> macro.</p>
<p>An example for launch files and nodes:</p>

```sh 
# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
  
# Install nodes
install(
  TARGETS [node-names]
  DESTINATION lib/${PROJECT_NAME}
)
```
<h3 id="python-packages"><a class="header" href="#python-packages">Python Packages</a></h3>
<p>ROS 2 follows Python’s standard module distribution process that uses <code>setuptools</code>. For Python packages, the <code>setup.py</code> file complements a C++ package’s <code>CMakeLists.txt</code>. More details on distribution can be found in the <a href="https://docs.python.org/3/distributing/index.html#distributing-index">official documentation</a>.</p>
<p>In your ROS 2 package, you should have a <code>setup.cfg</code> file which looks like:</p>
<pre><code class="language-sh">[develop]
script_dir=$base/lib/&lt;package-name&gt;
[install]
install_scripts=$base/lib/&lt;package-name&gt;
</code></pre>
<p>and a <code>setup.py</code> file that looks like:</p>

```sh
import os
from glob import glob
from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='ROS 2 Developer',
    author_email='ros2@ros.com',
    maintainer='ROS 2 Developer',
    maintainer_email='ros2@ros.com',
    keywords=['foo', 'bar'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='My awesome package.',
    license='TODO',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'my_script = my_package.my_script:main'
        ],
    },
)
```
<h3 id="combined-c-and-python-packages"><a class="header" href="#combined-c-and-python-packages">Combined C++ and Python Packages</a></h3>
<p>When writing a package with both C++ and Python code, the <code>setup.py</code> file and <code>setup.cfg</code> file are not used. Instead, use <a href="https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html">ament_cmake_python</a>.</p>
<h2 id="references"><a class="header" href="#references">References:</a></h2>
<ul>
<li><a href="https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html">Developing a ROS 2 Package</a></li>
</ul>
</br>
<hr />

<h1 align="left">
<a href="Workspace.md">⬅️</a>
</h1>
<h1 align="right">
<a href="Node.md">➡️</a>
</h1>