<h1 align="center">
Example #1: Hello Shell Scripting
</h1>
<h2 id="aim"><a class="header" href="#aim">Aim</a></h2>
<p>To write a simple shell script to print “Hello Shell Scripting” in the terminal.<br></p>
<p><strong>Steps:</strong><br>
Create a shell-scripts folder inside workspace folder in your home folder. This is just to keep all your project files organized.</p>
<pre><code>mkdir -p ~/workspace/shell-scripts
</code></pre>
<p>Go inside the shell-scripts folder.</p>
<pre><code>cd ~/workspace/shell-scripts
</code></pre>
<p>Inside this folder create a file called hello.sh.</p>
<pre><code>touch hello.sh
</code></pre>
<p>Open this file in a text-editor</p>
<pre><code>gedit hello.sh
</code></pre>
<p>If you have Sublime-Text installed, you can use-</p>
<pre><code>subl hello.sh
</code></pre>
<p>Now write the shebang for the script.</p>
<pre><code>#!/bin/bash
</code></pre>
<p>Below it write the command to print “Hello Shell Scripting” in the terminal.</p>
<pre><code>echo Hello World
</code></pre>
<p>Now save the file and close the text-editor.<br></p>
<p>Use chmod to make this script an executable.</p>
<pre><code>sudo chmod +x hello.sh
</code></pre>
<p>Run the script.</p>
<pre><code>./hello.sh
</code></pre>
<p>If done correctly you should see the following output in the terminal-</p>
<pre><code>Hello Shell Scripting 
</code></pre>
<p>Code</p>
<pre><code>#!/bin/bash
echo Hello World
</code></pre>
</br>
<hr />

<h1 align="left">
<a href="../Shell_Scripting.md">⬅️</a>
</h1>
<h1 align="right">
<a href="Example_2.md">➡️</a>
</h1>
