<h1 align="center">
Shell Scripting
</h1>
</br>
<ul>
<li>
<p>You have learned in the previous section that UNIX shells can take commands from a file also. Such files are called <strong>shell scripts</strong>.</p>
</li>
<li>
<p>In this section we will explore how to write your own shell scripts.</p>
</li>
</ul>
<h2 id="shebang-unix"><a class="header" href="#shebang-unix">Shebang (UNIX)</a></h2>
<ul>
<li>
<p>In UNIX, shebang is a line which is added at the beginning of any script that you want to run as a standalone executable.</p>
</li>
<li>
<p>The line should starts with <code>#!</code>.</p>
</li>
<li>
<p>A shebang tells the shell which interpreter to use and its location in order to execute the script. For example,</p>
<ul>
<li>
<p>If I write a script for Bash Shell then I would have to include the following at the beginning of the script.</p>
<pre><code class="language-sh">#!/bin/bash
</code></pre>
</li>
<li>
<p>If I write a script for Python then I would have to include the following at the beginning of the script.</p>
<pre><code class="language-sh">#!/usr/bin/env python3
</code></pre>
</li>
</ul>
</li>
<li>
<p>A shebang gives the script the ability to be executed as an executable.</p>
</li>
</ul>
</br>
<hr />
<p><strong>Two Examples related to shell scripting is attached below:</strong></p>
<ol>
<li><a href="Shell_Scripting/Example_1.md">Example #1: Hello Shell Scripting</a></li>
<li><a href="Shell_Scripting/Example_2.md"">Example #2: Open Webpages</a></li>
</ol>
<hr />

<h1 align="left">
<a href="File_Command.md">⬅️</a>
</h1>
<h1 align="right">
<a href="Shell_Scripting/Example_1.md">➡️</a>
</h1>
