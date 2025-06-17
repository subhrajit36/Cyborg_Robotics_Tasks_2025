<h1 align="center">
Example #2: Open URLs
</h1>
<h2 id="aim"><a class="header" href="#aim">Aim</a></h2>
<p>To write a shell script to open following two URLs in Firefox in two separate windows.</p>
<pre><code>https://www.youtube.com/
https://docs.ros.org/en/humble/index.html
</code></pre>
<p><strong>Steps:</strong></p>
<p>Go inside the shell-scripts folder.</p>
<pre><code>cd ~/workspace/shell-scripts
</code></pre>
<p>Inside this folder create a file called open-urls.sh.</p>
<pre><code>touch open-urls.sh
</code></pre>
<p>Open this file in a text-editor</p>
<pre><code>gedit open-urls.sh
</code></pre>
<p>If you have Sublime-Text installed you can use</p>
<pre><code>subl open-urls.sh
</code></pre>
<p>Now write the shebang for the script.</p>
<pre><code>#!/bin/bash
</code></pre>
<p>Create two variables to store the string two URLs.</p>
<pre><code>URL1=&quot;https://www.youtube.com/&quot;
URL2=&quot;https://docs.ros.org/en/humble/index.html&quot;
</code></pre>
<p>Pass the two URL variables as arguments to firefox command along with the flag to open the URLs in separate windows.</p>
<pre><code>firefox -new-window $URL1 $URL2
</code></pre>
<p>Now save the file and close the text-editor.<br></p>
<p>Use chmod to make this script an executable.</p>
<pre><code>sudo chmod +x open-urls.sh
</code></pre>
<p>Run the script.</p>
<pre><code>./open-urls.sh
</code></pre>
<p>Now Firefox should open up the two URLs in two separate windows.</p>
<p>Code</p>
<pre><code>#!/bin/bash
</code></pre>
<p>Store URLs in two variables</p>
<pre><code>URL1=&quot;https://www.youtube.com/&quot;
URL2=&quot;https://docs.ros.org/en/humble/index.html&quot;
</code></pre>
<p>Print some message</p>
<pre><code>echo &quot;** Opening $URL1 and $URL2 in Firefox **&quot;
</code></pre>
<p>Use firefox to open the two URLs in separate windows</p>
<pre><code>firefox -new-window $URL1 $URL2
</code></pre>
</br>
<hr />

<h1 align="left">
<a href="Example_1.md">⬅️</a>
</h1>
<h1 align="right">
<a href="../../Workspace.md">➡️</a>
</h1>
