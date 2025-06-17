<h1 align="center">
Linux File Permissions
</h1>
</br>
<ul>
<li>
<p>Like most of the modern operating systems, Linux is a multi-user OS hence extra layer of security is added to prevent users to access each other confidential files.</p>
</li>
<li>
<p>Linux divides authorization into two levels,</p>
<ul>
<li>
<p>Ownership</p>
</li>
<li>
<p>Permission</p>
</li>
</ul>
</li>
<li>
<p>Each file and directory has three user based permission groups,</p>
<ol>
<li>
<p><code>owner</code></p>
</li>
<li>
<p><code>group</code></p>
</li>
<li>
<p><code>all</code></p>
</li>
</ol>
</li>
<li>
<p>Each file or directory has three basic permission types,</p>
<ol>
<li>
<p><code>read</code></p>
</li>
<li>
<p><code>write</code></p>
</li>
<li>
<p><code>execute</code></p>
</li>
</ol>
</li>
<li>
<p>Linux File Permissions is a vast topic but you need not have to master it. The only thing you need to keep in mind that unlike Windows were programs with <code>.exe</code> extension can be executed, in UNIX/Linux, files with <strong>execute</strong> permission can only be executed/run by the user.</p>
</li>
<li>
<p>To make a file executable in Linux following command can be used,</p>
<pre><code class="language-sh">sudo chmod +x &lt;file_name&gt;
</code></pre>
</li>
</ul>
<br>
<hr />
<h4 id="references"><a class="header" href="#references">References</a></h4>
<ol>
<li><a href="https://www.linux.com/training-tutorials/understanding-linux-file-permissions/">Understanding Linux File Permissions</a></li>
<li><a href="https://www.tutorialspoint.com/unix/unix-file-permission.htm">Unix / Linux - File Permission / Access Modes</a></li>
</ol>
</br>
<hr />

<h1 align="left">
<a href="File_System.md">⬅️</a>
</h1>
<h1 align="right">
<a href="File_Command.md">➡️</a>
</h1>