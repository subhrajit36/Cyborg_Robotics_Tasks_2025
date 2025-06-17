<h1 align="center">
Linux File Commands
</h1>
</br>
<h2 id="the-linux-terminal"><a class="header" href="#the-linux-terminal">The Linux Terminal</a></h2>
<ul>
<li>
<p>In UNIX, terminal is a text interface to interact with the OS. There are list of commands which user can use to interact with the OS.</p>
</li>
<li>
<p>Terminal is also called shell or console.</p>
</li>
<li>
<p>Command Prompt and Powershell are two such kind of interface provided by Windows OS.</p>
</li>
<li>
<p>In Ubuntu and most of the UNIX based OS a particular type of shell called <strong>Bash</strong> which is one kind of UNIX shell, is launched when you open any Terminal program. <strong>Zsh</strong> is another such popular shell for UNIX.</p>
</li>
<li>
<p>These UNIX shell like Bash and Zsh are command processors which takes commands from user, interprets it and executes the commands with the help of the underlying OS.</p>
</li>
<li>
<p>UNIX shells can also read commands from a file. Such kind of files are called <strong>Shell Scripts</strong>. In the next section we will explore this.</p>
</li>
<li>
<p>One such shell script is <strong>.bashrc</strong> which resides in <code>/home/</code> directory. This shell script runs everytime you open an instance of Bash shell.</p>
</li>
<li>
<p>In <strong>Ubuntu</strong> you can press <code>Ctrl + Alt + T</code> to open up an instance of the terminal.</p>
</li>
</ul>
<h2 id="common-linux-commands"><a class="header" href="#common-linux-commands">Common Linux Commands</a></h2>
<h3 id="sudo"><a class="header" href="#sudo">sudo</a></h3>
<ul>
<li>
<p>In UNIX a <strong>superuser</strong> or <strong>root</strong> is a user which has unrestricted access to all the commands, files, folders and resources of the system.</p>
</li>
<li>
<p>If you want to run any command as a superuser you need to prefix that command with <code>sudo</code>.</p>
</li>
<li>
<p>eg: <code>sudo chmod +x hello.sh</code></p>
</li>
</ul>
<h3 id="ls"><a class="header" href="#ls">ls</a></h3>
<div class="table-wrapper"><table><thead><tr><th>Command</th><th>Function</th></tr></thead><tbody>
<tr><td><code>ls</code></td><td>List the files and folders in the current directory</td></tr>
<tr><td><code>ls -l</code></td><td><code>-l</code> flag is used with <code>ls</code> to list files and folders in the current directory with their permissions.</td></tr>
<tr><td><code>ls -la</code></td><td><code>-la</code> flag is used to list all the files and folders including hidden ones with their permissions.</td></tr>
</tbody></table>
</div>
<h3 id="mkdir"><a class="header" href="#mkdir">mkdir</a></h3>
<div class="table-wrapper"><table><thead><tr><th>Usage</th><th>Result</th></tr></thead><tbody>
<tr><td><code>mkdir folder1 folder2</code></td><td>This command will create two folders, folder1 and folder2 in current directory.</td></tr>
</tbody></table>
</div>
<h3 id="cd"><a class="header" href="#cd">cd</a></h3>
<div class="table-wrapper"><table><thead><tr><th>Usage</th><th>Result</th></tr></thead><tbody>
<tr><td><code>cd ~</code></td><td>This will take to to <code>/home/</code> directory. In UNIX, tilde <code>~</code>  is used to represent home directory.</td></tr>
<tr><td><code>cd ~/colcon_ws</code></td><td>This will take you inside a folder named colcon_ws in home directory</td></tr>
</tbody></table>
</div>
<h3 id="echo"><a class="header" href="#echo">echo</a></h3>
<div class="table-wrapper"><table><thead><tr><th>Usage</th><th>Result</th></tr></thead><tbody>
<tr><td><code>echo &quot;hello&quot;</code></td><td>This will print the string <code>hello</code> in the terminal.</td></tr>
<tr><td><code>echo &quot;hello&quot; &gt;&gt; file.txt </code></td><td>This will append the string <code>hello</code> at the end of <code>file.txt</code>. This won't overwrite anything.</td></tr>
<tr><td><code>echo &quot;hello&quot; &gt; file.txt </code></td><td>This will append the string <code>hello</code> at the beginning of <code>file.txt</code>. This will overwrite.</td></tr>
</tbody></table>
</div>
<h3 id="find"><a class="header" href="#find">find</a></h3>
<div class="table-wrapper"><table><thead><tr><th>Command</th><th>Function</th></tr></thead><tbody>
<tr><td><code>find . -type d -name &quot;dir_name_start*&quot;</code></td><td>List the files and folders in the current directory starting with &quot;dir_name_start&quot;.</td></tr>
<tr><td><code>find . -type f -name &quot;file_name.*&quot;</code></td><td>Search for files starting with &quot;file_name&quot; with unknown extension.</td></tr>
</tbody></table>
</div>
<h3 id="grep"><a class="header" href="#grep">grep</a></h3>
<ul>
<li>
<p>Search for <em>&quot;text&quot;</em> string inside any type of file.</p>
<pre><code class="language-sh">grep -rnw &quot;text&quot;
</code></pre>
</li>
<li>
<p>Highlight &amp; Show only the pre-defined text in the output of a command</p>
<pre><code class="language-sh">colcon build | grep &quot;pkg_my_ros&quot;
</code></pre>
</li>
</ul>
<h3 id="which"><a class="header" href="#which">which</a></h3>
<ul>
<li>
<p>This command is used to find the location of an executable available to shell.</p>

```sh
which python

# Output
/usr/bin/python
```
</li>
</ul>
<h3 id="pwd"><a class="header" href="#pwd">pwd</a></h3>
<ul>
<li>
<p>This will print out the path of current directory you are in.</p>
<pre><code class="language-sh">pwd
</code></pre>
</li>
</ul>
<h3 id="cat"><a class="header" href="#cat">cat</a></h3>
<ul>
<li>
<p>This will print the content of <code>file.txt</code> with line numbers, in the terminal.</p>
<pre><code class="language-sh">cat -n file.txt
</code></pre>
</li>
</ul>
<h3 id="man"><a class="header" href="#man">man</a></h3>
<ul>
<li>
<p>This is very useful command in UNIX. If you are not sure how to use any command just call <code>man</code>.</p>
</li>
<li>
<p><code>man</code> stands for manual.</p>

```sh
man which

man cat

man find

man grep

man man
```
</li>
</ul>
</br>
<hr />

<h1 align="left">
<a href="File_Permission.md">⬅️</a>
</h1>
<h1 align="right">
<a href="Shell_Scripting.md">➡️</a>
</h1>
