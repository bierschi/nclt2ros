# NCLT2ROSBAG

With [nclt2rosbag](https://github.com/bierschi/nclt2rosbag) it is possible to

- [download](https://github.com/bierschi/nclt2rosbag#download)
- [extract](https://github.com/bierschi/nclt2rosbag#extract)
- [convert](https://github.com/bierschi/nclt2rosbag#convert)
- [visualize](https://github.com/bierschi/nclt2rosbag#visualize)

the data from [The University of Michigan North Campus Long-Term Vision and LIDAR Dataset.](http://robots.engin.umich.edu/nclt/)


## USAGE
make the script <code>nclt2rosbag.py</code> executable
<pre><code>
chmod +x nclt2rosbag.py
</pre></code>

#### general structure

<pre><code>
./nclt2rosbag.py action date --lb3 --sen --vel --hokuyo --gt --gt_cov --bag --cam_folder
</pre></code>
<br>
The positional arguments <code>action</code> and <code>date</code> are mandatory.<br>
Specify an action command (download, extract, convert, visualize). Define also a date from the dataset. <br>
The optional arguments are --lb3 --sen --vel --hokuyo --gt --gt_cov --bag --cam_folder  
 
#### download

<pre><code>
./nclt2rosbag.py download 2013-01-10 --gt --gt_cov --sen --hokyuo --vel
</pre></code>

#### extract

<pre><code>
./nclt2rosbag.py extract 2013-01-10
</pre></code>

#### convert

<pre><code>
./nclt2rosbag.py convert 2013-01-10
</pre></code>

#### visualize

<pre><code>
./nclt2rosbag.py visualize 2013-01-10
</pre></code>

