
# NCLT2ROS

With [nclt2ros](https://github.com/bierschi/nclt2rosbag) it is possible to

- [download](https://github.com/bierschi/nclt2rosbag#download.launch)
- [extract](https://github.com/bierschi/nclt2rosbag#download.launch)
- [convert](https://github.com/bierschi/nclt2rosbag#convert.launch)
- [visualize](https://github.com/bierschi/nclt2rosbag#visualize.launch)

the data from [The University of Michigan North Campus Long-Term Vision and LIDAR Dataset.](http://robots.engin.umich.edu/nclt/)


![](examples/rviz_example.gif)


#### Table of Contents:

- [Launch files](https://github.com/bierschi/nclt2ros#launch-files)
- [Usage](https://github.com/bierschi/nclt2ros#usage)
- [Examples](https://github.com/bierschi/nclt2ros#examples)
- [Transformation tree](https://github.com/bierschi/nclt2ros#transformation-tree)




## Launch files

##### download.launch

```xml
<launch>
    # download
    <node name="nclt2ros" pkg="nclt2ros"  type="nclt2downloader" >
        <param name="date"          value="2013-01-10"/>
        <param name="output_path"   value="/home/christian/raw_data" />
        <param name="lb3"           value="False" />
        <param name="sen"           value="True" />
        <param name="vel"           value="False" />
        <param name="hokuyo"        value="False" />
        <param name="gt"            value="True" />
        <param name="gt_cov"        value="True" />
    </node>
</launch>
```

##### convert.launch

```xml
<launch>
    # download
    <node name="nclt2ros" pkg="nclt2ros"  type="nclt2rosbag" >
        <param name="date"          value="2013-01-10"/>
        <param name="output_path"   value="/home/christian/raw_data" />
        <param name="lb3"           value="False" />
        <param name="sen"           value="True" />
        <param name="vel"           value="False" />
        <param name="hokuyo"        value="False" />
        <param name="gt"            value="True" />
        <param name="gt_cov"        value="True" />
    </node>
</launch>
```


## Usage

create a catkin workspace
<pre><code>
mkdir -p ~/catkin_ws/src<br>
cd ~/catkin_ws/<br>
catkin_make <br>
source devel/setup.bash
</pre></code>

download and build this repository
<pre><code>
cd src <br>
git clone https://github.com/bierschi/nclt2ros.git <br>
cd ~/catkin_ws/ <br>
catkin_make
</pre></code>

source the catkin workspace to access the package nclt2ros
<pre><code>
source devel/setup.bash
</pre></code>

execute the roslaunch file 
<pre><code>
roslaunch nclt2ros download.launch
</pre></code>



## Examples

<pre><code>
./nclt2rosbag.py visualize 2013-01-10 --all
</pre></code>

<div align="left">
  <br>
  <img src="examples/nclt_all_png.png" alt="example" width="400" height="300">
</div>

<br>
visualized all data as a kml file:

<div align="left">
  <br>
  <img src="examples/nclt_all_kml.png" alt="example" width="673" height="375">
</div>


## Transformation Tree
<code>frame_id</code> and <code>topic_name</code> can be changed with the json configuration file 
<code>configuration.json</code> in folder <code>/cfg</code>

<div align="left">
  <br>
  <img src="tf_tree/nclt_tf_tree.png" alt="example" width="988" height="185">
</div>