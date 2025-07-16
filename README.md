## ️ Build Instructions

1. Clone the repository into your Catkin workspace:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/soonhyo/baxter_demo.git
   ```

2. Install dependencies:

   ```bash
   rosdep update
   rosdep install --from-paths . --ignore-src -r -y
   ```

3. Build the workspace:

   ```bash
   cd ~/catkin_ws
   catkin_build baxter_demo
   source devel/setup.bash
   ```

---

##  Launch the Demo

Run the baxter initiation nodes:

```bash
roslaunch baxter_demo baxter_demo.launch
```
Run the full point cloud segmentation pipeline:

```bash
roslaunch baxter_demo pcl_demo.launch
```


### Change rgb parameters in rgb filter 

* Run rqt_reconfigure node

  ```bash
  rosrun rqt_reconfigure rqt_reconfigure
  ```

* Find rgb_filter in the node's list and adjust RGB color filter thresholds
