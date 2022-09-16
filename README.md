# emcl2: mcl with expansion resetting (version 2)

![test](https://github.com/ryuichiueda/emcl2/actions/workflows/test.yml/badge.svg)

emcl is an alternative Monte Carlo localization (MCL) package to amcl (http://wiki.ros.org/amcl). Differently from amcl, KLD-sampling and adaptive MCL are not implemented. Instead, the expansion resetting and other features are implemented[^1][^2].

This package is an improved version of [ryuichi/emcl](https://github.com/ryuichiueda/emcl). This version works well in crowded or outdoor environments. 

## demo movies 

[![](https://img.youtube.com/vi/dqS7KgGxwBs/0.jpg)](https://www.youtube.com/watch?v=dqS7KgGxwBs)

[![](https://img.youtube.com/vi/n9tzKY6ua_o/0.jpg)](https://www.youtube.com/watch?v=n9tzKY6ua_o)

## Nodes

### emcl2_node

This node calculates the alpha value with a different algorithm than `emcl_node` in [ryuichi/emcl](https://github.com/ryuichiueda/emcl). This node counts the particles that make lasers penetrate occupancy cells. Specifically, this node chooses some particles at a rate of `~extraction_rate` and checks each of them with the following procedure:

* maps a set of laser scan on the occupancy grid map based on the pose of the particle
* judges the pose of the particle as wrong if all of lasers in a `~range_threshold`[rad] range penatrate occupancy grids

If the rate of the wrong particles is greater than `~alpha_threshold`, the node invokes a reset. 

This node also has a sensor resetting algorithm. When `~sensor_reset` is true, a particle with laser penetration is dragged back from occupied cells. 

#### Subscribed Topics 

* scan ([sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html))
    * laser scans
* tf ([tf/tfMessage](http://docs.ros.org/en/api/tf/html/msg/tfMessage.html))
    * transforms
* initialpose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
    * pose of particles for replacement

#### Published Topics

* mcl_pose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
    * the mean pose of the particles with covariance
* particlecloud ([geometry_msgs/PoseArray](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseArray.html))
    * poses of the particles
* tf ([tf/tfMessage](http://docs.ros.org/en/api/tf/html/msg/tfMessage.html))
    * the transform from odom (which can be remapped via the ~odom_frame_id parameter) to map
* alpha (std_msgs/Float32)
    * marginal likelihood of particles after sensor update

#### Services

* global_localization ([std_srvs/Empty](http://docs.ros.org/en/api/std_srvs/html/srv/Empty.html))
    * Initiate global localization, wherein all particles are dispersed randomly through the free space in the map.

#### Services Called

* static_map ([nav_msgs/GetMap](http://docs.ros.org/en/api/nav_msgs/html/srv/GetMap.html))
    * Initiate the map for localization.

#### parameters

* ~odom_freq (int, default: 20 [Hz])
    * frequency of odometry update
* ~num_particles (int, default: 1000)
    * number of particles
* ~odom_frame_id (string, default: "odom")
    * the frame for odometry
* ~footprint_frame_id (string, default: "base_footprint")
    * the frame of the localized robot's base
* ~base_frame_id (string, default: "base_link")
    * the frame of the robot's base. It is used for calculating the position and orientation of the LiDAR.
* ~global_frame_id (string, default: "map")
    * the frame for localization 
* ~initial_pose_x (double, default: 0.0 [m])
    * initial x coordinate of particles
* ~initial_pose_y (double, default: 0.0 [m])
    * initial y coordinate of particles
* ~initial_pose_a (double, default: 0.0 [rad])
    * initial yaw coordinate of particles
* ~odom_fw_dev_per_fw (double, default: 0.19 [m/m])
    * standard deviation of forward motion noise by forward motion
* ~odom_fw_dev_per_rot (double, default: 0.0001 [m/rad])
    * standard deviation of forward motion noise by rotational motion
* ~odom_rot_dev_per_fw (double, default: 0.13 [rad/m])
    * standard deviation of rotational motion noise by forward motion
* ~odom_rot_dev_per_rot (double, default: 0.2 [rad/rad])
    * standard deviation of rotational motion noise by rotational motion
* ~laser_likelihood_max_dist (double, default: 0.2 meters)
    * maximum distance to inflate occupied cells on the likelihood field map
* ~scan_increment (int, default: 1)
    * increment number when beams are picked from their sequence; the larger this number is, the fewer number of beams are used for calculation of likelihood
* ~alpha_threshold (double, default: 0.5)
    * threshold of the alpha value for expansion resetting
* ~expansion_radius_position (double, default: 0.1[m])
    * maximum change of the position on the xy-plane when the reset replaces a particle
* ~expansion_radius_orientation (double, default: 0.2[rad])
    * maximum change of the yaw angle when the reset replaces a particle
* ~extraction_rate (double, default: 0.1)
    * rate of particles that are checked by the node
* ~range_threshold (double, default: 0.1[rad])
    * threshold of the range of lasers; if all lasers on this range penetrate occupancy cells, the pose of the particle is judged as wrong
* ~sensor_reset (bool, default: true)
    * flag for sensor resettings


The followings have never been implemented yet.

* ~laser_min_range (double, default: 0.0[m])
    * threshold for discarding scans whose ranges are smaller than this value 
* ~laser_max_range (double, default: 100000000.0[m])
    * threshold for discarding scans whose ranges are larger than this value 




## ROS version 

* ROS Noetic Ninjemys (on Ubuntu 20.04 LTS, test on my note PC)
* ROS Melodic Morenia (on Ubuntu 18.04 LTS, test on GitHub Actions)


## citation

[^1]: R. Ueda: "[Syokai Kakuritsu Robotics (lecture note on probabilistic robotics)](https://www.amazon.co.jp/dp/B082SN3VTD)," Kodansya, 2019.

[^2]: R. Ueda, T. Arai, K. Sakamoto, T. Kikuchi, S. Kamiya: Expansion resetting for recovery from fatal error in Monte Carlo localization - comparison with sensor resetting methods, IEEE/RSJ IROS, pp.2481-2486, 2004. 

