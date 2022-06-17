# sonar_obstacle_avoidance
ROS package for obstacle avoidance using a sidescan sonar.
The aim of this obstacle avoidance package is to solely use the measurements from a 2D scanning sonar to move an underwater robot without colliding with any obstacle in the environment.

We will send the robot velocity commands in the $x$ (linear) and yaw axis (angular around $z$).

**This package has been tested under Ubuntu 18.04 with ROS Melodic.**

# Potential field avoidance

#### Problem statement:
We want to move an underwater robot in an unknown environment without colliding. For that, we have a sidescan sonar sensor whose measurements are being published in a `sensor_msgs/LaserScan` topic. The topic's message content is outlined [here](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/LaserScan.html).

In our potential-field like approach for obstacle avoidance, the obstacles will generate repulsive fields that will make the robot decrease the linear speed and/or rotate as the vehicle approaches them. 

When a sonar beam collides with an obstacle, we will define a potential vector $p_i$ pointing to the vehicle, and inversely proportional to the squared distance between the obstacle and the vehicle $r_i$, such that:

$$ p_i = \begin{bmatrix}
\frac{1}{r_i^2}cos(\psi_i) \\
\frac{1}{r_i^2}sin(\psi_i) 
\end{bmatrix} $$

![](https://raw.githubusercontent.com/olayasturias/sonar_obstacle_avoidance/7ef739a28145ee3e6a69ec017fc029a9c91f0842/assets/obsav.svg)

The overall projection of the potential fields in the $x$ and $y$ axis is obtained as the sum of all values:

$$
p_x = \sum_ip_{i_x} = \sum_i\frac{1}{r_i^2}cos(\psi_i)$$
$$p_y = \sum_ip_{i_y} = \sum_i\frac{1}{r_i^2}sin(\psi_i)
$$

We will obtain the new velocity vector $V_p$ according to the maximum linear speed that we have defined for the robot $V_{max}$ and the potential vector $p$, multiplied by a constant $K_p$ that we will determine experimentally.

$$V_p = V_{max} - K_p*p$$

The robot can receive commands for linear velocity in $x$ and angular velocity around $z$ (yaw). The velocities are then formulated as:

$$ V_x = V_{max} - K_p*p_x$$

$$ \dot{\theta} =  K_{\theta}*\theta = K_{\theta}*arctan\left(\frac{- K_p*p_y}{V_{max} - K_p*p_x}\right)
$$

That is, the new linear velocity $V_x$ is inveresely proportional to the distance between the robot and the obstacle. How much the distance to the obstacle affects the linear velocity is defined by the gain.
The angular velocity is directly proportional to the angle between the $x$ axis and $V_p$, also regulated by a gain $K_{\theta}$.

![](https://raw.githubusercontent.com/olayasturias/sonar_obstacle_avoidance/fb51da19165f0cf4996e7d96d4ae56f8e1b2a938/assets/potential_vel.svg)