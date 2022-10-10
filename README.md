## Vector Field-based Collision Avoidance for Moving Obstacles with Time-Varying Shape

Master's research at The University of Texas at Austin in the research group of Efstathios Bakolas.

The paper resulting from these simulations has been published at the Modeling, Estimation and Control Conference (MECC 2022).

To cite this work: 

Braquet, M. and Bakolas E., "Vector Field-based Collision Avoidance for Moving Obstacles with Time-Varying Shape", *Modeling, Estimation and Control Conference (MECC)*, 2022.

* Run `test_obstacle_avoidance.m` to obtain a trajectory derived from a control law based on the vector field for an environment composed of:
  * One/multiple static/moving ellipses/ellipsoids
  * Ellipses/ellipsoids with time-varying shapes
  * An ellipse and 4 edges (*bounded* environment)
  * A square and an ellipse
* Run `test_control_laws_circle.m` to compare the trajectories and control input norms for 3 different control laws (proportional controller only, proportional controller with threshold and proportional + derivative controllers) in the presence of a circular obstacle.
* Run `test_pathlines_ellipse.m` to plot the pathlines along the generated vector field for different initial position.
* Run `test_test_ellipsoid_closestPoint.m` to plot the shortest segment between a point and an ellipsoid.

### Simulation results

#### Moving ellipse

![Alt Text](https://github.com/MartinBraquet/vector-field-obstacle-avoidance/blob/main/videos/3%20movingellipse.gif)

#### Multiple ellipses

![Alt Text](https://github.com/MartinBraquet/vector-field-obstacle-avoidance/blob/main/videos/5%20multiellipse.gif)

#### Multiple moving ellipsoids

![Alt Text](https://github.com/MartinBraquet/vector-field-obstacle-avoidance/blob/main/videos/7%20moving_multiple_ellipsoids.gif)

#### Ellipses with time-varying shape

![Alt Text](https://github.com/MartinBraquet/vector-field-obstacle-avoidance/blob/main/videos/8%20uncertellipse_anim.gif)

#### Ellipsoids with time-varying shape

![Alt Text](https://github.com/MartinBraquet/vector-field-obstacle-avoidance/blob/main/videos/10%20uncertellipsoid.gif)

#### Ellipse and 4 edges (*bounded* environment)

![Alt Text](https://github.com/MartinBraquet/vector-field-obstacle-avoidance/blob/main/videos/11%20walls.gif)
