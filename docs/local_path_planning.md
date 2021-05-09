## local path planning
This robot has a local path planner like DWA (Dynamic Window Approach)

#### (1) find a search area (Dynamic window)
A search area (the maximum and minimum control inputs) is calculated based on the current linear speed and angular velocity and the maximum and minimum velocity and maximum acceleration of the robot.  
This search area is called Dynamic Window.

* [Robot.MAX\_LIN\_ACC](../src/robot.py)
* [Robot.MAX\_ANG\_ACC](../src/robot.py)
* [Robot.MAX\_V](../src/robot.py)
* [Robot.MIN\_V](../src/robot.py)
* [Robot.MAX\_OMEGA](../src/robot.py)
* [Robot.MIN\_OMEGA](../src/robot.py)

> In order to avoid obstacles, you have to eliminate the observed obstacle area from the calculated Dynamic Window above. But this robot does not implement this obstacle avoidance feature.

#### (2) sample the control inputs
The calculated Dynamic Window is sampled at a certain resolution. These sampled values are candidates for the next control input.

* [DWAwoObstacle.V\_RESOLUTION](../src/planner.py)
* [DWAwoObstacle.OMEG4\_RESOLUTION](../src/planner.py)

#### (3) evaluate the sampled control inputs
A next robot pose is calculated by using each candidate control input, and each cost value is evaluated by using below cost functions based on the calculated each robot pose. 

* [DWAwoObstacle.\_eval\_heading()](../src/planner.py)
  * the difference angle between the direction to destination and candidate pose's theta
* [DWAwoObstacle.\_eval\_velocity()](../src/planner.py)
  * the difference velocity between max linear velocity and next linear velocity
* [DWAwoObstacle.\_eval\_distance()](../src/planner.py)
  * the distance between the destination and candidate pose
* [DWAwoObstacle.\_eval\_theta()](../src/planner.py)
  * the difference angle between the destination's theta and candidate pose's theta

The calculated costs above are normalized and multiplied by the weight parameters, and add together.

* [DWAwoObstacle.ERROR\_ANGLE\_GAIN](../src/planner.py)
* [DWAwoObstacle.VELOCITY\_GAIN](../src/planner.py)
* [DWAwoObstacle.DISTANCE\_GAIN](../src/planner.py)
* [DWAwoObstacle.THETA\_GAIN](../src/planner.py)

> In order to avoid obstacles, you have to evaluate a cost of how far from obstacle too. But this robot does not implement this obstacle avoidance feature.

#### (4) select the next control input
The lowest cost candidate is selected as the next control input.

