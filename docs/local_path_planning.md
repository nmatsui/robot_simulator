## local path planning
This robot has a local path planner like DWA (Dynamic Window Approach)

#### (1) find a search area (Dynamic window)
A search area (the maximum and minimum control inputs) is calculated based on the current linear speed and angular velocity and the maximum and minimum velocity and maximum acceleration of the robot.  
This search area is called Dynamic Window.  
The default accelerations or velocities are defined in each Robot, but you can override them by your Agent.

* [Robot.MAX\_LIN\_ACC](../src/robot.py#L8)
* [Robot.MAX\_ANG\_ACC](../src/robot.py#L9)
* [Robot.MAX\_V](../src/robot.py#L11)
* [Robot.MIN\_V](../src/robot.py#L12)
* [Robot.MAX\_OMEGA](../src/robot.py#L13)
* [Robot.MIN\_OMEGA](../src/robot.py#L14)

* [WaypointsAgent.get\_max\_accelarations(self, current)](../waypoints_agent.py#L60)
* [WaypointsAgent.get\_linear\_velocities(self, current)](../waypoints_agent.py#L75)
* [WaypointsAgent.get\_angular\_velocities(self, current)](../waypoints_agent.py#L90)

> In order to avoid obstacles, you have to eliminate the observed obstacle area from the calculated Dynamic Window above. But this robot does not implement this obstacle avoidance feature.

#### (2) sample the control inputs
The calculated Dynamic Window is sampled at a certain resolution. These sampled values are candidates for the next control input.

* [DWAwoObstacle.V\_RESOLUTION](../src/planner.py#L11)
* [DWAwoObstacle.OMEG4\_RESOLUTION](../src/planner.py#L12)

#### (3) evaluate the sampled control inputs
A next robot pose is calculated by using each candidate control input, and each cost value is evaluated by using below cost functions based on the calculated each robot pose.

* [DWAwoObstacle.\_eval\_heading()](../src/planner.py#L127)
  * the difference angle between the direction to destination and candidate pose's theta
* [DWAwoObstacle.\_eval\_velocity()](../src/planner.py#L147)
  * the difference velocity between max linear velocity and next linear velocity
* [DWAwoObstacle.\_eval\_distance()](../src/planner.py#L164)
  * the distance between the destination and candidate pose
* [DWAwoObstacle.\_eval\_theta()](../src/planner.py#L183)
  * the difference angle between the destination's theta and candidate pose's theta

The calculated costs above are normalized and multiplied by the weight parameters, and add together.

* When the current position and the target position are far apart:
  * [DWAwoObstacle.FAR\_ERROR\_ANGLE\_GAIN](../src/planner.py#L14)
  * [DWAwoObstacle.FAR\_VELOCITY\_GAIN](../src/planner.py#L15)
  * [DWAwoObstacle.FAR\_DISTANCE\_GAIN](../src/planner.py#L16)
  * [DWAwoObstacle.FAR\_THETA\_GAIN](../src/planner.py#L17)
* When the current position and the target position are close:
  * [DWAwoObstacle.NEAR\_ERROR\_ANGLE\_GAIN](../src/planner.py#L18)
  * [DWAwoObstacle.NEAR\_VELOCITY\_GAIN](../src/planner.py#L19)
  * [DWAwoObstacle.NEAR\_DISTANCE\_GAIN](../src/planner.py#L20)
  * [DWAwoObstacle.NEAR\_THETA\_GAIN](../src/planner.py#L21)


> In order to avoid obstacles, you have to evaluate a cost of how far from obstacle too. But this robot does not implement this obstacle avoidance feature.

#### (4) select the next control input
The lowest cost candidate is selected as the next control input.

