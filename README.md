# Human-like optimization

A package for path planning and trajectory optimization on Franka Emika Panda robot

Schematization of the whole process:
* Point-to-point trajectory
* Wp in cartesian space
* Fix a robot joint
* Calculate IK in close-form
* Solve trajectory with spline
* Calculate area of the displacement of every joint
* Find fitness of cost function
* Optimize with search algorithm
