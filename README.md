# Quadruped locomotion

## Features
- Low-level joint force control
- Differential inverse kinematics control
- Support consistent differential inverse kinematics control
- TODO: COM zero moment point motion planner for any gait sequence

## Current plan
- [X] Calculate feed forward torques to pass to joint controller
- [X] Generalize to arbitrary feet in contact for HoQpController
- [X] Implement end-effector task for HoQpController
- [X] Fix support polygons from ZMP motion planner
- [X] Fix gait sequence feet order
- [X] Get end-effector trajectory from ZMP motion planner
- [ ] Get A and b from polygon
- [ ] Implement zmp constraints
- [X] Implement current position as constraint on first spline
- [ ] (Make constraint on final position a soft one)
- [ ] (Implement path regularizer?)

## Cleanup:
TODO: remove anymal_config.yaml
TODO: give all topics 'animal' prefix

[##](##) Dependencies 
The lines
```
		<gazebo>
			<plugin name="anymal_plugin" filename="libanymal_plugin.so"/>
		</gazebo>
```

must be added to the anymal_c_simple_description .urdf file for this to work.
