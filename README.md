# [WIP] Quadruped locomotion

*Note: This project is a work-in-progress.*

## Current Features
- Whole-Body Control based on Hierarchical Optimization
- Zero-Moment-Point Center-of-Mass Motion Planner
- Low-level joint force control
- Differential inverse kinematics control
- Support consistent differential inverse kinematics control


## Current plan
- [X] Calculate feed forward torques to pass to joint controller
- [ ] Generalize to arbitrary feet in contact for HoQpController
- [ ] Implement end-effector task for HoQpController
- [ ] Fix support polygons from ZMP motion planner
- [ ] Fix gait sequence feet order
- [ ] Get end-effector trajectory from ZMP motion planner
- [ ] Get A and b from polygon
- [ ] Implement zmp constraints
- [ ] Implement current position as constraint on first spline
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
