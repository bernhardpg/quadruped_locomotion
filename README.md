# Quadruped locomotion

## Features
- Low-level joint force control
- Differential inverse kinematics control
- Support consistent differential inverse kinematics control
- TODO: COM zero moment point motion planner for any gait sequence
- TODO: Hierarchical Optimization control of joint accelerations

## HO
- [X] Clean up HOQP class
- [ ] Make sure that all indices of task matrices are correct
- [ ] Implement equality constraints
- [ ] Solve a single task
- [ ] Solve all tasks in order
- [ ] Implement higher-level solutions instead of placeholder functions

## ZMP Motion planner
- [X] Specify a format for gait sequence
- [X] Get polygons from vel_cmd and gait sequence
- [ ] Get A and b from polygon
- [ ] Implement zmp constraints
- [ ] Implement current position as constraint on first spline
- [ ] (Make constraint on final position a soft one)
- [ ] (Implement path regularizer?)

[##](##) Dependencies 
The lines
```
		<gazebo>
			<plugin name="anymal_plugin" filename="libanymal_plugin.so"/>
		</gazebo>
```

must be added to the anymal_c_simple_description .urdf file for this to work.
