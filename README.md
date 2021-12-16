# Quadruped locomotion

## Todos
- [x] Test joint control
- [x] Tune position PIDs
- [] Tune velocity PIDs
- [] Add base pose and velocity to generalized coords and vels
- [] Implement inverse kinematics trajectory optimization to make the robot stand up

## HO QP
- [x] Download and install drake as a dependency
- [] Test Optimization of one task, toy task
- [] Implement dynamic constraint task (will require to somehow get dynamics here)
- [] Implement controller torques
- [] Implement more tasks
- [] Formulate HO QP optimization in general
- ... ?

## ZMP Motion planner
- ...?

[##](##) Dependencies 
The lines
```
		<gazebo>
			<plugin name="anymal_plugin" filename="libanymal_plugin.so"/>
		</gazebo>
```

must be added to the anymal_c_simple_description .urdf file for this to work.
