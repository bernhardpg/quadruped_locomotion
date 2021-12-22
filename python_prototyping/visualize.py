# NOTE: this example needs RViz to be installed
# usage: start ROS master (roscore) and then run this test

import pinocchio as pin
from os.path import dirname, join, abspath

from pinocchio.visualize import RVizVisualizer

# Load the URDF model.
mesh_dir = "/home/bernhardpg/catkin_ws/src/anymal_c_simple_description/meshes/"
urdf_filename = "/home/bernhardpg/catkin_ws/src/anymal_c_simple_description/urdf/anymal.urdf"

model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_filename, mesh_dir, pin.JointModelFreeFlyer())
viz = RVizVisualizer(model, collision_model, visual_model)

# Initialize the viewer.
viz.initViewer()
viz.loadViewerModel("pinocchio")

# Display a robot configuration.
q0 = pin.neutral(model)
viz.display(q0)

input("Press enter to exit...")

viz.clean()
