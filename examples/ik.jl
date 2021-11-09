using Revise
using mujoco_sim

target = ([0.5,0.,0.3], [1.,0.,0.,1.])
joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", 
                    "joint_6"]
m = get_model("models/arm.xml")
d = get_data(m) 
sim = MJSim(m,d) 

q = inverse_kinematics!(sim, "gripsite", target[1], target[2], 
        joint_names)