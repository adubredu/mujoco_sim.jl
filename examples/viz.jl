using Revise
using MuJoCo
using LyceumMuJoCo 
using mujoco_sim
import Distributions: Uniform
using ControlSystems
using MatrixEquations

keypath = joinpath(@__DIR__, "../key/mjkey.txt")
println(keypath)
mj_activate(keypath) 

T = 100
M = mk = 1.5
m = mp = 0.5
g = 9.81
l = lp = 0.3 

a = g/(lp*(4.0/3 - mp/(mp+mk)))
A =[[0. 1. 0. 0.];
    [0. 0. a  0.];
    [0. 0. 0. 1.];
    [0. 0. a  0.]] 
b = -1/(lp*(4.0/3 - mp/(mp+mk)))
B = [[0.]; [1.0/(mp+mk)]; [0.]; [b]]

Q = [[5. 0. 0. 0.]; [0 5. 0 0]; [0 0 5. 0]; [0 0 0 5.]]
R = 1.


function lqr(A, B, Q, R)
    G = B*inv(R)*B' 
    P = arec(A, G, Q)[1]  #continuous ricatti eqn solver
    K = inv(R)*B'*P
    return K 
end
# K = dlqr(A, B, Q, R)
K = lqr(A, B, Q, R)
# println("fxn: ",K)
# println("fxn me: ",K1)



function apply_ctrl(K,x)
    u = -K*x 
    if u > 0.0 
        return [1.]
    else
        return [-1.]
    end
end


function ctrler!(s)
    # x = [s.d.qpos; s.d.qvel]
    x = [s.d.qpos[1], s.d.qvel[1],s.d.qpos[2], s.d.qvel[2]]
    # u = [-K*x]
    u = apply_ctrl(K,x)
    # if u 

    if s.d.time < 0.0525 #randomize pole deviation initially to make things interesting
        u = [rand(Uniform(-1.,1.))]
    end
    # a = [rand(Uniform(-1.,1.))]
    # println(a)
    # sleep(0.0125)
    setaction!(s, u)
    println("state: ",s.d.qpos[2], " action: ",s.d.ctrl)
    # a
    # s.d.ctrl = a
end

m = jlModel("models/cartpole.xml")
d = jlData(m) 
sim = MJSim(m,d) 
sim.d.qpos[2] = -1.57
# print(actionspace(sim))
visualize( sim, controller = ctrler!;mode="active")




# id = mj_name2id(sim.m, MJCore.mjOBJ_BODY, "mocap2")
# m = sim.m.body_mass[id+1]
