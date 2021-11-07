using Revise
using MuJoCo 
using mujoco_sim
import Distributions: Uniform 
using MatrixEquations

keypath = joinpath(@__DIR__, "../key/mjkey.txt") 
mj_activate(keypath) 

function lqr(A, B, Q, R)
    G = B*inv(R)*B' 
    P = arec(A, G, Q)[1]  #continuous ricatti eqn solver
    K = inv(R)*B'*P
    return K 
end 

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

K = lqr(A, B, Q, R)


function apply_ctrl(K,x)
    u = -K*x  
    if u > 1.0 
        return [1.0]
    elseif u < -1.0
        return [-1.0]
    else 
        return [u]
    end
end


function ctrler!(s) 
    x = [s.d.qpos[1], s.d.qvel[1],s.d.qpos[2], s.d.qvel[2]] 
    u = apply_ctrl(K,x) 
    if s.d.time < 0.05 #randomize pole deviation initially to make things interesting
        u = [rand(Uniform(-0.5,0.5))]
    end 
    setaction!(s, u)
    # println("state: ",s.d.qpos[2], " action: ",s.d.ctrl) 
end

m = jlModel("models/cartpole.xml")
d = jlData(m) 
sim = MJSim(m,d) 
sim.d.qpos[2] = -1.57 
simulate( sim, controller = ctrler!;mode="active")




