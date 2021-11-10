# using Revise 
using mujoco_sim
import Distributions: Uniform 
using MatrixEquations

keypath = joinpath(@__DIR__, "../key/mjkey.txt") 
activate_mujoco(keypath)

function lqr(A, B, Q, R)
    G = B*inv(R)*B' 
    P = arec(A, G, Q)[1]  #continuous ricatti eqn solver
    K = inv(R)*B'*P
    return K 
end 

T = 100
mk = 1.5
mp = 0.5
g = 9.81
lp = 0.3 
noise = 0.5

a = g/(lp*(4.0/3 - mp/(mp+mk)))
A =[[0. 1. 0. 0.];
    [0. 0. a  0.];
    [0. 0. 0. 1.];
    [0. 0. a  0.]] 
b = -1/(lp*(4.0/3 - mp/(mp+mk)))
B = [[0.]; [1.0/(mp+mk)]; [0.]; [b]]

Q = [[5. 0. 0. 0.]; 
     [0. 5. 0. 0.]; 
     [0. 0. 5. 0.]; 
     [0. 0. 0. 5.]]
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

    #randomize init pole deviation to make things interesting
    if s.d.time < 0.25
        u = [rand(Uniform(-noise, noise))]
    end 
    setaction!(s, u)
    # println("state: ",s.d.qpos[2], " action: ",s.d.ctrl) 
end

m = get_model("models/cartpole.xml")
d = get_data(m) 
sim = MJSim(m,d)   
simulate( sim, controller = ctrler!; mode="active")




