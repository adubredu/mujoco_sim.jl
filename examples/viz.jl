using Revise
using LyceumMuJoCo 
using mjviz
env = LyceumMuJoCo.HopperV2()
T = 100
states = Array(undef, statespace(env), T)
for t = 1:T
    step!(env)
    states[:, t] .= getstate(env)
end
visualize(
    env,
    trajectories=[states],
    controller = env -> setaction!(env, rand(actionspace(env)))
)
