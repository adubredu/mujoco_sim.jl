using Revise
using MuJoCo
using LyceumMuJoCo 
using mjviz
# env = LyceumMuJoCo.HopperV2()
T = 100
# states = Array(undef, statespace(env), T)
# for t = 1:T
#     step!(env)
#     states[:, t] .= getstate(env)
# end
function ctrler!(s)
    a = rand(actionspace(s))
    # println(a)
    # setaction!(s, a)
    s.d.ctrl = a
end

m = jlModel("models/cartpole.xml")
d = jlData(m) 
sim = MJSim(m,d)
visualize(
    sim,
    # trajectories=[states],
    controller = ctrler!)
)
