module mujoco_sim

using Base: RefValue, @lock, @lock_nofail, @propagate_inbounds 
# Stdlib
using Printf: @printf 

# 3rd party
using GLFW: GLFW, Window, Key, Action, MouseButton, GetKey, RELEASE, PRESS, REPEAT
using PrettyTables: pretty_table
using BangBang: @set!!
using StaticArrays: SVector, MVector
using DocStringExtensions
using Observables: AbstractObservable, Observable, on, off
using FFMPEG
using UnsafeArrays
using Shapes

# Lyceum
using MuJoCo, MuJoCo.MJCore 
using MuJoCo.MJCore: mjtNum
using LyceumBase: LyceumBase, Maybe, AbsVec, AbsMat, @mustimplement
 
const RealVec = AbstractVector{<:Real}

include("mjsim.jl")
export MJSim, zeroctrl!, zerofullctrl!, forward!, reset!
export setaction!, getaction!, getstate!, getaction, getstate
export simulate


const FONTSCALE = MJCore.FONTSCALE_150 # can be 100, 150, 200
const MAXGEOM = 10000 # preallocated geom array in mjvScene
const MIN_REFRESHRATE = 30 # minimum rate when sim cannot run at the native refresh rate
const SIMGAMMA = 0.99
const RNDGAMMA = 0.9
const VIDFPS = 40

const RES_HD = (1280, 720)
const RES_FHD = (1920, 1080)
const RES_XGA = (1024, 768)
const RES_SXGA = (1280, 1024)


include("util.jl")
include("glfw.jl")
include("ratetimer.jl")
include("types.jl")
include("functions.jl")
include("modes.jl")
include("defaulthandlers.jl")

 

 
function simulate(
    model::Union{AbstractString,MJSim };
    trajectories::Union{Nothing,AbsMat,AbsVec{<:AbsMat}} = nothing,
    controller = nothing,
    windowsize::NTuple{2,Integer} = default_windowsize(),
    mode = "active"
)
    model isa AbstractString && (model = MJSim(model))
    reset!(model) 

    modes = EngineMode[]
    mode == "active" && push!(modes, Controller(controller))
    mode == "passive" && push!(modes, PassiveDynamics())

    run(Engine(windowsize, model, Tuple(modes)))
    return
end


function run(e::Engine) 

    # render first frame before opening window
    prepare!(e)
    render(e)
    e.ui.refreshrate = GetRefreshRate()
    e.ui.lastrender = time()
    GLFW.ShowWindow(e.mngr.state.window)

    # run the simulation/mode in second thread
    modetask = Threads.@spawn runphysics(e)

    # println(ASCII)
    # println("Press \"F1\" to show the help message.")

    runui(e)
    wait(modetask)
    return
end


function runui(e::Engine)
    shouldexit = false
    trecord = 0.0
    try
        while !shouldexit
            @lock e.phys.lock begin
                GLFW.PollEvents()
                prepare!(e)
            end

            render(e)
            trender = time()

            rt = 1 / (trender - e.ui.lastrender)
            @lock e.ui.lock begin
                e.ui.refreshrate = RNDGAMMA * e.ui.refreshrate + (1 - RNDGAMMA) * rt
                e.ui.lastrender = trender
                shouldexit = e.ui.shouldexit |= GLFW.WindowShouldClose(e.mngr.state.window)
            end

            tnow = time()
            if e.ffmpeghandle !== nothing && tnow - trecord > 1 / e.min_refreshrate
                trecord = tnow
                recordframe(e)
            end

            yield()
        end
    finally
        @lock e.ui.lock begin
            e.ui.shouldexit = true
        end
        GLFW.DestroyWindow(e.mngr.state.window)
    end
    return
end

function prepare!(e::Engine)
    ui, p = e.ui, e.phys
    sim = getsim(p.model) 
    mjv_updateScene(
        sim.m,
        sim.d,
        ui.vopt,
        p.pert,
        ui.cam,
        MJCore.mjCAT_ALL,
        ui.scn,
    )
    prepare!(ui, p, mode(e))
    return e
end

function render(e::Engine)
    w, h = GLFW.GetFramebufferSize(e.mngr.state.window)
    rect = mjrRect(Cint(0), Cint(0), Cint(w), Cint(h))
    mjr_render(rect, e.ui.scn, e.ui.con)
    e.ui.showinfo && overlay_info(rect, e)
    GLFW.SwapBuffers(e.mngr.state.window)
    return
end


function runphysics(e::Engine)
    p = e.phys
    ui = e.ui
    resettime!(p) # reset sim and world clocks to 0

    try
        while true
            shouldexit, lastrender, reversed, paused, refrate, = @lock_nofail ui.lock begin
                ui.shouldexit, ui.lastrender, ui.reversed, ui.paused, ui.refreshrate
            end

            if shouldexit
                break
            elseif (time() - lastrender) > 1 / e.min_refreshrate
                yield()
                continue
            else
                @lock p.lock begin
                    elapsedworld = time(p.timer)

                    # advance sim
                    if ui.paused
                        pausestep!(p, mode(e))
                    elseif ui.reversed && p.elapsedsim > elapsedworld
                        reversestep!(p, mode(e))
                        p.elapsedsim -= timestep(p.model)
                    elseif !ui.reversed && p.elapsedsim < elapsedworld
                        forwardstep!(p, mode(e))
                        p.elapsedsim += timestep(p.model)
                    end
                end
            end
        end
    finally
        @lock ui.lock begin
            ui.shouldexit = true
        end
    end

    return
end

end # module
