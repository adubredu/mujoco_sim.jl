using LinearAlgebra, Rotations

function nullspace_method(jac_joints, delta; regularization_strength=0.0)
    hess_approx = transpose(jac_joints)*jac_joints
    joint_delta = transpose(jac_joints)*delta 

    if regularization_strength > 0.0
        hess_approx += I(size(hess_approx)[1])*regularization_strength
        return hess_approx\joint_delta
    else
        return hess_approx\joint_delta
    end
end



function inverse_kinematics!(sim::MJSim, site_name::String,
    target_pos::Union{Vector{Float64},Nothing}, 
    target_quat::Union{Vector{Float64},Nothing}=nothing, 
    joint_names=nothing,
    tol::Float64=1E-14, rot_weight::Float64=1.0, 
    regularization_threshold::Float64=0.1,
    regularization_strength::Float64=3E-2, 
    max_update_norm::Float64=2.0,
    progress_thresh::Float64=2.0, max_steps::Int64=100)

    if isnothing(target_quat)
        jac = Array{Float64}(undef, 3, sim.m.nv)#zeros((3, sim.m.nv))
        err = Array{Float64}(undef, 3)#zeros(3)
        jac_pos, jac_rot = jac, nothing 
        err_pos, err_rot = err, nothing 
    
    else 
        jac = Array{Float64}(undef, 6, sim.m.nv)#zeros((6, sim.m.nv))
        err = Array{Float64}(undef, 6)#zeros(6)
        jac_pos, jac_rot = jac[1:4], jac[4:end]
        err_pos, err_rot = err[1:4], err[4:end]
    end 

    update_nv = zeros(sim.m.nv)

    if !isnothing(target_quat)
        site_xquat = Array{Float64}(undef, 4)#zeros(4)
        neg_site_xquat = Array{Float64}(undef, 4)#zeros(4)
        err_rot_quat = Array{Float64}(undef, 4)
    end

    mj_fwdPosition(sim.m, sim.d)

    site_id = jl_name2id(sim.m, MJCore.mjOBJ_SITE, site_name)

    site_xpos = sim.d.site_xpos[:,site_id]
    site_xmat = sim.d.site_xmat[:,site_id]

    if isnothing(joint_names)
        dof_indices = sim.m.dof_jntid
    else
        dof_indices = []
        for joint_name in joint_names  
            jid = jl_name2id(sim.m, MJCore.mjOBJ_JOINT, joint_name)
            push!(dof_indices, jid)
        end
    end

    steps = 0
    success = false 

    for steps = 1:max_steps
        
        err_norm = 0.0

        if !isnothing(target_pos)
            err_pos = target_pos - site_xpos 
            err_norm += norm(err_pos)
        end

        if !isnothing(target_quat)
            # q = UnitQuaternion(SMatrix{3,3}(transpose(reshape(site_xmat, (3,3)))))
            MJCore.mju_mat2Quat(site_xquat, site_xmat)
            MJCore.mju_negQuat(neg_site_xquat, site_xquat)
            MJCore.mju_mulQuat(err_rot_quat, target_quat, neg_site_xquat)
            MJCore.mju_quat2Vel(err_rot, err_rot_quat, 1.)
            err_norm += norm(err_rot)*rot_weight
        end

        if err_norm < tol 
            success = true 
            break 
        else
            mj_jacSite(sim.m, sim.d, jac_pos, jac_rot, site_id)
            jac_joints = Array{Float64}(undef, 6, length(dof_indices))
            for i = 1:length(dof_indices)
                jac_joints[:,i] = jac[:,dof_indices[i]]
            end
 
            reg_strength = err_norm > regularization_threshold ? 
                            regularization_strength : 0.0
            
            update_joints = nullspace_method(jac_joints, err, 
                                regularization_strength=reg_strength)
            update_norm = norm(update_joints)
            println(update_joints)
            progress_criterion = err_norm/update_norm 
            if progress_criterion > progress_thresh 
                break 
            end
            if update_norm > max_update_norm 
                update_joints *= max_update_norm/update_norm 
            end
            
            for i in dof_indices
                update_nv[i] = update_joints[i]
            end

            mj_integratePos(sim.m, sim.d.qpos, update_nv, 1.)
            mj_fwdPosition(sim.m, sim.d)
        end
    end

    if !success && steps == max_steps
        println("Failed to converge")
    end

    qpos = sim.d.qpos 
    return qpos 
end


    
    
    
    



