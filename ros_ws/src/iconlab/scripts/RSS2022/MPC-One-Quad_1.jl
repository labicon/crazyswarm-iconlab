module Julia_Functions

#import Pkg; Pkg.activate(@__DIR__); Pkg.instantiate();
using TrajectoryOptimization
using RobotDynamics
using StaticArrays, LinearAlgebra
using Rotations
using Altro

function Rotations.getproperty(q::QuatRotation, f::Symbol)
    if f == :w
        getfield(q,:q).s
    elseif f == :x
        getfield(q,:q).v1
    elseif f == :y
        getfield(q,:q).v2
    elseif f == :z
        getfield(q,:q).v3
    else
        getfield(q,f)
    end
end

struct Quadrotor{R} <: RigidBody{R}
    n::Int
    m::Int
    mass::Float64
    J::Diagonal{Float64,SVector{3,Float64}}
    Jinv::Diagonal{Float64,SVector{3,Float64}}
    gravity::SVector{3,Float64}
    motor_dist::Float64
    kf::Float64
    km::Float64
    bodyframe::Bool  # velocity in body frame?
    info::Dict{Symbol,Any}
end

function Quadrotor{R}(;
        mass=0.5,
        J=Diagonal(@SVector [0.0023, 0.0023, 0.004]),
        gravity=SVector(0,0,-9.81),
        motor_dist=0.1750,
        kf=1.0,
        km=0.0245,
        bodyframe=false,
        info=Dict{Symbol,Any}()) where R
    Quadrotor{R}(13,4,mass,J,inv(J),gravity,motor_dist,kf,km,bodyframe,info)
end

(::Type{Quadrotor})(;kwargs...) = Quadrotor{UnitQuaternion{Float64}}(;kwargs...)

RobotDynamics.control_dim(::Quadrotor) = 4

function RobotDynamics.forces(model::Quadrotor, x, u)
    q = orientation(model, x)
    kf = model.kf
    g = model.gravity
    m = model.mass

    # Extract motor speeds
    w1 = u[1]
    w2 = u[2]
    w3 = u[3]
    w4 = u[4]

    # Calculate motor forces
    F1 = max(0,kf*w1);
    F2 = max(0,kf*w2);
    F3 = max(0,kf*w3);
    F4 = max(0,kf*w4);
    F = @SVector [0., 0., F1+F2+F3+F4] #total rotor force in body frame

    m*g + q*F # forces in world frame
end

function RobotDynamics.moments(model::Quadrotor, x, u)

    kf, km = model.kf, model.km
    L = model.motor_dist

    # Extract motor speeds
    w1 = u[1]
    w2 = u[2]
    w3 = u[3]
    w4 = u[4]

    # Calculate motor forces
    F1 = max(0,kf*w1);
    F2 = max(0,kf*w2);
    F3 = max(0,kf*w3);
    F4 = max(0,kf*w4);

    # Calculate motor torques
    M1 = km*w1;
    M2 = km*w2;
    M3 = km*w3;
    M4 = km*w4;
    tau = @SVector [L*(F2-F4), L*(F3-F1), (M1-M2+M3-M4)] #total rotor torque in body frame
end

RobotDynamics.inertia(model::Quadrotor) = model.J
RobotDynamics.inertia_inv(model::Quadrotor) = model.Jinv
RobotDynamics.mass(model::Quadrotor) = model.mass

model = Quadrotor();
n,m = size(model)
N = 101                # number of knot points
tf = 10.0               # total time (sec)
dt = tf/(N-1)          # time step (sec)

x0 = RBState([-1.5,1.5,1], UnitQuaternion(I), zeros(3), zeros(3))
xf = RBState([1.5,-1.5,1], UnitQuaternion(I), zeros(3), zeros(3))

# objective
Q = Diagonal(@SVector fill(2.0, n))
R = Diagonal(@SVector fill(0.02, m))
Qf = Diagonal(@SVector fill(100.0, n))
obj = LQRObjective(Q,R,Qf,xf,N)

# constraints
cons = ConstraintList(n,m,N)
add_constraint!(cons, BoundConstraint(n,m, u_min=zeros(4), u_max=fill(10.0,4)), 1:N-1)
add_constraint!(cons, SphereConstraint(n, SA_F64[0], SA_F64[0], SA_F64[1], SA_F64[0.3]), 1:N-1)
add_constraint!(cons, GoalConstraint(xf, SA[1,2,3]), N)

# problem
prob = Problem(model, obj, xf, tf, x0=x0, constraints=cons);

u0 = @SVector fill(0.5*model.mass/m, m)
U_hover = [copy(u0) for k = 1:N-1]; # initial hovering control trajectory

#U_hover = load("/home/iconlab/iconlab/mcbhatt2/U_data.jld","U");

#print(U_hover)


initial_controls!(prob, U_hover)
rollout!(prob);

opts = SolverOptions(
    penalty_scaling=5.,
    penalty_initial=2.,
    constraint_tolerance = 1e-2,
    cost_tolerance = 1e-2
    )

solver = ALTROSolver(prob,opts);
solve!(solver)
#println("Cost: ", cost(solver))
#println("Constraint violation: ", max_violation(solver))
#println("Iterations: ", iterations(solver))
#print((solver.stats).status)

function gen_tracking_problem(prob::Problem, N; Qk = 2.0, Rk = 0.02, Qfk = Qk,)
    n,m = size(prob)
    dt = prob.Z[1].dt
    tf = (N-1)*dt

    # Get sub-trajectory
    Z = Traj(prob.Z[1:N])
    x0 = state(Z[1])
    xf = state(Z[N])  # this actually doesn't effect anything

    # Generate a cost that tracks the trajectory
    Q = Diagonal(@SVector fill(Qk, n))
    R = Diagonal(@SVector fill(Rk, m))
    Qf = Diagonal(@SVector fill(Qfk, n))
    obj = TrajectoryOptimization.TrackingObjective(Q, R, Z, Qf=Qf)

    # Use the same constraints, except the Goal constraint
    cons = ConstraintList(n,m,N)
    for (inds, con) in zip(prob.constraints)
        if !(con isa GoalConstraint)
            if inds.stop > N
                inds = inds.start:N-(prob.N - inds.stop)
            end
            length(inds) > 0 && add_constraint!(cons, con, inds)
        end
    end

    prob = Problem(prob.model, obj, xf, tf, x0=x0, constraints=cons,
        integration=TrajectoryOptimization.integration(prob)
    )
    initial_trajectory!(prob, Z)
    return prob
end

function run_MPC_iterate(x_update,t0,k_mpc,prob_mpc,Z_track,X_traj,iters,times,i)
    # Update initial time
    #t0 += dt
    #k_mpc += 1
    global altro = altro
    global prob_mpc = prob_mpc
    #TrajectoryOptimization.set_initial_time!(prob_mpc, t0)

    global X_traj
    #print(X_traj)
    # Update initial state by using 1st control, and adding some noise
    r = x_update[1:3]
    q = UnitQuaternion(x_update[4],x_update[5],x_update[6],x_update[7], false)
    q_prev = UnitQuaternion(X_traj[i][4],X_traj[i][5],X_traj[i][6],X_traj[i][7],false)

    q_dot = (q-q_prev)/dt

    ang = inv(q_prev)*q_dot

    v = (r - X_traj[i][1:3])/dt
    w = [ang[3,2], ang[1,3], ang[2,1]]
    x0 = RBState(r, q, v, w)

    # Update the initial state after the dynamics are propogated.
    TrajectoryOptimization.set_initial_state!(prob_mpc, x0)
    X_traj[i+1] = x0

    global Z_track = Z_track

    # Update tracking cost
    TrajectoryOptimization.update_trajectory!(prob_mpc.obj, Z_track, k_mpc)

    # Shift the initial trajectory
    RobotDynamics.shift_fill!(prob_mpc.Z)


    #Shift the multipliers and penalties
    Altro.shift_fill!(get_constraints(altro))


    # Solve the updated problem
    Altro.solve!(altro)

    # Log the results and performance
    global iters[i,1] = iterations(altro)

    # ALTRO in ms and ECOS in s, by default
    global times[i,1] = altro.stats.tsolve

end

function run_MPC(prob_mpc, opts_mpc, Z_track,
                            num_iters = length(Z_track) - prob_mpc.N)
    # First, Let's generate the ALTRO problem
    altro = ALTROSolver(prob_mpc, opts_mpc)

    Qk = prob_mpc.obj[1].Q[1]
    Rk = prob_mpc.obj[1].R[1]
    Qfk = prob_mpc.obj[end].Q[1]

    # Solve initial iteration
    Altro.solve!(altro)

    err_traj = zeros(num_iters,2)
    err_x0 = zeros(num_iters,2)
    iters = zeros(Int, num_iters,2)
    times = zeros(num_iters,2)

    # Get the problem state size and control size
    n,m = size(prob_mpc)

    t0 = 0
    k_mpc = 1
    x0 = SVector(prob_mpc.x0)
    X_traj = [copy(x0) for k = 1:num_iters+1]

    # Begin the MPC LOOP
    for i = 1:num_iters
        # Update initial time
        t0 += dt
        k_mpc += 1
        TrajectoryOptimization.set_initial_time!(prob_mpc, t0)

        # Update initial state by using 1st control, and adding some noise
        x0 = discrete_dynamics(TrajectoryOptimization.integration(prob_mpc),
                                    prob_mpc.model, prob_mpc.Z[1])

        # Update the initial state after the dynamics are propogated.
        TrajectoryOptimization.set_initial_state!(prob_mpc, x0)
        X_traj[i+1] = x0

        # Update tracking cost
        TrajectoryOptimization.update_trajectory!(prob_mpc.obj, Z_track, k_mpc)

        # Shift the initial trajectory
        RobotDynamics.shift_fill!(prob_mpc.Z)

        # Shift the multipliers and penalties
        Altro.shift_fill!(get_constraints(altro))


        # Solve the updated problem
        Altro.solve!(altro)

        # Log the results and performance
        iters[i,1] = iterations(altro)

        # ALTRO in ms and ECOS in s, by default
        times[i,1] = altro.stats.tsolve

    end


    # Return the trajectory that was traced out by the MPC
    # X_ecos and U_ecos are needed to grab the ECOS variable information
    # All other logistics are in the dictionary.
    return X_traj, Dict(:time=>times, :iter=>iters)
end

N_mpc = 11

global prob_mpc = gen_tracking_problem(prob, N_mpc)

print(prob_mpc)

#opts_mpc = SolverOptions(
#    penalty_scaling=5.,
#    penalty_initial=2.,
#    constraint_tolerance = 1e-2,
#    cost_tolerance = 1e-2
#    );

opts_mpc = SolverOptions(
    cost_tolerance=1e-4,
    cost_tolerance_intermediate=1e-4,
    constraint_tolerance=1e-4,
    reset_duals=false,
    penalty_initial=1000.0,
    penalty_scaling=10.0,
    projected_newton=false
)

#Z_track = load("/home/iconlab/iconlab/mcbhatt2/data.jld2")["Z_track"];
Z_track = TrajectoryOptimization.get_trajectory(solver)
#print(Z_track)


altro = ALTROSolver(prob_mpc, opts_mpc)

num_iters = length(Z_track) - prob_mpc.N

Qk = prob_mpc.obj[1].Q[1]
Rk = prob_mpc.obj[1].R[1]
Qfk = prob_mpc.obj[end].Q[1]

# Solve initial iteration
Altro.solve!(altro)

err_traj = zeros(num_iters,2)
err_x0 = zeros(num_iters,2)
iters = zeros(Int, num_iters,2)
times = zeros(num_iters,2)

# Get the problem state size and control size
n,m = size(prob_mpc)

t0 = 0
k_mpc = 1
x0 = SVector(prob_mpc.x0)
X_traj = [copy(x0) for k = 1:num_iters+1]

integrate = TrajectoryOptimization.integration

set_initial_time = TrajectoryOptimization.set_initial_time!
end



#N_mpc = 6

#prob_mpc = gen_tracking_problem(prob, N_mpc)

#opts_mpc = SolverOptions(
#    penalty_scaling=5.,
#    penalty_initial=2.,
#    constraint_tolerance = 1e-2,
#    cost_tolerance = 1e-2
#    );

#Z_track = TrajectoryOptimization.get_trajectory(solver);

#X_traj, res = run_MPC(prob_mpc, opts_mpc, Z_track);

#using Statistics

#times = mean(res[:time], dims=1)
#print("Average time:", times[1])
