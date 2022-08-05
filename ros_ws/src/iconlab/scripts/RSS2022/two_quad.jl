module Julia_Functions

import Pkg; Pkg.activate(@__DIR__); Pkg.instantiate();
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

struct Two_QuadRotors{T} <: AbstractModel
    mass::T
    J::Diagonal{T,SVector{3,T}}
    Jinv::Diagonal{T,SVector{3,T}}
    gravity::SVector{3,T}
    motor_dist::T
    kf::T
    km::T
end

mass=0.5
J=Diagonal(@SVector [0.0023, 0.0023, 0.004])
Jinv = inv(J)
gravity=SVector(0,0,-9.81)
motor_dist=0.1750
kf=1.0
km=0.0245

Two_QuadRotors() = Two_QuadRotors(mass,J,Jinv,gravity,motor_dist,kf,km)

function RobotDynamics.dynamics(model::Two_QuadRotors, x, u)
    m = model.mass
    J = model.J
    Jinv = model.Jinv
    g = model.gravity
    L = model.motor_dist
    kf = model.kf
    km = model.km

    # Extract motor speeds
    w1_1 = u[1]
    w2_1 = u[2]
    w3_1 = u[3]
    w4_1 = u[4]

    w1_2 = u[5]
    w2_2 = u[6]
    w3_2 = u[7]
    w4_2 = u[8]

    # Calculate motor forces
    F1_1 = max(0,kf*w1_1);
    F2_1 = max(0,kf*w2_1);
    F3_1 = max(0,kf*w3_1);
    F4_1 = max(0,kf*w4_1);

    F1_2 = max(0,kf*w1_2);
    F2_2 = max(0,kf*w2_2);
    F3_2 = max(0,kf*w3_2);
    F4_2 = max(0,kf*w4_2);

    F_1_b = @SVector [0., 0., F1_1+F2_1+F3_1+F4_1] #total rotor force in body frame
    F_2_b = @SVector [0., 0., F1_2+F2_2+F3_2+F4_2] #total rotor force in body frame

    #F = [m*g + q_1*F_1; m*g + q_2*F_2] # forces in world frame

    # Calculate motor torques
    M1_1 = km*w1_1;
    M2_1 = km*w2_1;
    M3_1 = km*w3_1;
    M4_1 = km*w4_1;

    M1_2 = km*w1_2;
    M2_2 = km*w2_2;
    M3_2 = km*w3_2;
    M4_2 = km*w4_2;

    tau_1 = @SVector [L*(F2_1-F4_1), L*(F3_1-F1_1), (M1_1-M2_1+M3_1-M4_1)] #total rotor torque in body frame
    tau_2 = @SVector [L*(F2_2-F4_2), L*(F3_2-F1_2), (M1_2-M2_2+M3_2-M4_2)] #total rotor torque in body frame

    #tau = [tau_1;tau_2]

    r1 = x[1:3]
    q1 = UnitQuaternion(x[4],x[5],x[6],x[7], false)
    v1 = x[8:10]
    w1 = x[11:13]

    r2 = x[14:16]
    q2 = UnitQuaternion(x[17],x[18],x[19],x[20], false)
    v2 = x[21:23]
    w2 = x[24:26]

    F1 = m*g + q1*F_1_b
    F2 = m*g + q2*F_2_b

    q1dot = Rotations.kinematics(q1,w1)
    q2dot = Rotations.kinematics(q2,w2)

    r1dot = v1
    v1dot = F1 ./m

    w1dot = Jinv*(tau_1 - w1 × (J*w1))

    r2dot = v2
    v2dot = F2 ./m

    w2dot = Jinv*(tau_2 - w2 × (J*w2))

    [r1dot; q1dot; v1dot; w1dot; r2dot; q2dot; v2dot; w2dot]
end

RobotDynamics.state_dim(::Two_QuadRotors) = 26
RobotDynamics.control_dim(::Two_QuadRotors) = 8

model = Two_QuadRotors()
n,m = size(model)
N = 101                # number of knot points
tf = 10.0               # total time (sec)
dt = tf/(N-1)

x0 = SA[2.0,0,1,1,0,0,0,0,0,0,0,0,0, -2,0,1,1,0,0,0,0,0,0,0,0,0]
xf = SA[-2.0,0,1,1,0,0,0,0,0,0,0,0,0, 2,0,1,1,0,0,0,0,0,0,0,0,0];

Q = Diagonal(@SVector fill(0.1, n))
R = Diagonal(@SVector fill(0.01, m))
Qf = Diagonal(@SVector fill(100.0, n))
obj = LQRObjective(Q,R,Qf,xf,N)

cons = ConstraintList(n,m,N)
add_constraint!(cons, BoundConstraint(n,m, u_min=zeros(8), u_max=fill(10.0,8)), 1:N-1)
#add_constraint!(cons, GoalConstraint(xf, SA[1,3,1]), N)

collision_avoidance = CollisionConstraint(n, SA[1,2,3], SA[14,15,16], 0.5)
add_constraint!(cons,collision_avoidance,1:N-1)

prob = Problem(model, obj, xf, tf, x0=x0, constraints=cons);

u0 = @SVector fill(0.5*model.mass/m, m)
U_hover = [copy(u0) for k = 1:N-1];

initial_controls!(prob, U_hover)
rollout!(prob);

opts = SolverOptions(
    cost_tolerance_intermediate=1e-4,
    penalty_scaling=500.,
    penalty_initial=1e-2,
    # verbose = 1,
    projected_newton = false,
    constraint_tolerance = 1.0e-5,
    iterations = 5000,
    iterations_inner = 100,
    iterations_linesearch = 100,
    iterations_outer = 500,
)

solver = ALTROSolver(prob,opts);
solve!(solver)
println("Cost: ", cost(solver))
println("Constraint violation: ", max_violation(solver))
println("Iterations: ", iterations(solver))

function gen_tracking_problem(prob::Problem, N; Qk = 0.1, Rk = 0.01, Qfk = 2*Qk,)
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

    global prob_mpc = prob_mpc

    global X_traj

    r1 = x_update[1:3]
    q1_add = x_update[4:7]
    q1 = UnitQuaternion(x_update[4],x_update[5],x_update[6],x_update[7], false)
    q1_prev = UnitQuaternion(X_traj[i][4],X_traj[i][5],X_traj[i][6],X_traj[i][7],false)

    q1_dot = (q1-q1_prev)/dt

    ang1 = inv(q1_prev)*q1_dot

    v1 = (r1 - X_traj[i][1:3])/dt
    w1 = [ang1[3,2], ang1[1,3], ang1[2,1]]

    r2 = x_update[8:10]
    q2_add = x_update[11:14]
    q2 = UnitQuaternion(x_update[11],x_update[12],x_update[13],x_update[14], false)
    q2_prev = UnitQuaternion(X_traj[i][11],X_traj[i][12],X_traj[i][13],X_traj[i][14],false)

    q2_dot = (q2-q2_prev)/dt

    ang2 = inv(q2_prev)*q2_dot

    v2 = (r2 - X_traj[i][8:10])/dt
    w2 = [ang2[3,2], ang2[1,3], ang2[2,1]]


    x0 = [r1; q1_add; v1; w1; r2; q2_add; v2; w2]

    # Update the initial state after the dynamics are propogated.
    TrajectoryOptimization.set_initial_state!(prob_mpc, x0)
    X_traj[i+1] = x0

    global Z_track = Z_track

    # Update tracking cost
    TrajectoryOptimization.update_trajectory!(prob_mpc.obj, Z_track, k_mpc)

    # Shift the initial trajectory
    RobotDynamics.shift_fill!(prob_mpc.Z)


    # Shift the multipliers and penalties
    Altro.shift_fill!(get_constraints(altro))


    # Solve the updated problem
    Altro.solve!(altro)

    # Log the results and performance
    global iters[i,1] = iterations(altro)

    # ALTRO in ms and ECOS in s, by default
    global times[i,1] = altro.stats.tsolve

end

N_mpc = 11

global prob_mpc = gen_tracking_problem(prob, N_mpc)

print(prob_mpc)

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


altro = ALTROSolver(prob_mpc,opts_mpc)

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
