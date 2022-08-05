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

struct Human_QuadRotors{T} <: AbstractModel
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

Human_QuadRotors() = Human_QuadRotors(mass,J,Jinv,gravity,motor_dist,kf,km)

function RobotDynamics.dynamics(model::Human_QuadRotors, x, u)
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


    # Calculate motor forces
    F1_1 = max(0,kf*w1_1);
    F2_1 = max(0,kf*w2_1);
    F3_1 = max(0,kf*w3_1);
    F4_1 = max(0,kf*w4_1);


    F_1_b = @SVector [0., 0., F1_1+F2_1+F3_1+F4_1] #total rotor force in body frame

    #F = [m*g + q_1*F_1; m*g + q_2*F_2] # forces in world frame

    # Calculate motor torques
    M1_1 = km*w1_1;
    M2_1 = km*w2_1;
    M3_1 = km*w3_1;
    M4_1 = km*w4_1;

    tau_1 = @SVector [L*(F2_1-F4_1), L*(F3_1-F1_1), (M1_1-M2_1+M3_1-M4_1)] #total rotor torque in body frame

    #tau = [tau_1;tau_2]

    r1 = x[1:3]
    q1 = UnitQuaternion(x[4],x[5],x[6],x[7], false)
    v1 = x[8:10]
    w1 = x[11:13]


    F1 = m*g + q1*F_1_b

    q1dot = Rotations.kinematics(q1,w1)

    r1dot = v1
    v1dot = F1 ./m

    w1dot = Jinv*(tau_1 - w1 × (J*w1))


    humans_pose_dot = [u[5]*cos(x[17]); u[5]*sin(x[17]); 0; u[6]; u[7]*cos(x[21]); u[7]*sin(x[21]); 0; u[8]]

    [r1dot; q1dot; v1dot; w1dot; humans_pose_dot]
end

RobotDynamics.state_dim(::Human_QuadRotors) = 21
RobotDynamics.control_dim(::Human_QuadRotors) = 8

import TrajectoryOptimization.AbstractConstraint
import TrajectoryOptimization.StageConstraint

import RobotDynamics: state_dim, control_dim

struct Cylinder{D} <: TrajectoryOptimization.StateConstraint
	n::Int
    x1::SVector{D,Int}
    x2::SVector{D,Int}
    radius::Float64
    height::Float64
    function Cylinder(n::Int, x1::AbstractVector{Int}, x2::AbstractVector{Int}, r::Real, h::Real)
        @assert length(x1) == length(x2) "Position dimensions must be of equal length, got $(length(x1)) and $(length(x2))"
        D = length(x1)
        new{D}(n, x1, x2, r, h)
    end
end

@inline state_dim(con::Cylinder) = con.n
@inline TrajectoryOptimization.sense(::Cylinder) = Inequality()
@inline Base.length(::Cylinder) = 1

function TrajectoryOptimization.evaluate(con::Cylinder, x::SVector)
    x1 = x[con.x1]
    x2 = x[con.x2]
    d = x1[1:2] - x2[1:2]
    if abs(x1[3]-x2[3])>con.height
        return @SVector [-abs(x1[3]-x2[3])]
    else
        return @SVector [con.radius^2 - d'd]
    end
end

#function TrajectoryOptimization.jacobian!(∇c, con::Cylinder, x::SVector)
#    x1 = x[con.x1]
#    x2 = x[con.x2]
#    d = x1 - x2
#	∇x1 = -2d
#	∇x2 =  2d
#	∇c[1,con.x1] .= ∇x1
#	∇c[1,con.x2] .= ∇x2
#	return false
#end

#function TrajectoryOptimization.change_dimension(con::Cylinder, n::Int, m::Int, ix=1:n, iu=1:m)
#	Cylinder(n, ix[con.x1], ix[con.x2], con.radius)
#end

import TrajectoryOptimization.AbstractConstraint
import TrajectoryOptimization.StageConstraint

import RobotDynamics: state_dim, control_dim

model = Human_QuadRotors()
n,m = size(model)
N = 51               # number of knot points
tf = 10.0              # total time (sec)
dt = tf/(N-1)

cons = ConstraintList(n,m,N)
u_quad = 3
add_constraint!(cons, BoundConstraint(n,m, u_min=[0,0,0,0,-1,-1,-1,-1],
                    u_max=[u_quad,u_quad,u_quad,u_quad, 1,1,1,1]), 1:N-1)
#add_constraint!(cons, GoalConstraint(xf, SA[1,3,1]), N)

#collision_avoidance_1 = CollisionConstraint(n, SA[1,2,3], SA[14,15,16], 0.75)
#add_constraint!(cons,collision_avoidance_1,1:N-1)

threshold = 1

#obj_l = 0.5*sqrt(2)

collision_avoidance_1 = CollisionConstraint(n, SA[14,15], SA[18,19], threshold)
add_constraint!(cons,collision_avoidance_1,1:N-1)

collision_avoidance_2 = Cylinder(n, SA[14,15,16], SA[1,2,3], threshold,1)
add_constraint!(cons,collision_avoidance_2,1:N-1)

collision_avoidance_3 = Cylinder(n, SA[18,19,20], SA[1,2,3], threshold,1)
add_constraint!(cons,collision_avoidance_3,1:N-1)


x0 = SA[-2.0,0,1,1,0,0,0,0,0,0,0,0,0,
                        0,-2,1,pi/2, 0,2,1,3*pi/2]
xf = SA[2.0,0,1,1,0,0,0,0,0,0,0,0,0,
                        0,2,1,pi/2, 0,-2,1,3*pi/2];

Q = Diagonal(@SVector fill(0.1, n))
R = Diagonal(@SVector fill(0.01, m))
Qf = Diagonal(@SVector fill(100.0, n))
obj = LQRObjective(Q,R,Qf,xf,N)

prob = Problem(model, obj, xf, tf, x0=x0, constraints=cons);
u0 = @SVector fill(0.5*model.mass/m, m)
U_hover = [copy(u0) for k = 1:N-1];
#U_hover = U

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
println("Cost: ", cost(solver))
println("Constraint violation: ", max_violation(solver))
println("Iterations: ", iterations(solver))
print((solver.stats).status)

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

    h1 = x_update[8:11]
    h2 = x_update[12:15]


    x0 = [r1; q1_add; v1; w1; h1; h2]

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

N_mpc = 6

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
