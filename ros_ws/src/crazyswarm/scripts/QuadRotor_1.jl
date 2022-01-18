import Pkg; Pkg.activate(@__DIR__); Pkg.instantiate();
using RobotDynamics, Rotations
using TrajectoryOptimization
using StaticArrays, LinearAlgebra
#using TrajOptPlots
#using MeshCat
#using Plots
#using FileIO, MeshIO
using Altro
#using PyPlot
#using Distributions

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
N = 51             # number of knot points
tf = 10.0               # total time (sec)
dt = tf/(N-1)          # time step (sec)

x0_pos = SA[0, -10, 1.]
xf_pos = SA[0, +10, 1.]
x0 = RobotDynamics.build_state(model, x0_pos, UnitQuaternion(I), zeros(3), zeros(3))
xf = RobotDynamics.build_state(model, xf_pos, UnitQuaternion(I), zeros(3), zeros(3));

x0 = RBState([0,0,1.5], UnitQuaternion(I), zeros(3), zeros(3))
xf = RBState([1,3,2.5], UnitQuaternion(I), zeros(3), zeros(3))

# objective
Q = Diagonal(@SVector fill(0.1, n))
R = Diagonal(@SVector fill(0.01, m))
Qf = Diagonal(@SVector fill(100.0, n))
obj = LQRObjective(Q,R,Qf,xf,N)

# constraints
cons = ConstraintList(n,m,N)
add_constraint!(cons, BoundConstraint(n,m, u_min=zeros(4), u_max=fill(10.0,4)), 1:N-1)
#add_constraint!(cons, CircleConstraint(n, SA_F64[1,2], SA_F64[1,2], SA[0.1,0.1]), 1:N-1)
add_constraint!(cons, GoalConstraint(xf, SA[1,2,3]), N)

# problem
prob = Problem(model, obj, xf, tf, x0=x0, constraints=cons);

u0 = @SVector fill(0.5*model.mass/m, m)
U_hover = [copy(u0) for k = 1:N-1]; # initial hovering control trajectory

initial_controls!(prob, U_hover)
rollout!(prob);

opts = SolverOptions(
    penalty_scaling=100.,
    penalty_initial=0.1,
)

solver = ALTROSolver(prob, opts);
solve!(solver)
println("Cost: ", cost(solver))
println("Constraint violation: ", max_violation(solver))
println("Iterations: ", iterations(solver))

X = states(solver);
U = controls(solver);
X = hcat(Vector.(X)...);
X[1:3,:]
#x = X[1,:]
#y = X[2,:]
#z = X[3,:];
