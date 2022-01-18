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
N = 51                # number of knot points
tf = 10.0               # total time (sec)
dt = tf/(N-1)          # time step (sec)

x0 = RBState([0,0,1], UnitQuaternion(I), zeros(3), zeros(3))
xf = RBState([-1,-3,2], UnitQuaternion(I), zeros(3), zeros(3))

# objective
Q = Diagonal(@SVector fill(0.1, n))
R = Diagonal(@SVector fill(0.01, m))
Qf = Diagonal(@SVector fill(100.0, n))
obj = LQRObjective(Q,R,Qf,xf,N)

# constraints
cons = ConstraintList(n,m,N)
add_constraint!(cons, BoundConstraint(n,m, u_min=zeros(4), u_max=fill(10.0,4)), 1:N-1)
add_constraint!(cons, GoalConstraint(xf, SA[1,2,3]), N)

# problem
prob = Problem(model, obj, xf, tf, x0=x0, constraints=cons);

u0 = @SVector fill(0.5*model.mass/m, m)
U_hover = [copy(u0) for k = 1:N-1]; # initial hovering control trajectory

#U_hover = load("/home/iconlab/iconlab/mcbhatt2/U_data.jld","U");

#print(U_hover)

U_hover = SVector{4, Float64}[[1.367968171814254, 1.3288459328855668, 1.4070908617102398, 1.4462141437387321], [1.9715547374597115, 2.016302740771755, 1.9268066202442293, 1.8820582747773127], [1.101539665993785, 1.1122955948108078, 1.0907837323752416, 1.080027653295336], [0.9785812329649376, 0.9776680989738087, 0.9794941673273266, 0.9804068966558835], [1.1292101850989422, 1.1245215555576424, 1.1338986241835711, 1.1385869472115653], [1.2141981728925115, 1.2087430632875573, 1.2196532176193535, 1.2251082719950694], [1.226494794448998, 1.222232240650811, 1.2307573491970238, 1.2350199424271557], [1.2182360120139846, 1.2157195168623933, 1.2207525124764211, 1.2232690237066424], [1.214498719096447, 1.2134448115516385, 1.2155526298777555, 1.2166065333813074], [1.2167951676792275, 1.2166873064363313, 1.2169030361237105, 1.217010897813051], [1.2208287344905977, 1.2212008525331624, 1.220456627373153, 1.2200845201657018], [1.2241163982776095, 1.2246418140785544, 1.2235909936153182, 1.2230655937401798], [1.2262920336792238, 1.226785803999927, 1.2257982724072862, 1.2253045177675832], [1.2277553631232958, 1.2281406014076521, 1.2273701315238497, 1.2269849066208238], [1.2288768258968328, 1.229143059101066, 1.2286105977251365, 1.2283443756470047], [1.229830961928901, 1.2299994402661116, 1.2296624877273563, 1.2294940188566406], [1.230668586066505, 1.2307693938866808, 1.2305677819495864, 1.2304669824190064], [1.2313980366082589, 1.2314580357481975, 1.2313380409375672, 1.2312780491961117], [1.2320228051606268, 1.232061236531608, 1.2319843770710248, 1.2319459523694354], [1.2325503753817022, 1.2325788336396764, 1.2325219202007534, 1.232493467982806], [1.2329912733626125, 1.2330155537916891, 1.2329669957765534, 1.2329427208266164], [1.2333569958225272, 1.23337927763559, 1.2333347165969604, 1.2333124397545459], [1.2336587517018904, 1.2336793809505775, 1.233638124777673, 1.2336175000356435], [1.2339068483065294, 1.2339255092319097, 1.2338881894482645, 1.2338695326060694], [1.2341104038227906, 1.2341267433878709, 1.2340940660842001, 1.2340777302164134], [1.2342772484796398, 1.2342911188912795, 1.234263379680955, 1.234249512615075], [1.2344139585717606, 1.2344254430182913, 1.2344024755632028, 1.2343909941416025], [1.2345259761025853, 1.234535321268843, 1.2345166322502137, 1.2345072898150597], [1.2346177675532075, 1.2346252976918919, 1.2346102386644313, 1.234602710986793], [1.234692986592227, 1.2346990345520883, 1.2346869398771154, 1.2346808941296725], [1.23475462108564, 1.234759486278137, 1.234749757171373, 1.2347448939628092], [1.2348051172052021, 1.2348090474027353, 1.2348011883124925, 1.234797259890199], [1.2348464806500759, 1.2348496699452716, 1.2348432926093942, 1.2348401049008293], [1.2348803581003898, 1.2348829535649506, 1.2348777636848676, 1.2348751696388238], [1.234908102769214, 1.2349102145457638, 1.2349059916299103, 1.2349038811240203], [1.2349308275938178, 1.2349325396383493, 1.234929115591899, 1.2349274046887955], [1.2349494489003654, 1.23495082951042, 1.2349480676945412, 1.234946688111845], [1.2349647225855094, 1.2349658348385575, 1.2349636093131053, 1.2349624979832279], [1.234977274070631, 1.2349781862101008, 1.2349763610092737, 1.2349754496920227], [1.2349876224980303, 1.2349884170618366, 1.234986827817505, 1.2349860339711924], [1.234996198855409, 1.2349969769418154, 1.2349954220105774, 1.2349946445278943], [1.2350033570264471, 1.2350042307922409, 1.235002485778577, 1.2350016124933072], [1.2350093764610997, 1.2350104396105166, 1.2350083160666505, 1.2350072532572183], [1.2350144558638096, 1.2350157203914138, 1.2350131925719725, 1.2350119280312268], [1.2350186983898652, 1.2350199916848168, 1.2350174053466831, 1.2350161147094239], [1.2350220960852338, 1.2350229116711917, 1.2350212749232432, 1.235020452205432], [1.2350244308944385, 1.2350238154808144, 1.2350250510130711, 1.2350256761268736], [1.2350246193165197, 1.2350210614475667, 1.235028173663761, 1.2350317192830789], [1.2350147241902587, 1.2350064721389382, 1.2350229734259566, 1.2350312370145065], [1.2349449621936592, 1.234955264417305, 1.2349346677445727, 1.2349243694146959]]

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

function gen_tracking_problem(prob::Problem, N; Qk = 0.1, Rk = 0.01, Qfk = Qk,)
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
t0 += dt
k_mpc += 1
global prob_mpc = prob_mpc
TrajectoryOptimization.set_initial_time!(prob_mpc, t0)

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


# Shift the multipliers and penalties
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

N_mpc = 6

global prob_mpc = gen_tracking_problem(prob, N_mpc)

print(prob_mpc)

opts_mpc = SolverOptions(
    penalty_scaling=5.,
    penalty_initial=2.,
    constraint_tolerance = 1e-2,
    cost_tolerance = 1e-2
    );

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








