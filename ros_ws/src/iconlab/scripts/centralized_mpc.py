import casadi as cs
import numpy as np
from scipy.constants import g
from casadi import *
import logging
from time import perf_counter

from util import (
    compute_pairwise_distance,
    compute_pairwise_distance_Sym,
    define_inter_graph_threshold,
    distance_to_goal,
    split_graph, 
    generate_f,
    objective,
    generate_min_max_input,
    generate_min_max_state
)


#Define constants for constraints
centralized = True
def solve_centralized(x0,xf,u_ref,N,Q,R,Qf,n_agents,n_states,n_inputs,radius,
             max_input,min_input,max_state,min_state):
    #N is the shifting prediction horizon
    
    
    p_opts = {"expand":True}
    s_opts = {"max_iter": 1000,"print_level":0}
    
    
    M = 100  # this is the maximum number of outer iterations
 
    n_x = n_agents*n_states
    n_u = n_agents*n_inputs
    x_dims = [n_states]*n_agents
    f = generate_f(x_dims)
    X_full = np.zeros((0, n_x))
    U_full = np.zeros((0, n_u))

    pairwise_dist = compute_pairwise_distance(x0,x_dims)
    print(f'The current pairwise distances is {pairwise_dist} \n')
    
    t = 0

    J_list = []
    J_list.append(np.inf)
    # for i in range(M) :
    i = 0
    dt = 0.1
    failed_count = 0
    converged = False

    t_solve_start = perf_counter()
   
    opti = Opti()
    
    X = opti.variable(n_x,N+1)
    U = opti.variable(n_u,N)
    
    cost_fun = objective(X,U,u_ref,xf,Q,R,Qf)
    opti.minimize(cost_fun)
    
    for k in range(N): #loop over control intervals
        # Runge-Kutta 4 integration
        k1 = f(X[:,k],         U[:,k])
        k2 = f(X[:,k]+dt/2*k1, U[:,k])
        k3 = f(X[:,k]+dt/2*k2, U[:,k])
        k4 = f(X[:,k]+dt*k3,   U[:,k])
        x_next = X[:,k] + dt/6*(k1+2*k2+2*k3+k4) 

        opti.subject_to(X[:,k+1]==x_next) # close the gaps
        
        #Constraints on inputs:
        # for j in range(max_input.T.flatten().shape[0]):
            # opti.subject_to(U[j,k] <= max_input.T.flatten()[j] )
            # opti.subject_to(min_input.T.flatten()[j] <= U[j,k] )
        opti.subject_to(U[:,k] <= max_input.T.flatten()[:] )
        opti.subject_to(min_input.T.flatten()[:] <= U[:,k] )

    #collision avoidance constraints
    for k in range(N+1):
        distances = compute_pairwise_distance_Sym(X[:,k], x_dims)
        for n in range(len(distances)):
            opti.subject_to(distances[n] >= radius)
            
        #constraints on states:
        # for m in range(max_state.T.flatten().shape[0]):
        #     opti.subject_to(X[m,k]<= max_state.T.flatten()[m] )
        #     opti.subject_to(max_state.T.flatten()[m] <= X[m,k])
        opti.subject_to(X[:,k]<= max_state.T.flatten()[:])
        opti.subject_to(max_state.T.flatten()[:] <= X[:,k])
            
            
    #equality constraints for initial condition:
    opti.subject_to(X[:,0] == x0)
    
    opti.solver("ipopt",p_opts,
                s_opts) 
    t_solve = None
    try:
        sol = opti.solve()
        t_solve_end = perf_counter()
        t_solve = t_solve_end-t_solve_start
    except RuntimeError:
        print('Current problem is infeasible \n')
        failed_count +=1
        opti.debug.show_infeasibilities()
        return X_full,U_full, t, J_list, failed_count, converged
        
    x0 = sol.value(X)[:,1].reshape(-1,1)
    u_sol = sol.value(U)[:,0]
    objective_val = sol.value(cost_fun)
    J_list.append(objective_val)
        
    print(f'current objective function value is {sol.value(cost_fun)}')
        
    #Store the trajectory
    
    X_full = np.r_[X_full, x0.reshape(1,-1)]
    U_full = np.r_[U_full, u_sol.reshape(1,-1)]
    
    t += dt
    i += 1

            
    # logging.info(
    # f'{j_trial},'
    # f'{n_agents},{t},{failed_count},{converged},'
    # f'{objective_val},{N},{dt},{radius},{centralized},{t_solve},'
    #     )
        
    return X_full,U_full, t, J_list, failed_count, converged

