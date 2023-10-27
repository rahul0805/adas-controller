from sim import *
from utils import *

def nmpc_controller(kappa_table = None):
    T = 3 ## TODO, design your own planning horizon. 
    N = 30 ## TODO
    h = T/N ## TODO
    ###################### Modeling Start ######################
    # system dimensions
    Dim_state = 6
    Dim_ctrl  = 2
    Dim_aux   = 4 

    xm = ca.MX.sym('xm', (Dim_state, 1))
    um = ca.MX.sym('um', (Dim_ctrl, 1))
    zm = ca.MX.sym('zm', (Dim_aux, 1)) # F_yf F_yr | F_muf F_mur

    ## rename the control inputs
    Fx = um[0]
    delta = um[1]

    ## air drag
    Fd = param["Frr"] + param["Cd"] * xm[0]**2
    Fd = Fd * ca.tanh(- xm[0] * 100)
    Fb = 0.0
    ## 
    
    af, ar   = get_slip_angle(xm[0], xm[1], xm[2], delta, param) ## TODO: refer to the car simulator to get the slip angle
    Fzf, Fzr = normal_load(Fx, param) ## TODO: refer to the car simulator to get the normal load distribution
    Fxf, Fxr = chi_fr(Fx) ## TODO: refer to the car simulator to get the tire force distribution

    ##
    Fxf = ca.if_else( ca.fabs(Fxf) >= param["mu_f"] * ca.fabs(Fzf), 
                     param["mu_f"] * ca.fabs(Fzf) * ca.sign(Fxf) ,
                       Fxf)
    
    Fxr = ca.if_else( ca.fabs(Fxr) >= param["mu_r"] * ca.fabs(Fzr), 
                     param["mu_r"] * ca.fabs(Fzr) * ca.sign(Fxr) ,
                       Fxr)
    
    Fyf = tire_model_ctrl(af, Fzf, Fxf, param["C_alpha_f"], param["mu_f"]) ## TODO: use the modified tire force model tire_model_ctrl()
    Fyr = tire_model_ctrl(ar, Fzr, Fxr, param["C_alpha_r"], param["mu_r"]) ## TODO: use the modified tire force model tire_model_ctrl()

    # do not distrub these
    dUx  = (Fxf * ca.cos(delta) - zm[0] * ca.sin(delta) + Fxr + Fd) / param["m"] + xm[2] * xm[1]
    dUy  = (zm[0] * ca.cos(delta) + Fxf * ca.sin(delta) + zm[1] + Fb) / param["m"] - xm[2] * xm[0] ## TODO: refer to car simulator, replace Fxf and Fyf with auxiliary variable
    dr   = (param["L_f"] * (zm[0] * ca.cos(delta) + Fxf * ca.sin(delta)) - param["L_r"] * zm[1]) / param["Izz"] ## TODO: refer to car simulator, replace Fxf and Fyf with auxiliary variable
    
    dx   = ca.cos(xm[5]) * xm[0] - ca.sin(xm[5]) * xm[1]
    dy   = ca.sin(xm[5]) * xm[0] + ca.cos(xm[5]) * xm[1]
    dyaw = xm[2]
    xdot = ca.vertcat(dUx, dUy, dr, dx, dy, dyaw)

    xkp1 = xdot * h + xm
    Fun_dynmaics_dt = ca.Function('f_dt', [xm, um, zm], [xkp1])

    # enforce constraints for auxiliary variable z[0] = Fyf
    alg  = ca.vertcat(zm[0]-Fyf, zm[1]-Fyr) # TODO, # TODO)
    Fun_alg = ca.Function('alg', [xm, um, zm], [alg])
    
    ###################### MPC variables ######################
    x = ca.MX.sym('x', (Dim_state, N + 1))
    u = ca.MX.sym('u', (Dim_ctrl, N))
    z = ca.MX.sym('z', (Dim_aux, N))
    p = ca.MX.sym('p', (Dim_state, 1))
    
    



    ###################### MPC constraints start ######################
    ## MPC equality constraints ##
    cons_dynamics = []
    for k in range(N):
        xkp1 = Fun_dynmaics_dt(x[:, k], u[:, k], z[:, k])
        Fy2  = Fun_alg(x[:, k], u[:, k], z[:, k])
        for j in range(Dim_state):
            cons_dynamics.append(x[j, k+1] - xkp1[j])
        for j in range(2):
            cons_dynamics.append(Fy2[j])
    
    ## MPC inequality constraints ##
    # G(x) <= 0
    cons_ineq = []

    ## state / inputs limits:
    # for k in range(N):
    #     cons_ineq.append(2-x[0,k])## TODO: minimal longitudinal speed
    #     cons_ineq.append((u[0,k]*x[0,k])-(param["Peng"]))## TODO: engine power limits
    #     cons_ineq.append(1 - (((x[3,k]-500)/10)**2) - (((x[4,k])/10)**2) )## TODO: collision avoidance

    ## friction cone constraints
    for k in range(N):
        Fx    = u[0, k]; delta = u[1, k]
        af, ar = get_slip_angle(x[0,k], x[1,k], x[2,k], delta, param) ## TODO 
        Fzf, Fzr = normal_load(Fx, param) ## TODO 
        Fxf, Fxr = chi_fr(Fx) ## TODO 

        Fxf = ca.if_else( ca.fabs(Fxf) >= param["mu_f"] * ca.fabs(Fzf), 
                        param["mu_f"] * ca.fabs(Fzf) * ca.sign(Fxf) ,
                        Fxf)
        
        Fxr = ca.if_else( ca.fabs(Fxr) >= param["mu_r"] * ca.fabs(Fzr), 
                        param["mu_r"] * ca.fabs(Fzr) * ca.sign(Fxr) ,
                        Fxr)

        Fyf = tire_model_ctrl(af, Fzf, Fxf, param["C_alpha_f"], param["mu_f"]) ## TODO: use tire_model_ctrl or auxiliary variable z[2, k] z[3, k]
        Fyr = tire_model_ctrl(ar, Fzr, Fxr, param["C_alpha_r"], param["mu_r"]) ## TODO: use tire_model_ctrl or auxiliary variable z[2, k] z[3, k]

        cons_ineq.append((z[0,k]**2 + Fxf**2) - ((param["mu_f"]*Fzf)**2 + z[2,k]**2))## TODO: front tire limits)
        cons_ineq.append((z[1,k]**2 + Fxr**2) - ((param["mu_r"]*Fzr)**2 + z[3,k]**2))## TODO: reat  tire limits)        

    ###################### MPC cost start ######################
    ter_lambda = 0.0 
    ter_lambda1 =0.0
    ter_lambda2 = 0.0 
    ter_lambda3 = 0.0

    stg_lambda = 0.0 
    stg_lambda1 = 0.0
    stg_lambda2 = 0.0 
    stg_lambda3 = 0.0

    w_alpha_f = 1e8 #1e8
    w_alpha_r = 1e8 #1e8
    z_alpha_f = 1e8
    z_alpha_r = 1e8
    
    ## cost function design
    J = 0.0
    J = J + (ter_lambda*(x[4,-1]**2) + ter_lambda1*(x[5,-1]**2) + ter_lambda2*(((x[0,-1]-1e3)**2)) + ter_lambda3*(((x[3,-1]-1e3)**2)) )## TODO terminal cost
    
    ## road tracking 
    for k in range(N):
        J = J + (stg_lambda*(x[4,k]**2) + stg_lambda1*(x[5,k]**2) + stg_lambda2*((x[0,k]-1e3)**2) + stg_lambda3*(((x[3,k]-1e3)**2) )) + (u[0,k]**2 + u[1,k]**2) ## stage cost
 
    ## excessive slip angle / friction
    for k in range(N):
        Fx = u[0, k]; delta = u[1, k]
        af, ar = get_slip_angle(x[0, k], x[1,k], x[2,k], delta, param) # TODO)
        Fzf, Fzr = normal_load(Fx, param)
        Fxf, Fxr = chi_fr(Fx)

        xi = 0.85
        F_offset = 2000 ## a slacked ReLU function using only sqrt()
        Fyf_max_sq = (param["mu_f"] * Fzf)**2 - (0.999 * Fxf)**2
        Fyf_max_sq = (ca.sqrt( Fyf_max_sq**2 + F_offset) + Fyf_max_sq) / 2
        Fyf_max = ca.sqrt(Fyf_max_sq)

        ## modified front slide sliping angle
        alpha_mod_f = ca.arctan(3 * Fyf_max / param["C_alpha_f"] * xi)

        Fyr_max_sq = (param["mu_f"] * Fzf)**2 - (0.999 * Fxf)**2 # check this once
        Fyr_max_sq = (ca.sqrt( Fyr_max_sq**2 + F_offset) + Fyr_max_sq) / 2
        Fyr_max = ca.sqrt(Fyr_max_sq)

        ## modified rear slide sliping angle
        alpha_mod_r = ca.arctan(3 * Fyr_max / param["C_alpha_r"] * xi)
        
        # alpha_f_cost = ca.if_else(ca.fabs(af)>alpha_mod_f, (ca.fabs(af)-alpha_mod_f)**2 , 0)

        # alpha_r_cost = ca.if_else(ca.fabs(ar)>alpha_mod_r, (ca.fabs(ar)-alpha_mod_r)**2 , 0)

        ## limit friction penalty
        J = J + w_alpha_f*ca.if_else(ca.fabs(af)>alpha_mod_f, (ca.fabs(af)-alpha_mod_f)**2 , 0) ## TODO: Avoid front tire saturation
        J = J + w_alpha_r*ca.if_else(ca.fabs(ar)>alpha_mod_r, (ca.fabs(ar)-alpha_mod_r)**2 , 0)  ## TODO: Avoid  rear tire saturation
        J = J + z_alpha_f*(z[2,k]**2) + z_alpha_r*(z[3,k]**2) ## TODO: Penalize slack variable for friction cone limits

    # initial condition as parameters
    cons_init = [x[:, 0] - p]
    ub_init_cons = np.zeros((Dim_state, 1))
    lb_init_cons = np.zeros((Dim_state, 1))

    state_ub = np.array([ 1e2,  1e2,  1e2,  1e8,  1e8,  1e8])
    state_lb = np.array([-1e2, -1e2, -1e2, -1e8, -1e8, -1e8])
    ctrl_ub  = np.array([1e8, param["delta_max"]])## TODO,   ## TODO]) ## traction force | steering angle, use param["delta_max"]
    ctrl_lb  = np.array([-1e8, -param["delta_max"]])## TODO,   ## TODO])
    aux_ub   = np.array([ 1e5,  1e5,  1e5,  1e5])
    aux_lb   = np.array([-1e5, -1e5, -1e5, -1e5])

    lb_dynamics = np.zeros((len(cons_dynamics), 1))
    ub_dynamics = np.zeros((len(cons_dynamics), 1))

    lb_ineq = np.zeros((len(cons_ineq), 1)) - 1e9
    ub_ineq = np.zeros((len(cons_ineq), 1))

    ub_x = np.matlib.repmat(state_ub, N + 1, 1)
    lb_x = np.matlib.repmat(state_lb, N + 1, 1)
    ub_u = np.matlib.repmat(ctrl_ub, N, 1)
    lb_u = np.matlib.repmat(ctrl_lb, N, 1)
    ub_z = np.matlib.repmat(aux_ub, N, 1)
    lb_z = np.matlib.repmat(aux_lb, N, 1)

    lb_var = np.concatenate((lb_u.reshape((Dim_ctrl * N, 1)), 
                             lb_x.reshape((Dim_state * (N+1), 1)),
                             lb_z.reshape((Dim_aux * N, 1))
                             ))

    ub_var = np.concatenate((ub_u.reshape((Dim_ctrl * N, 1)), 
                             ub_x.reshape((Dim_state * (N+1), 1)),
                             ub_z.reshape((Dim_aux * N, 1))
                             ))

    vars_NLP   = ca.vertcat(u.reshape((Dim_ctrl * N, 1)), x.reshape((Dim_state * (N+1), 1)), z.reshape((Dim_aux * N, 1)))
    # cons_NLP = cons_dynamics + cons_ineq + cons_init
    cons_NLP = cons_dynamics + cons_init
    cons_NLP = ca.vertcat(*cons_NLP)
    # lb_cons = np.concatenate((lb_dynamics, lb_ineq, lb_init_cons))
    # ub_cons = np.concatenate((ub_dynamics, ub_ineq, ub_init_cons))
    lb_cons = np.concatenate((lb_dynamics, lb_init_cons))
    ub_cons = np.concatenate((ub_dynamics, ub_init_cons))

    prob = {"x": vars_NLP, "p":p, "f": J, "g":cons_NLP}

    return prob, N, vars_NLP.shape[0], cons_NLP.shape[0], p.shape[0], lb_var, ub_var, lb_cons, ub_cons