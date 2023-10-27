import numpy as np
import cvxpy as cp

def Setup_Derivative(param):
    ## this function is optional

    ## TODO
    

    return # TODO

def Student_Controller_LQR(x_bar, u_bar, x0, Fun_Jac_dt, param):
    # TODO
    # print(x_bar.shape[0],x_bar.shape[1])
    # print(x0, x[:,0])
    # print(param)
    
    N = x_bar.shape[0]
    x = cp.Variable((x_bar.shape[0],x_bar.shape[1]))
    u = cp.Variable((u_bar.shape[0],u_bar.shape[1]))
    
    costlist = 0.0
    constrlist = []

    
    P = np.eye(x_bar.shape[1])*30
    Q = np.eye(x_bar.shape[1])*10
    Q[2,2] = 50
    P[2,2] = 50
    Q[2,2] = 40
    P[2,2] = 40
    
    R = np.eye(u_bar.shape[1])*1
    R[1,1] = 10
    
    
    Lr = param["L_r"]
    Lf = param["L_f"]

    A = np.zeros((4,4))
    B = np.zeros((4,2))
    
    delta_t = param["h"]
    
    for i in range(x.shape[0]-1):
        
        costlist += (cp.quad_form((x[i,:]-x_bar[i,:]),Q) + cp.quad_form((u[i,:]-u_bar[i,:]), R))
        

        A[0,0] = 1.0
        A[1,1] = 1.0
        A[2,2] = 1.0
        A[3,3] = 1.0
        beta = np.arctan((Lr*np.arctan(u_bar[i,1]))/(Lr+Lf))
        inside_beta = ((Lr*np.arctan(u_bar[i,1]))/(Lr+Lf))

        A[0,2] = -x_bar[i,3]*np.sin(x_bar[i,2]+beta)*delta_t
        A[0,3] = x_bar[i,3]*np.cos(x_bar[i,2]+beta)*delta_t
        
        A[1,2] = np.cos(x_bar[i,2]+beta)*delta_t
        A[1,3] = np.sin(x_bar[i,2]+beta)*delta_t
        den1 = ((Lr*np.arctan(u_bar[i,1]))**2 + (Lr+Lf)**2)**0.5
        A[2,3] = ((np.arctan(u_bar[i,1]))/den1)*delta_t

        B[3,0] = 1*delta_t
        B[0,1] = A[0,2] * (Lr/(Lr+Lf)) * (1/(1+(u_bar[i,1]**2))) * (1/(1+((Lr+Lf)**2))) * delta_t
        B[1,1] = A[1,2] * (Lr/(Lr+Lf)) * (1/(1+(u_bar[i,1]**2))) * (1/(1+((Lr+Lf)**2))) * delta_t
        B[2,1] =  x_bar[i,3] * (1/(1+(u_bar[i,1]**2))) * (1/den1**3) * delta_t
        
        
        constrlist += [(x[i + 1, :]-x_bar[i + 1, :]) == ((A @ (x[i, :]-x_bar[i, :]) + (B @ (u[i,:] - u_bar[i,:]))))]


    costlist += (cp.quad_form((x[-1,:]-x_bar[-1,:]),P))
    constrlist += [(x[0, :] - x_bar[0, :]) == (x0 - x_bar[0, :])]

    prob = cp.Problem(cp.Minimize(costlist), constrlist)
    
    prob.solve(verbose=False)

    
    
    return u.value[0,:]
    # return u_bar[0,:]

def Student_Controller_CMPC(x_bar, u_bar, x0, Fun_Jac_dt, param):
    # TODO
    
    N = x_bar.shape[0]
    x = cp.Variable((x_bar.shape[0],x_bar.shape[1]))
    u = cp.Variable((u_bar.shape[0],u_bar.shape[1]))
    
    costlist = 0.0
    constrlist = []
    # P = np.eye(x_bar.shape[1])*10
    # Q = np.eye(x_bar.shape[1])*10
    # R = np.eye(u_bar.shape[1])
    # R[1,1] = 0.5
    
    # P = np.eye(x_bar.shape[1])*15*3
    # Q = np.eye(x_bar.shape[1])*10*2
    # R = np.eye(u_bar.shape[1])*2
    # R[1,1] = 0.5*2*2
    # P = np.eye(x_bar.shape[1])*30
    # Q = np.eye(x_bar.shape[1])*20
    # # Q[2,2] = 50
    # # P[2,2] = 50
    # Q[2,2] = 50
    # P[2,2] = 50
    
    # R = np.eye(u_bar.shape[1])
    # R[1,1] = 50

    P = np.eye(x_bar.shape[1])*30
    Q = np.eye(x_bar.shape[1])*10
    Q[2,2] = 50
    P[2,2] = 50
    Q[2,2] = 40
    P[2,2] = 40
    
    R = np.eye(u_bar.shape[1])*1
    R[1,1] = 10
    
    Lr = param["L_r"]
    Lf = param["L_f"]

    A = np.zeros((4,4))
    B = np.zeros((4,2))
    delta_t = param["h"]
    # print(u_bar.shape)
    
    for i in range(x.shape[0]-1):
        
        costlist += (cp.quad_form((x[i,:]-x_bar[i,:]),Q) + cp.quad_form((u[i,:]-u_bar[i,:]), R))
        

        A[0,0] = 1.0
        A[1,1] = 1.0
        A[2,2] = 1.0
        A[3,3] = 1.0
        beta = np.arctan((Lr*np.arctan(u_bar[i,1]))/(Lr+Lf))
        inside_beta = ((Lr*np.arctan(u_bar[i,1]))/(Lr+Lf))

        A[0,2] = -x_bar[i,3]*np.sin(x_bar[i,2]+beta)*delta_t
        A[0,3] = x_bar[i,3]*np.cos(x_bar[i,2]+beta)*delta_t
        
        A[1,2] = np.cos(x_bar[i,2]+beta)*delta_t
        A[1,3] = np.sin(x_bar[i,2]+beta)*delta_t
        den1 = ((Lr*np.arctan(u_bar[i,1]))**2 + (Lr+Lf)**2)**0.5
        A[2,3] = ((np.arctan(u_bar[i,1]))/den1)*delta_t

        B[3,0] = 1*delta_t
        B[0,1] = A[0,2] * (Lr/(Lr+Lf)) * (1/(1+(u_bar[i,1]**2))) * (1/(1+((Lr+Lf)**2))) * delta_t
        B[1,1] = A[1,2] * (Lr/(Lr+Lf)) * (1/(1+(u_bar[i,1]**2))) * (1/(1+((Lr+Lf)**2))) * delta_t
        B[2,1] =  x_bar[i,3] * (1/(1+(u_bar[i,1]**2))) * (1/den1**3) * delta_t
        
        
        constrlist += [(x[i + 1, :]-x_bar[i + 1, :]) == ((A @ (x[i, :]-x_bar[i, :]) + (B @ (u[i,:] - u_bar[i,:]))))]
        constrlist += [u[i, 0]-u_bar[i,0] >= param["a_lim"][0]-u_bar[i,0]]
        constrlist += [u[i, 0]-u_bar[i,0] <= param["a_lim"][1]-u_bar[i,0]]
        
        constrlist += [u[i, 1]-u_bar[i,1] >= param["delta_lim"][0]-u_bar[i,1]]
        constrlist += [u[i, 1]-u_bar[i,1] <= param["delta_lim"][1]-u_bar[i,1]]


    costlist += (cp.quad_form((x[-1,:]-x_bar[-1,:]),P))
    constrlist += [(x[0, :] - x_bar[0, :]) == (x0 - x_bar[0, :])]

    prob = cp.Problem(cp.Minimize(costlist), constrlist)
    
    prob.solve(verbose=False)
    # print(u.value[0,:])
    return u.value[0,:]