def Student_Controller(t, x, param):
    import numpy as np
    vd = param["vd"]
    v0 = param["v0"]

    m = param["m"]
    Cag = param["Cag"]
    Cdg = param["Cdg"]


    D = x[0]
    v = x[1]
    
    ## TODO
    lam = 1000.0
    alpha = 0.2
    w = 100000.0
    # print(lam, alpha, w)
    
    h = ((v-vd)**2)/2
    B = D - ((v-v0)**2)/(2*Cdg) - (1.8*v)
    # other ...

    ## TODO
    
    
    # breakpoint()
    P = np.zeros((2,2))
    
    # P = [[1, 0],
    #      [0, w]]
    P[0,0] = 2
    P[0,1] = 0
    P[1,0] = 0
    P[1,1] = w*2
    
    ## TODO
    A = np.zeros([5, 2])
    A[0,0] = (v-vd)/m
    A[0,1] = -1
    A[1,0] = (1.8 +((v-v0)/Cdg))/m
    A[1,1] = 0
    A[2,0] = -1
    A[2,1] = 0
    A[3,0] = 1
    A[3,1] = 0
    A[4,0] = 0
    A[4,1] = -1
    # A = [[(v-vd), 0],
    #      [1.8 +((v-v0)/Cdg), 0], 
    #     [-1, 0],
    #     [1,0],
    #      [0,-1]
    #     ]

    ## TODO
    b = np.zeros([5])
    b[0] = -lam*h
    b[1] = ((alpha*B) + (v0-v))
    b[2]  = m*Cdg
    b[3] = m*Cag
    b[4] = 0

    ## TODO
    q = np.zeros([2, 1])
    
    return A, b, P, q