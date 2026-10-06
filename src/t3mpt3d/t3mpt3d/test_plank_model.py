import numpy as np
from matplotlib import pyplot as plt
def diff_drive(X,U,dt):

    x, y, th = X
    v, vth = U

    xn = x + v * np.cos(th) * dt
    yn = y + v * np.sin(th) * dt
    thn = th + vth * dt

    return xn,yn,thn


def plank_model1(X,U,r,dt):

    x, y, th, thp = X
    v1, v2, vth = U

    #r*np.cos((2*dv/(r*2)+th))*0

    dv = v2-v1
    vx = (v2+v1)/2 * np.cos(th)
    vy = (v2+v1)/2 * np.sin(th)
    vthp = dv/r

    xn = x + vx*dt
    yn = y + vy*dt
    thp = thp + vthp*dt
    th = th + vth * dt

    X_n = (xn, yn, th, thp)

    return X_n

def plank_model_ormen(X,U,r,dt):

    x, y, th, th1, th2 = X
    v1, v2, vth1, vth2 = U

    #th1 = th1 + th #conversion from global to local frame
    #th2 = th2 + th
    vx = (v1 * np.cos(th1) + v2 * np.cos(th2))/2
    vy = (v1 * np.sin(th1) + v2 * np.sin(th2))/2
    vth = ((v2*np.cos(th2) - v1*np.cos(th1)) * -np.sin(th) + (v2*np.sin(th2)-v1*np.sin(th1)) * np.cos(th))/(2*r)


    xn = x + vx*dt
    yn = y + vy*dt
    thn = th + vth * dt
    th1n = th1 + vth1 * dt
    th2n = th2 + vth2 * dt

    X_n = (xn, yn, thn, th1n, th2n)

    return X_n

def testa_ormen():
    x1, y1, th1 = [-1.0,0.0,np.pi/2]
    x1s = []
    y1s = []
    th1s = []

    x2, y2, th2 = [1.0,0.0,np.pi/2]
    x2s = []
    y2s = []
    th2s = []

    U1 = [np.pi/360, 2*np.pi/360]
    U2 = [5*np.pi/360, 2*np.pi/360]
    #U1 = [1.0,0.0]
    #U2 = [1.0,0.0]

    xp, yp, thp = [0.0, 0.0, 0.0]
    xps = []
    yps = []
    thps = []
    r = 1.0
    dt = 1.0
    plot_every = 10
    for i in range(int(270/dt)):
        if i < int(90/dt):
            U1 = [0*np.pi/360, 0*np.pi/360]
            U2 = [np.pi/360, 0*np.pi/360]
        else:
            U1 = [5*np.pi/360, 0*np.pi/360]
            U2 = [np.pi/360, -2*np.pi/360]


        X1 = x1,y1,th1
        x1,y1,th1 = diff_drive(X1,U1,dt)


        X2 = x2,y2,th2
        x2,y2,th2 = diff_drive(X2,U2,dt)



        UP = [U1[0],U2[0],U1[1],U2[1]]
        XP = xp,yp,thp,th1,th2
        xp, yp, thp,_,_ = plank_model_ormen(XP,UP,r,dt)

        if i%plot_every == 0:
            x1s.append(x1)
            y1s.append(y1)
            th1s.append(th1)

            x2s.append(x2)
            y2s.append(y2)
            th2s.append(th2)

            xps.append(xp)
            yps.append(yp)
            thps.append(thp)

    print(th1,th2)
    print(f"ROBOT 1: X: {x1} Y: {y1} Yaw: {th1}")
    print(f"ROBOT 2: X: {x2} Y: {y2} Yaw: {th2}")
    print(f"PLANK: X: {xp} Y: {yp} Yaw: {thp}")
    plt.figure()
    u1 = np.cos(th1s)
    v1 = np.sin(th1s)
    plt.quiver(x1s,y1s,u1,v1,color="red")

    u2 = np.cos(th2s)
    v2 = np.sin(th2s)
    plt.quiver(x2s,y2s,u2,v2,color="blue")

    thps = np.array(thps)
    up = np.cos(thps)
    vp = np.sin(thps)
    plt.quiver(xps,yps,up,vp)

    plt.xlim([-3,3])
    plt.ylim([-3,3])
    plt.axis('equal')
    plt.show()


def testa_fasta_hjul():
    x1, y1, th1 = [-1.0,0.0,np.pi/2]
    x1s = []
    y1s = []
    th1s = []

    x2, y2, th2 = [1.0,0.0,np.pi/2]
    x2s = []
    y2s = []
    th2s = []

    U1 = [np.pi/360, 2*np.pi/360]
    U2 = [5*np.pi/360, 2*np.pi/360]
    #U1 = [1.0,0.0]
    #U2 = [1.0,0.0]

    xp, yp, thp = [0.0, 0.0, 0.0]
    xps = []
    yps = []
    thps = []
    r = 2.0
    dt = 0.1
    plot_every = 10
    for i in range(int(270/dt)):
        if i < int(90/dt):
            U1 = [np.pi/360, 2*np.pi/360]
            U2 = [5*np.pi/360, 2*np.pi/360]
        else:
            U1 = [5*np.pi/360, -2*np.pi/360]
            U2 = [np.pi/360, -2*np.pi/360]


        X1 = x1,y1,th1
        x1,y1,th1 = diff_drive(X1,U1,dt)


        X2 = x2,y2,th2
        x2,y2,th2 = diff_drive(X2,U2,dt)



        UP = [U1[0],U2[0],U1[1]]
        XP = xp,yp,th1,thp
        xp, yp, _, thp = plank_model(XP,UP,r,dt)

        if i%plot_every == 0:
            x1s.append(x1)
            y1s.append(y1)
            th1s.append(th1)

            x2s.append(x2)
            y2s.append(y2)
            th2s.append(th2)

            xps.append(xp)
            yps.append(yp)
            thps.append(thp)

    print(th1,th2)
    print(f"ROBOT 1: X: {x1} Y: {y1} Yaw: {th1}")
    print(f"ROBOT 2: X: {x2} Y: {y2} Yaw: {th2}")
    print(f"PLANK: X: {xp} Y: {yp} Yaw: {thp}")
    plt.figure()
    u1 = np.cos(th1s)
    v1 = np.sin(th1s)
    #plt.quiver(x1s,y1s,u1,v1,color="red")

    u2 = np.cos(th2s)
    v2 = np.sin(th2s)
    #plt.quiver(x2s,y2s,u2,v2,color="blue")

    thps = np.array(thps)
    up = np.cos(thps)
    vp = np.sin(thps)
    plt.quiver(xps,yps,up,vp)

    plt.xlim([-3,3])
    plt.ylim([-3,3])
    plt.axis('equal')
    plt.show()

if __name__ == '__main__':
    testa_ormen()





