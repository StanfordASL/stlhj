import h5py
import numpy as np
from setup import *
from casadi import *
import rospy
from std_msgs.msg import Float64MultiArray
import scipy.interpolate

class MPC:

    def __init__(self):
        rospy.init_node('MPC_controller', anonymous=True)

        self.v2 = 0.04

        hf = h5py.File('overtake_output_Ext.mat','r')
        self.V = hf['overtake_output_Ext'] #array dimensions are reversed
        self.p = load_params(hf)
        self.u = Float64MultiArray()
        self.u.data = [float(0),float(0)]
        self.integration_iters = 2
        self.solver, self.bc = self.casadi_setup()

        xg = np.linspace(self.gmin[0],self.gmax[0],self.N[0])
        yg = np.linspace(self.gmin[1],self.gmax[1],self.N[1])
        self.thg = np.linspace(self.gmin[2],self.gmax[2],self.N[2])
        self.velg = np.linspace(self.gmin[3],self.gmax[3],self.N[3])
        y2g = np.linspace(self.gmin[4],self.gmax[4],self.N[4])
        tg = np.linspace(0,27,np.shape(self.V)[0])
        self.interp_function = scipy.interpolate.RegularGridInterpolator((tg,y2g,self.velg,self.thg,yg,xg),np.array(self.V))

        self.ctrl_publisher = rospy.Publisher('/ctrl_MATLAB',Float64MultiArray,queue_size=1)
        rospy.Subscriber('/StateSpace',Float64MultiArray,self.mpc_callback)

    def mpc_callback(self,msg):
        #sim_start = rospy.get_time()

        self.states = [msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5]]
        self.OptStates = self.loadOptimalStates()

        lbx = self.bc['lbx']
        ubx = self.bc['ubx']
        lbg = self.bc['lbg']
        ubg = self.bc['ubg']

        lbx[:4] = self.states[:4]
        ubx[:4] = self.states[:4]
        lbx[4:6] = [self.u.data[1],self.u.data[0]]
        ubx[4:6] = [self.u.data[1],self.u.data[0]]

        sol = self.solver(lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=self.OptStates)

        w_opt = sol['x'].full().flatten()
        u_omega = w_opt[10]
        u_accel = w_opt[11]

        self.u.data = [float(u_accel),float(u_omega)]


        #print(rospy.get_time()-sim_start)
        self.ctrl_publisher.publish(self.u)

        # import matplotlib.pyplot as plt
        # w_opt_th = w_opt[2::6]
        # w_opt_vel = w_opt[3::6]
        # print('w_opt_th:', w_opt_th)
        # print('MPC_th:', self.OptStates[0:11])
        # print('w_opt_vel:', w_opt_vel)
        # print('MPC_vel:', self.OptStates[11:23])
        #
        # plt.figure(1)
        # plt.clf()
        # plt.plot(w_opt_th, '--')
        # plt.plot(self.OptStates[0:11], '-')
        #
        # plt.figure(2)
        # plt.clf()
        # plt.plot(w_opt_vel, '--')
        # plt.plot(self.OptStates[11:23], '-')
        # plt.show()


    def loadOptimalStates(self):
        th_bar = []
        vel_bar = []
        temp_states = list(self.states)
        t = np.linspace(temp_states[5],temp_states[5]+self.T,self.iters+1)
        for i in range(np.size(t)):
            single_OptStates = self.loadSingleOptimalStates(temp_states)
            omega = (single_OptStates[0] - temp_states[2])/self.dt
            if omega < -0.4:
                omega = -0.4
            elif omega > 0.4:
                omega = 0.4

            accel = (single_OptStates[1] - temp_states[3])/self.dt
            if accel < -0.2:
                accel = -0.2
            elif accel > 0.2:
                accel = 0.2

            th_bar.append(single_OptStates[0])
            vel_bar.append(single_OptStates[1])

            dt2 = self.dt/self.integration_iters
            for j in range(self.integration_iters):
                temp_states[0] = temp_states[0] + temp_states[3]*np.cos(temp_states[2])*dt2
                temp_states[1] = temp_states[1] + (temp_states[3]*np.sin(temp_states[2])-0.01)*dt2
                temp_states[2] = temp_states[2] + omega*dt2
                temp_states[3] = temp_states[3] + accel*dt2
                temp_states[4] = temp_states[4] + self.v2*dt2
                temp_states[5] = temp_states[5] + dt2
        th_bar += vel_bar
        return th_bar

    def loadSingleOptimalStates(self,states):
        points_for_th = np.meshgrid(states[5],states[4],states[3],self.thg,states[1],states[0])
        flat_for_th = np.array([m.flatten() for m in points_for_th])
        out_array_th = self.interp_function(flat_for_th.T)

        I_th = np.argmax(out_array_th)

        points_for_vel = np.meshgrid(states[5],states[4],self.velg,states[2],states[1],states[0])
        flat_for_vel = np.array([m.flatten() for m in points_for_vel])

        out_array_vel = self.interp_function(flat_for_vel.T)
        I_vel = np.argmax(out_array_vel)

        return [self.thg[I_th],self.velg[I_vel]]


    def casadi_setup(self):

        # Unpack relevant properties
        self.gmin = self.p['min'].tolist()
        self.gmax = self.p['max'].tolist()
        self.N = self.p['N'] #number of discrete grids for each state
        self.dt = self.p['dt']
        self.T = self.p['T']
        self.iters = self.p['iters']

        #State variables
        x = MX.sym('x')
        y = MX.sym('y')
        vel = MX.sym('vel')
        th = MX.sym('th')
        #y2 = MX.sym('y2')
        state = vertcat(x,y,th,vel)

        #Control variables
        omega = MX.sym('omega')
        accel = MX.sym('accel')
        #v2 = MX.sym('v2')
        control = vertcat(omega,accel)

        #Tracking variables
        num_th_vel_bar = int(2*(self.iters+1))
        th_vel_bar = MX.sym('th_vel_bar',num_th_vel_bar) #[th_bar, vel_bar]

        # System dynamics equations
        statedot = vertcat(vel*cos(th),
                           vel*sin(th)-0.01,
                           omega,
                           accel)

        f = Function('f',[state,control],[statedot])


        X0 = MX.sym('X0',4)
        U = MX.sym('U',2)
        X = X0
        k1 = f(X,U)
        k2 = f(X + self.dt/2*k1, U)
        X = X + self.dt/2*(k1 + k2)
        F = Function('F',[X0,U],[X],['x0','p'],['xf'])
        # test = F(x0=[0,0,0,0],p=[1,1])
        # print(test)

        # Start with an empty NLP
        w = []    # the decision variables lbw < w < ubw
        lbw = []  # lower bound constraint on dec var
        ubw = []  # upper bound constraint on dec var
        g = []    # vector for constraints lbg < g(w,p) < ubg
        lbg = []  # lower bound on constraints
        ubg = []  # upper bound on constraints
        th_bar = []
        vel_bar = []
        J = 0

        X_mat = []; U_mat = []

        Xn = MX.sym('Xi',4)
        X_mat += [Xn]
        w += [Xn]
        lbw += self.gmin[0:4]
        ubw += self.gmax[0:4]

        for k in range(self.iters):

            Un = MX.sym('U_' + str(k), 2)
            U_mat += [Un]
            w += [Un]
            lbw += [-0.4, -0.2]
            ubw += [0.4, 0.2]


            fn = F(x0=Xn, p=Un)
            Xn_end = fn['xf']

            Xn = MX.sym('X_' + str(k), 4)
            X_mat += [Xn]
            w += [Xn]
            lbw += self.gmin[0:4]
            ubw += self.gmax[0:4]

            if k != 0:
                J = J + 10*(Un_end[0]-Un[0])**2 + 10*(Un_end[1]-Un[1])**2 + \
                    10*(th_vel_bar[k+1]-Xn[2])**2 + 10*(th_vel_bar[self.iters+1+k+1]-Xn[3])**2

            Un_end = Un
            g += [Xn_end-Xn]
            lbg += [0, 0, 0, 0]
            ubg += [0, 0, 0, 0]

        J = J/self.iters
        prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g), 'p': vertcat(th_vel_bar)}
        solver = nlpsol('solver','ipopt', prob,{'ipopt.print_level':0,'print_time':False})
        #solver = nlpsol('solver','ipopt', prob)
        return solver, {'lbx':lbw,'ubx':ubw,'lbg':lbg,'ubg':ubg}

if __name__ == '__main__':
    MPC()
    rospy.spin()
