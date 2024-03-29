#
# LEPL1504 -  MISSION 1  -  Modeling a cart-pendulum system
#
# @date 2022
# @author Robotran team
# 
# Universite catholique de Louvain


# import useful modules and functions
from math import sin, cos, pi
import numpy as np
from scipy.integrate import solve_ivp


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
class MBSData:
    """ Class to define the parameters of the double mass-spring model
    and of the problem to solve.
     
    It contains the following fields (in SI units): 
    
    general parameters:
    -------------------
    g:     Gravity
    
    masses:
    -------
    m1:    Cart mass
    m2:    Pendulum mass
    
    parameters of the pendulum:
    ---------------------------
    Lp:    Pendulum length
    
    parameter of the external force:
    --------------------------------
    Fmax:  Semi-amplitude of the force
    f0:    Frequency at start
    t0:    Time for specifying frequency at start
    f1:    Frequency at end
    t1:    Time for specifying frequency at end
    
    parameter of the controller:
    ----------------------------
    Kp:    Proportional factor
    Kd:    Differential factor
    
    initial positions and velocities:
    ---------------------------------
    q1:    Initial position coordinate of the cart
    q2:    Initial position coordinate of the pendulum
    qd1:   Initial velocity coordinate of the cart
    qd2:   Initial velocity coordinate of the pendulum
    """
    
    def __init__(self):  #Initial parameters of the pendulum
        "general parameters"
        self.g = 9.81  # [m/s^2]
        
        "masses"
        self.m1 = 1  # [Kg]
        self.m2 = 0.5  # [Kg]
        
        "parameters of the pendulum"
        self.Lp = 1  # [m]
        
        "parameter of the external force"
        self.Fmax = 0.5  # [N]
        self.f0 = 1  # [Hz]
        self.t0 = 0  # [s]
        self.f1 = 10  # [Hz]
        self.t1 = 10  # [s]
        
        
        "parameter of the controller"
        self.Kp = 0  
        self.Kd = 0  
        
        "initial positions and velocities"
        self.q1 = 0  # [m]
        self.q2 = pi/6  # [deg]
        self.qd1 = 0  # [m/s]
        self.qd2 = 0  # [m/s]
    
        
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
def sweep(t, t0, f0, t1, f1, Fmax):
    """ Compute the value of a force sweep function at the given time.
    The sweep function has a sinusoidal shape with constant amplitude 
    and a varying frequency. This function enables to consider linear
    variation of the frequency between f0 and f1 defined at time t0 and t1.

    :param t: the time instant when to compute the function.
    :param t0: the time instant when to specify f0.
    :param f0: the frequency at time t0.
    :param t1: the time instant when to specify f1.
    :param f1: the frequency at time t1.
    :param Fmax: the semi-amplitude of the function.
        
    :return Fext: the value of the sweep function.
    """
    return Fmax*sin(2*pi*t*(f0 + (f1-f0)/(t1-t0)*(t/2)))


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *    
def compute_derivatives(t, y, data):
    """ Compute the derivatives yd for a given state y of the system.
        The derivatives are computed at the given time t with
        the parameters values in the given data structure.
        
        It is assumed that the state vector y contains the following states:
          y = [q1, q2, qd1, qd2] with:
             - q1: the cart position
             - q2: the pendulum position 
             - qd1: the cart velocity
             - qd2: the pendulum velocity 

        :param  t: the time instant when to compute the derivatives.
        :param  y: the numpy array containing the states 
        :return: yd a numpy array containing the states derivatives  yd = [qd1, qd2, qdd1, qdd2]
        :param data: the MBSData object containing the parameters of the model
    """
    
    """
    On résout avec le système M*qdd = Q - c avec les définitions données au cours
    """
    # yd = np.array([y[2],y[3],0,0])  #on déterminera les coéfficient de ¨q juste après pour avoir le yd final
                          # -> yd = d[y]/dt -> yd = [qd1, qd2, qdd1, qdd2]
    
    # Matrice M #
    M = np.array([[data.m1 + data.m2 ,  data.m2 * data.Lp/2 * cos(y[1])],
                  [ cos(y[1])       , 2/3 * data.Lp]])

    # Vecteur Q #
    Q1 = sweep(t, data.t0, data.f0, data.t1, data.f1, data.Fmax)
    Q2 = 0
    Q = np.array([Q1,Q2])
    
    # Matrice c #
    c1 = - data.m2 * data.Lp/2 * y[3]**2 * sin(y[1])
    c2 = - data.g * sin(y[1])
    c = np.array([c1,c2])
    
    # solve système
    ydd = np.linalg.solve(M,Q-c)
    
    return np.array([y[2],y[3],ydd[0],ydd[1]])
    # sweep function should be called here: sweep(t, data.t0, data.f0, data.t1, data.f1, data.Fmax)


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
def compute_dynamic_response(data):
    """  Compute the time evolution of the dynamic response of the cart-pendulum system
         for the given data. Initial and final time are determined
         by the t0 and t1 parameter of the parameter data structure.
         Results are saved to three text files named dirdyn_q.res, dirdyn_qd.res and dirdyn_qdd.res
 
        Time evolution is computed using an time integrator (typically Runge-Kutta).
 
       :param data: the MBSData object containing the parameters of the model
     """
    # Write your code here
    # ### Runge Kutta ###   should be called via solve_ivp()
    # to pass the MBSData object to compute_derivative function in solve_ivp, you may use lambda mechanism:
    #
    #    fprime = lambda t,y: compute_derivatives(t, y, data)
    #
    # fprime can be viewed as a function that takes two arguments: t, y
    # this fprime function can be provided to solve_ivp
    # Note that you can change the tolerances with rtol and atol options (see online solve_iv doc)
    #
    # Write some code here
    fprime = lambda t, y: compute_derivatives(t, y, data)
    init = np.array([data.q1, data.q2, data.qd1, data.qd2])
    
    teval = np.linspace(data.t0, data.t1, 1000, endpoint=True)
    
    sol = solve_ivp(fprime, (data.t0, data.t1), init,'Radau', t_eval=teval,rtol=1e-9)

    q = np.array([sol.t, sol.y[0], sol.y[1]])
    qd = np.array([sol.t, sol.y[2], sol.y[3]])
    qdd = np.zeros((3,len(sol.t)))
    qdd[0] = sol.t
    for i in range(len(sol.t)):
        qddtemp = compute_derivatives(teval[i], sol.y[:,i] , data)
        qdd[1][i] = qddtemp[2]
        qdd[2][i] = qddtemp[3]
    np.savetxt('dirdyn_q.res',q.transpose())
    np.savetxt('dirdyn_qd.res',qd.transpose())
    np.savetxt('dirdyn_qdd.res',qdd.transpose())
    
    return sol
    


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
# Main function

if __name__ == '__main__':
    mbs_data = MBSData()
    
    compute_dynamic_response(mbs_data)
    
    
    
    
    