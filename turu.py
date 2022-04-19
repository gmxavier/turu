# turu.py - A Python collection of PID tuning rules
# GMX, 17 April 2022
#
# This file contains a collection of PID tuning rules.

from numpy import interp

def callender(K, tau, theta, 
              type_of_plant='FODT',
              type_of_control='regulatory', 
              type_of_controller='PI', 
              criteria=1):
    r'''Returns the PI controller parameters from the rule of Callender et al. (1935/6).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)
    criteria : integer
         If criteria = 1, the rule gives decay ratio = 0.015 and period of decaying 
         oscillation = 5.10*theta.
         If criteria = 2, the rule gives decay ratio = 0.043 and period of decaying 
         oscillation = 6.28*theta.

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]

    Notes
    -----
    Applicable to theta/tau = 0.3.

    Example
    --------

    >>> callender(K=1, tau=10, theta=3)
    [0.18933333333333333, 0.01733821733821734]

    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''
    if criteria not in [1,2]:
        return []
    if criteria == 1:
        Kp = 0.568/(K*theta)
        Ki = Kp/(3.64*theta)
        return [Kp, Ki]
    if criteria == 2:
        if type_of_controller == 'PI':
            Kp = 0.690/(K*theta)
            Ki = Kp/(2.45*theta)
            return [Kp, Ki]
        if type_of_controller == 'PID':
            Kp = 1.066/(K*theta)
            Ki = Kp/(1.418*theta)
            Kd = Kp*(0.47*theta)
            return [Kp, Ki, Kd]

def ziegler_nichols(K, tau, theta, 
                   type_of_plant='FODT',
                   type_of_control='regulatory', 
                   type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rule of Ziegler and Nichols (1942).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----
    Applicable to theta/tau <= 1.0.

    Example
    --------

    >>> Kp, Ki, Kd = ziegler_nichols(K=1.25, tau=4, theta=0.9, type_of_controller='PID'); [Kp, Kp/Ki, Kd/Kp]
    [4.266666666666667, 1.8, 0.45]
    
    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''
    if type_of_controller == 'P':
        Kp = (1/K)*(tau/theta)
        return [Kp]
    if type_of_controller == 'PI':
        Kp = 0.9*(1/K)*(tau/theta)
        Ki = Kp/(3.3*theta)
        return [Kp, Ki]
    if type_of_controller == 'PID':
        Kp = 1.2*(1/K)*(tau/theta)
        Ki = Kp/(2*theta)
        Kd = Kp*0.5*theta
        return [Kp, Ki, Kd]    
 
def hazebroek_vanderwaerden(K, tau, theta, 
                            type_of_plant='FODT',
                            type_of_control='regulatory', 
                            type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rule of Hazebroek and Van der Waerden (1950).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> hazebroek_vanderwaerden(K=1, tau=10, theta=3)
    [2.3333333333333335, 0.16339869281045755]

    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    if thetaovertau < 0.2:
        return [] 
    if (thetaovertau >= 0.2) & (thetaovertau <= 3.4):    
        thetaovertau_ = [0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4,
                         1.5,1.6,1.7,1.8,1.9,2.0,2.2,2.4,2.6,2.8,3.0,3.2,3.4]
        x1_ = [0.68,0.70,0.72,0.74,0.76,0.79,0.81,0.84,0.87,0.90,0.93,0.96,0.99,
               1.02,1.06,1.09,1.13,1.17,1.20,1.28,1.36,1.45,1.53,1.62,1.71,1.81]
        x2_ = [7.14,4.76,3.70,3.03,2.50,2.17,1.92,1.75,1.61,1.49,1.41,1.32,1.25,
               1.19,1.14,1.10,1.06,1.03,1.00,0.95,0.91,0.88,0.85,0.83,0.81,0.80]
        x1 = interp(thetaovertau, thetaovertau_, x1_)
        x2 = interp(thetaovertau, thetaovertau_, x2_)
        Kp = x1/(K*thetaovertau)
        Ki = Kp/(x2*theta)
        return [Kp, Ki]        
    if thetaovertau > 3.4:
        Kp = 1/(K*thetaovertau)*(0.5*thetaovertau + 1)
        Ki = Kp/(theta/(1.6*theta - 1.2*tau))
        return [Kp, Ki]  
 
def oppelt(K, tau, theta, 
           type_of_plant='FODT',
           type_of_control='regulatory', 
           type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rule of Oppelt (1951).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----
    This function calculates the recommended value for Kp.

    Example
    --------

    >>> oppelt(K=1, tau=10, theta=3)
    [1.5666666666666669, 0.15729585006693445]

    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    if thetaovertau < 1:
        Kp = (1/K)*(0.77/thetaovertau - 1)
        Ki = Kp/(3.32*theta)
        return [Kp, Ki]  
    if thetaovertau > 1:
        Kp = (1/K)*(0.77/thetaovertau - 1)
        Ki = Kp/(1.66*theta)
        return [Kp, Ki]  

def moros_oppelt(K, tau, theta, 
                 type_of_plant='FODT',
                 type_of_control='regulatory', 
                 type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of Moros (1999).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> moros_oppelt(K=1, tau=10, theta=3)
    [2.666666666666667, 0.29629629629629634]

    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    if type_of_controller == 'PI':    
      Kp = 0.8/(K*thetaovertau)
      Ki = Kp/(3*theta)
      return [Kp, Ki] 
    if type_of_controller == 'PID':
      Kp = 1.2/(K*thetaovertau)
      Ki = Kp/(2*theta)
      Kd = Kp*(0.42*theta)
      return [Kp, Ki, Kd]       

def moros_rosenberg(K, tau, theta, 
                    type_of_plant='FODT',
                    type_of_control='regulatory', 
                    type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of Moros (1999).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> moros_rosenberg(K=1, tau=10, theta=3)
    [3.0333333333333337, 0.3063973063973065]

    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    if type_of_controller == 'PI':
      Kp = 0.91/(K*thetaovertau)
      Ki = Kp/(3.3*theta)
      return [Kp, Ki] 
    if type_of_controller == 'PID':
      Kp = 1.2/(K*thetaovertau)
      Ki = Kp/(2.0*theta)
      Kd = Kp*(0.44*theta)
      return [Kp, Ki, Kd] 

def cohen_coon(K, tau, theta, 
               type_of_plant='FODT',
               type_of_control='regulatory', 
               type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rule of Cohen and Coon (1953).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> Kp, Ki, Kd = cohen_coon(K=1.25, tau=4, theta=0.9, type_of_controller='PID'); [Kp, Kp/Ki, Kd/Kp]
    [4.940740740740741, 2.253378378378379, 0.38816108685104317]
    
    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''
    if type_of_controller == 'P':
        Kp = (1/K)*(tau/theta + 1/3)
        return [Kp]
    if type_of_controller == 'PI':
        Kp = (1/K)*(0.9*tau/theta + 1/12)
        Ki = Kp/((theta*(30 + 3*(theta/tau))/(theta*(9 + 20*(theta/tau)))))
        return [Kp, Ki]
    if type_of_controller == 'PID':
        Kp = (1/K)*((4/3)*(tau/theta) + 1/4)
        Ki = Kp/((theta*(32 + 6*(theta/tau))/(theta*(13 + 8*(theta/tau)))))
        Kd = Kp*(4/(theta*(11 + 2*(theta/tau))))
        return [Kp, Ki, Kd]    

def fertik_sharpe(K, tau, theta, 
                  type_of_plant='FODT',
                  type_of_control='regulatory', 
                  type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of Fertik and Sharpe (1979).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> fertik_sharpe(K=1, tau=10, theta=3)
    [0.56, 0.08615384615384616]

    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    Kp = 0.56/K
    Ki = Kp/(0.65*tau)
    return [Kp, Ki]

def parr(K, tau, theta, 
         type_of_plant='FODT',
         type_of_control='regulatory', 
         type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of Parr (1989).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> parr(K=1, tau=10, theta=3)
    [3.0333333333333337, 0.3063973063973065]

    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    if type_of_controller == 'PI':
      Kp = 0.91/(K*thetaovertau)
      Ki = Kp/(3.3*theta)
      return [Kp, Ki] 
    if type_of_controller == 'PID':
      Kp = 1.25/(K*thetaovertau)
      Ki = Kp/(2.5*theta)
      Kd = Kp*(0.4*theta)
      return [Kp, Ki, Kd] 

def sakai(K, tau, theta, 
          type_of_plant='FODT',
          type_of_control='regulatory', 
          type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of Sakai et al. (1989).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> sakai(K=1, tau=10, theta=3)
    [4.136, 2.7573333333333334]

    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    Kp = 1.2408/(K*thetaovertau)
    Ki = Kp/(0.5*theta)
    return [Kp, Ki] 

def borresen_grindal(K, tau, theta, 
                      type_of_plant='FODT',
                      type_of_control='regulatory', 
                      type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of Borresen and Grindal (1990).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> borresen_grindal(K=1, tau=10, theta=3)
    [3.3333333333333335, 0.3703703703703704]

    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    if type_of_controller == 'PI':
      Kp = 1.0/(K*thetaovertau)
      Ki = Kp/(3.0*theta)
      return [Kp, Ki] 
    if type_of_controller == 'PID':
      Kp = 1.0/(K*thetaovertau)
      Ki = Kp/(3.0*theta)
      Kd = Kp*(0.5*theta)
      return [Kp, Ki, Kd]     

def klein(K, tau, theta, 
          type_of_plant='FODT',
          type_of_control='regulatory', 
          type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of Klein et al. (1992).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> klein(K=1, tau=10, theta=3)
    [0.7000000000000001, 0.1320754716981132]

    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    Kp = 0.28/(K*(thetaovertau + 0.1))
    Ki = Kp/(0.53*tau)
    return [Kp, Ki] 
  
def mcmillan(K, tau, theta, 
             type_of_plant='FODT',
             type_of_control='regulatory', 
             type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of McMillan (1994).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> mcmillan(K=1, tau=10, theta=3)
    [0.3333333333333333, 1.0]

    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    Kp = K/3
    Ki = Kp/theta
    return [Kp, Ki] 
  
def stclair(K, tau, theta, 
            type_of_plant='FODT',
            type_of_control='regulatory', 
            type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of St. Clair (1997).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----
    It's valid for theta/tau >= 0.33.

    Example
    --------

    >>> stclair(K=1, tau=10, theta=3)


    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    Kp = 0.333/(K*thetaovertau)
    Ki = tau
    return [Kp, Ki] 
  
def shinskey(K, tau, theta, 
             type_of_plant='FODT',
             type_of_control='regulatory', 
             type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of Shinskey (2000).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> shinskey(K=1, tau=10, theta=3)


    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    Kp = 0.667/(K*thetaovertau)
    Ki = Kp/(3.78*theta)
    return [Kp, Ki] 
  
def liptak(K, tau, theta, 
           type_of_plant='FODT',
           type_of_control='regulatory', 
           type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of Liptak (2001).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> liptak(K=1, tau=10, theta=3)


    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    if type_of_controller == 'PI':
      Kp = 0.95/(K*thetaovertau)
      Ki = Kp/(4.0*theta)
      return [Kp, Ki] 
    if type_of_controller == 'PID':
      Kp = 0.85/(K*thetaovertau)
      Ki = Kp/(1.6*theta)
      Kd = Kp*(0.6*theta)
      return [Kp, Ki]       
  
def chidambaram(K, tau, theta, 
               type_of_plant='FODT',
               type_of_control='regulatory', 
               type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of Chidambaram (2002).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> chidambaram(K=1, tau=10, theta=3)


    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    if type_of_controller == 'PI':
      Kp = (1/K)*(0.4 + 0.665/thetaovertau)
      Ki = Kp/(3.4*theta)
      return [Kp, Ki] 
    if type_of_controller == 'PID':
      Kp = (1/K)*(0.45 + 1.8/thetaovertau)
      Ki = Kp/(2.4*theta)
      Kd = Kp*(0.38*theta)
      return [Kp, Ki] 

def faanes_skogestad(K, tau, theta, 
                     type_of_plant='FODT',
                     type_of_control='regulatory', 
                     type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of Faanes and Skogestad (2004).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> faanes_skogestad(K=1, tau=10, theta=3)


    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    Kp = 0.71/(K*thetaovertau)
    Ki = Kp/(3.3*theta)
    return [Kp, Ki] 
  
def pma(K, tau, theta, 
       type_of_plant='FODT',
       type_of_control='regulatory', 
       type_of_controller='PI'):
    r'''Returns the PI controller parameters from the rules of PMA (2006).

    Parameters
    ----------
    K : float
         Static gain of the process reaction curve, [-] 
    tau : float
         Time constant (lag) of the process reaction curve, [time]
    theta : float
         Dead time of the process reaction curve, [time]
    type_of_plant : string
         Type of the plant model   
    type_of_control : string
         Type of the control loop (regulatory or servo)
    type_of_controller : string
         Type of the controller (P, PI, PD, PID)

    Returns
    -------
    Kp : float
         Proportional gain, [-]

    Ki : float
         Integral gain, [1/time]
         
    Kd : float
         Derivative gain, [time]         

    Notes
    -----


    Example
    --------

    >>> pma(K=1, tau=10, theta=3)


    Reference
    ----------
    .. [1] O’Dwyer, A. Handbook of PI and PID Controller Tuning Rules. London:
       Imperial College Press, 2009.
    '''    
    thetaovertau = theta/tau
    Kp = 0.39/(K*thetaovertau)
    Ki = Kp/(6.0*theta)
    return [Kp, Ki] 
