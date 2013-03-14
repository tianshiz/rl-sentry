""" Tracking predictor

Using Kalman Filter, predicts the position of a target in the future

TODO Evaluate A,Q,H,R

TODO A should be learn rather then set by hand
TODO Does state need momentum (probably yes NdAF)?
"""
from numpy import *


def KalmanStep(x,p,  z_past = [], step_future = 0):
    """ Perform Kalman filtering

    x,p are the intial conditions
    z_past is the sequence of measurement
    step_future is the number of iteration in the future
    """
    I = eye(len(x0))
    
    A = KalmanStep.A
    Q = KalmanStep.Q 
    H = KalmanStep.H 
    R = KalmanStep.R 

    for z in z_past:
        ## Time Update
        x_ = dot(A, x_old)
        p_ = dot(dot(A, p_old), A.T) + Q
    
        ## Measurement Update
        k = dot(dot(p_, H.T), linalg.inv(dot( dot(H, p_),H.T)+R) )
        p = dot(I - dot(k,H),p_)
    
        #z = dot(H,x_old) 
        x = x_ + dot(k,(z-dot(H,x_)))
    
    for t in xrange(step_future):
        ## Time Update
        x = dot(A, x)
        p = dot(dot(A, p), A.T) + Q

    return z,x

def predict(trajectory, t):
    """ Return predicted position at time t

    TODO what's the role of time? How to set the reprate?

    :trajectory: [time, (x,y,z)]
    :t:  float
    """
    ## Init kalman
    x = trajectory[0][1]
    p = diag((1,1,3))     ## TODO How we decide it?
    
    n_steps = (t - x[-1][0])*sample_rate

    x,p = KalmanStep(x,p, z_past = trajectory[:][1], step_future = n_steps)
    return x

def init():
    ## Init the parameters
    sample_rate = 10 

    KalmanStep.A = eye(3)
    KalmanStep.Q = eye(3)
    KalmanStep.H = eye(3)
    KalmanStep.R = eye(3)

if __name__ == '__main__':
    ## Init node

    ## Create publisher

    ## Run it
    pass