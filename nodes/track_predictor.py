""" Tracking predictor

Using Kalman Filter, predicts the position of a target in the future
we dont know the current input(u), only have measurement z,
TODO Evaluate A,Q,H,R

TODO A should be learn rather then set by hand
TODO Does state need momentum (probably yes NdAF)?
"""
import os
import sys
import numpy
from numpy import *


def KalmanStep(x,p, z_past= [], step_future = 0):
    """ Perform Kalman filtering

    x,p are the intial conditions
    z_past is the sequence of measurement
    step_future is the number of iteration in the future
    """
    I = eye(4)

    A = KalmanStep.A
    Q = KalmanStep.Q 
    H = KalmanStep.H 
    R = KalmanStep.R 

    x_old=x
    p_old=p
    #for z in z_past:
        ## Time Update

    for z in z_past:
        print z
        x_ = dot(A, x_old)
        p_ = dot(dot(A, p_old), A.T) + Q
        
    
            ## Measurement Update
        k = dot(dot(p_, H.T), linalg.inv(dot( dot(H, p_),H.T)+R) )
        p = dot(I - dot(k,H),p_)
        z=numpy.array(z)[numpy.newaxis]
        
            #print dot(k,(z.T-dot(H,x_)))
            #z = dot(H,x_old) 
        x = x_ + dot(k,(z.T-dot(H,x_)))
        
        x_old=x
        p_old=p
    
    
    for t in xrange(step_future):
        ## Time Update
        
        x = dot(A, x_old)
        p = dot(dot(A, p_old), A.T) + Q
        


    return x,p

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

    KalmanStep.A = numpy.array([[1,0 ,.1, 0],[0, 1 ,0 ,.1],[0, 0 ,1 ,0],[0, 0 ,0 ,1]])

    KalmanStep.Q = eye(4) * 0.01
    KalmanStep.H = eye(4)
    KalmanStep.R = eye(4) * 0.01

if __name__ == '__main__':
    ## Init node
    init()
    ## Create publisher
    z_past=[]
    ## Run it
    with open('approach1_path.txt','r') as f:
        read_data=f.readlines()
    for line in read_data:
        line=line[:-1] #remove /n char
        line=line.split(',') #convert to list
        x=line[0]
        y=line[1]
        vx=line[2]
        vy=line[3]
        z_past.extend([[float(x) ,float(y) ,float(vx), float(vy)]])
    x=numpy.array([[float(z_past[0][0])],[float(z_past[0][1])],[float(z_past[0][2])],[float(z_past[0][3])]])
    
    z_past=numpy.array(z_past)
    p=eye(4)
   
    f=open('predicted_path.txt','w')
    f.close()
    f=open('predicted_path.txt','a')
    for i in xrange(0,len(z_past)):
        z=z_past[i:(i+5)]
        
        x,p = KalmanStep(x,p, z, 1)
        
        f.write(str(x[0][0])+','+str(x[1][0])+','+str(x[2][0])+','+str(x[3][0])+'\n')
   
    f.close()
        
