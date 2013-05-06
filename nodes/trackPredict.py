""" Tracking Predictor

Using Kalman Filter, predicts the position of a target in the future
we dont know the current input(u), only have measurement z,

TODO Evaluate A,Q,H,R
TODO A should be learn rather then set by hand
TODO Does state need momentum (probably yes)?
"""
import os
import sys
import numpy
from numpy import *

def KalmanStep(x,p, z_past= [], step_future = 0):
  """ Perform Kalman filtering

  Parameters
  ----------
    x : ndarray
      initial estimate of position in the format [x, y, vx, vy]
    p : ndarray
      initial estimate of error covariance
    z_past: ndarray
      sequence of state measurements in the format 
      [[x1, y1, vx1, vy1]
       [x1, y1, vx1, vy1]
               .
               .
       [xn, yn, vxn, vyn]]
    step_future: int
      number of iteration into the future

  Returns
  -------
    x : ndarray
      updated estimate of position
    p : ndarray
      updated estimate of error covariance
      
  """
  I = eye(4)

  A = KalmanStep.A
  Q = KalmanStep.Q
  H = KalmanStep.H
  R = KalmanStep.R

  x_old=x[:,numpy.newaxis]
  p_old=p

  for z in z_past:
    ## Time Update
    x_ = dot(A, x_old)
    p_ = dot(dot(A, p_old), A.T) + Q
        
    ## Measurement Update
    k = dot(dot(p_, H.T), linalg.inv(dot( dot(H, p_),H.T)+R) )
    p = dot(I - dot(k,H),p_)

    x = x_ + dot(k,(z[:,numpy.newaxis]-dot(H,x_)))
        
    x_old=x
    p_old=p

  for t in xrange(step_future):
    ## Time Update
    x = dot(A, x_old)
    p = dot(dot(A, p_old), A.T) + Q
        
    x_old=x
    p_old=p

  return x[:,0],p

def init():
  """ Init the parameters
  """
  KalmanStep.A = numpy.array([[1,0 ,.1, 0],[0, 1 ,0 ,.1],[0, 0 ,1 ,0],[0, 0 ,0 ,1]])

  KalmanStep.Q = eye(4) * 0.01
  KalmanStep.H = eye(4)
  KalmanStep.R = diag((0.1,0.1,.9,.9))

def predict(trajectory, steps):
  """ Return predicted trajectory of object over a given number
  steps in the future

  Parameters
  ----------
    trajectory : ndarray
      array of past positions and velocities in the format 
      [[x1, y1, vx1, vy1]
       [x1, y1, vx1, vy1]
               .
               .
       [xn, yn, vxn, vyn]]
    steps: int
      number of steps into the future to predict

  Returns
  -------
    pred : ndarray
      predicted trajectory
  """
  pred = []
  for j in xrange(0,steps):
    pred.append(KalmanStep(trajectory[-1], eye(4), trajectory, j)[0])
  pred = array(pred)

  return pred

if __name__ == '__main__':
  # Init node
  init()
