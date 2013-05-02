#!/usr/bin/env python
""" poseClassify

modified version of pose_classifier that can be used by mastermind.py

Classifies given poses and returns robot state
"""

import frameExtract
import sys
import numpy
sys.path.insert(0,'../libsvm-3.17/python')
from svmutil import *

from PyKDL import *

# array of states robot can be in
STATES = ["Neutral", "Friendly", "Hostile", "HOSTILE"]

def classify_frame(frame,m):
  """Classify an individual frame as either friendly or hostile
  output confidence values 
 
 params:
   :frame: used to find joint positions and orientations
   :m: model used to classify frames
 
 returns:
   :state: confidence value of classification 
 
  """

#  xp = []
  vect = frameExtract.extractPoseFeatures(frame)
  me=numpy.mean(vect)
  se=numpy.std(vect)
  vect[:]=[(p-float(me)/float(se)) for p in vect]
#  xp.extend([vect])
  p_label, p_acc, p_val = svm_predict([0]*len(vect), vect, m, '-q')
  
  # PR2 Sentry States
  NEUTRAL  = 1
  FRIENDLY = 0
  WHOSTILE = 2  
  SHOSTILE = 3
  # Initial state is neutral  
  state = NEUTRAL  
  pval = [0]*10
  s_pval = 0
  
  print 'Current State: ', STATES[state], 'pval = ', s_pval

  # constantly shift in confidence values, use their sums to determine state transitions
  pval[:-1] = pval[1:]
  pval[-1] =  p_val[0][0]
  s_pval = sum(pval)
  if state == NEUTRAL:
    if s_pval > 9:
      state = FRIENDLY
      pval = [0]*10
    elif s_pval < -9:
      state = WHOSTILE
      pval = [0]*10
  elif state == FRIENDLY:
    if s_pval < -8:
      state = WHOSTILE
      pval = [0]*10
  elif state == WHOSTILE:     
    if s_pval > 9:
      state = NEUTRAL
      pval = [0]*10
    elif s_pval < -12:
      state = SHOSTILE
      pval = [0]*10
  else:
    if s_pval > 9:
      state = WHOSTILE 
      pval = [0]*10

  return state


