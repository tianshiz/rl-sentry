#!/usr/bin/env python
import re
import fof
import sys
import os
import numpy
sys.path.insert(0, '../libsvm-3.14/python')
from svmutil import *

if __name__ == "__main__":
	#define names for friendly/hostile files
	friendly=['approach','walkby','stand']
	hostile=['creep','crouch','sidle','lookaround','fast','direct']
	y=[]
	x=[]
	fnum=0

	#loop through files in /data folder
	for fn in os.listdir('../data/'):
		fnum+=1
		file_name=fn[:-5] #truncate text and number
		if os.path.isdir('../data/'+fn) == False:
			f=fof.loadPose('../data/'+fn)
			N=len(f) #number of vectors
			for index in xrange(0,N):
				if file_name in friendly:
					y.append(1)
				elif file_name in hostile:
					y.append(-1)
				else:
					print file_name

				index2=index+1
				#get single vector length 45	
				vect=fof.extractPoseFeature(f[index:index2])
				big=max(vect)
				print big
				small=min(vect)
				vect[:]=[(p-small)/(big-small) for p in vect]

				x.extend([vect])
		
	#begin svm training
	best_m=0
	prob=svm_problem(y,x)
	#5 fold xvalidation
	for log2c in xrange(-1,4):
		for log2g in xrange(-4,2):

			param = svm_parameter('-v 5 -c '+str(2**log2c) +' -g '+str(2**log2g))
			m = svm_train(prob, param)
			if m>best_m:
				best_m=m
				bestg=2**log2g
				bestc=2**log2c

	param = svm_parameter('-c '+str(bestc) +' -g '+str(bestg))
	m = svm_train(prob, param)
	svm_save_model('heart_scale.model', m)
	#m = svm_load_model('heart_scale.model')
	#get test data

	yp=[]
	xp=[]
	for fn in os.listdir('../data/test'):
		file_name=fn[:-5] #truncate text and number

		f=fof.loadPose('../data/test/'+fn)
		N=len(f) #number of vectors
		for index in xrange(0,N):
			if file_name in friendly:
				yp.append(1)
			elif file_name in hostile:
				yp.append(-1)
			else:
				print file_name

			index2=index+1
			#get single vector length 45	
			vect=fof.extractPoseFeature(f[index:index2])
			big=max(vect)
			small=min(vect)
			vect[:]=[(p-small)/(big-small) for p in vect]
			xp.extend([vect])	

	p_label, p_acc, p_val = svm_predict(yp, xp, m)
	ACC, MSE, SCC = evaluations(yp, p_label)

	#f=fof.loadPose('friendly_1.txt')
	#print len(fof.extractPoseFeature(f[0:1]))
