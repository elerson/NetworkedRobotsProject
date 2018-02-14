#!/usr/bin/python

from math import sqrt

def dist_to_segment( p, q, r):
	#p->q, r is external

	alpha = (r[0]*q[0] - r[0]*p[0] + r[1]*q[1] - r[1]*p[1] - p[0]*(q[0]-p[0]) - p[1]*(q[1] - p[1]))/( (q[1] - p[1])**2 + (q[0]-p[0])**2 )
	#print alpha
	if(alpha < 0 or alpha > 1):
		if(alpha < 0):
			return p
		if(alpha > 1):
			return q
	else:
		return (p[0] + alpha*(q[0] - p[0]), p[1] + alpha*(q[1] - p[1]))

def dist_to_segment_alpha( p, q, r):
	#p->q, r is external

	alpha = (r[0]*q[0] - r[0]*p[0] + r[1]*q[1] - r[1]*p[1] - p[0]*(q[0]-p[0]) - p[1]*(q[1] - p[1]))/( (q[1] - p[1])**2 + (q[0]-p[0])**2 )
	return alpha




