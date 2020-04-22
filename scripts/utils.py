import numpy as np
import cv2
pi = np.pi

class point():
	'''
	Class for defining the state as (x,y, theta) with theta in radians 
	>>> a = point(0,1,0)
	>>> print(a)
	... Point(0.00,1.00,0.00)
	'''
    def __init__(self, x,y, t):
        self.x = x
        self.y = y
        self.t = t
        
    def __repr__(self):
         return 'Point({:.2f},{:.2f},{:.2f})'.format(self.x,self.y,self.t)
    def __str__(self):
        return 'Point({:.2f},{:.2f},{:.2f})'.format(self.x,self.y,self.t)

def nearest_point(p, point, alpha = 5):
	'''
	given a collection of points and a point in the configuration space, it returns nearest point in the configuration space\
	

	p-> collection of points (list of point objects)
	point-> point instance, the point for which we want to compute the nearest point
	alpha-> relative importance of the angle

	returns
	point obejct which is closest to 'point'
	'''
    dist = np.inf
    index = -1
    for i in range(len(p)):
        angle = np.arctan2((point.y-p[i].y),(point.x-p[i].x+0.01))
        angle = min(abs(angle - point.t),2*pi - abs(angle - point.t))
        new_dist = (point.x-p[i].x)**2 + (point.y-p[i].y)**2 + alpha*angle
        if dist> new_dist:
            index = i
            dist = new_dist
    return p[index]

def in_obstacle(cur_point):
	'''
	Define this function to return if a point is in obstacle space or not
	'''
    return False

def get_children(cur_point, rpm1, rpm2,r,c,wr,l, dt=1):
	'''
	Given a state, and 2 rpms and robot parameters, it computes the possible new states that
	emerge from this point not in the obstacle space

	cur_point-> current state
	rpm1, rpm2 -> 2 values of wheel speed in rpm
	r,c,wr,l -> radius, clearance, wheel radius and wheel distance
	dt -> discretized time 

	return 
	a list of point instances which emerge out of this current state
	'''
    rpm1 = rpm1*(pi/30)
    rpm2 = rpm2*(pi/30)
    v =[[0,rpm1],[rpm1,0],[rpm1,rpm1],[0,rpm2],[rpm2,0],[rpm2,rpm2],[rpm1,rpm2],[rpm2,rpm1]]
    children = []
    for i in range(8):
        ul = v[i][0]*wr
        ur = v[i][1]*wr
        
        if(ul==ur):
            new_state = point(cur_point.x+ul*dt*np.cos(cur_point.t)\
                             ,cur_point.y+ul*dt*np.sin(cur_point.t), \
                              cur_point.t)
        else:
            omega = (ur-ul)/l
            
            ir = (ur+ul)/(2*omega)
            
            iccx = cur_point.x - ir*np.sin(cur_point.t)
            iccy = cur_point.y + ir*np.cos(cur_point.t)
            
            new_state = point(0,0,0)
            new_state.x = (cur_point.x-iccx)*np.cos(omega*dt)-(cur_point.y-iccy)*np.sin(omega*dt)+iccx
            new_state.y = (cur_point.x-iccx)*np.sin(omega*dt)-(cur_point.y-iccy)*np.cos(omega*dt)+iccy
            new_state.t = omega*dt + cur_point.t
            
        while new_state.t<-pi: new_state.t +=2*pi
        while new_state.t>pi: new_state.t -=2*pi
            
        if not in_obstacle(new_state): children.append(new_state)
        
    return children 