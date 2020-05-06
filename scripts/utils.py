import numpy as np
import cv2
pi = np.pi

class pid():

	def __init__(self, p, i, d):

		self.p = p
		self.i = i
		self.d = d

		self.prev_error= 0
		self.sum_ = 0

	def control(self, eror):
		u = self.p*error + self.d*(error - prev_error) + self.i*sum_

		self.sum_ += error
		self.prev_error = error

		return u

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
    def is_same(self, p):
        return p.x==self.x and p.y == self.y and p.t == self.t

def euc_dist(p1, p2):
    return (p1.x-p2.x)**2 + (p1.y-p2.y)**2

def config_dist(start, goal,alpha = 2):
    angle = np.arctan2((goal.y-start.y),(goal.x-start.x+0.01))
    angle = min(abs(angle - start.t),2*pi - abs(angle - start.t))
    return euc_dist(start, goal) + alpha*angle


def best_node(p, cost_map):
    cost = np.inf
    for i in range(len(p)):
        if cost> cost_map[p[i]]:
            cost = cost_map[p[i]]
            index = i
    return p[index]

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
#         angle = np.arctan2((point.y-p[i].y),(point.x-p[i].x+0.01))
#         angle = min(abs(angle - point.t),2*pi - abs(angle - point.t))
        new_dist = euc_dist(p[i], point)
        if dist> new_dist:
            index = i
            dist = new_dist
    return p[index]

def nearest_neighbours(p, point, radius=  0.5):
    neighbours =  []
    for i in p: 
        if radius*radius > euc_dist(i, point): neighbours.append(i)
    return neighbours

def in_obstacle(cur_point,r,c):
	'''
	returns if a point is in obstacle space or not

	cur_point -> current state
	r,c -> radius and clearnace

	return boolean
	'''
	x,y = cur_point.x, cur_point.y
    return (x)*(x) + (y)*(y) <= (1+r+c)*(1+r+c) or\
    (x-2)*(x-2) + (y-3)*(y-3) <= (1+r+c)*(1+r+c) or\
    (x-2)*(x-2) + (y+3)*(y+3) <= (1+r+c)*(1+r+c)or\
    (x+2)*(x+2) + (y+3)*(y+3) <= (1+r+c)*(1+r+c) or\
    x<-5-r-c or x>5+r+c or y<-5-r-c or y>5+r+c or\
    (x>=-4.75-r-c and x<=-3.25+r+c and y>= -0.75-r-c and y<= 0.75+r+c) or\
    (x>=-2.75-r-c and x<=-1.25+r+c and y>= 2.25-r-c and y<= 3.75+r+c) or\
    (x>=3.25-r-c and x<=4.75+r+c and y>= -0.75-r-c and y<= 0.75+r+c)
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


'''
SAMPLE TRIAL

state = [point(-4,-4,0)]
goal = point(4,4,0)
img = np.ones((200,200))*255
for i in range(2000):
    rndm_node = point(np.random.randint(-5,5),np.random.randint(-5,5),np.random.random()*2*pi-pi)
    nn = nearest_point(state, rndm_node)
    children = get_children(nn, 30, 40,0,0,0.066,0.16, dt = 1) # These values are with respect to turtlebot3 burger
    if len(children)!=0: 
        state.append(nearest_point(children, rndm_node))
        img = cv2.line(img,(int(nn.x*20)+100,int(nn.y*20)+100),(int(state[-1].x*20)+100,int(state[-1].y*20)+100),0, 1)
    cv2.imshow('rrt', img)
    cv2.waitKey(10)
cv2.destroyAllWindows()

'''

def rrt_star(goal,start, rpm1, rpm2, threshold = 0.35,r = 0.105,c=0.06,wr = 0.066,l = 0.16, visualisation = True):
	'''
	Implements RRT*
	input:

	goal -> point object with goal cordinates
	start-> point object with goal cordinates
	rp1, rpm2-> wheel angular speeds
	theshold-> how close we should reach the goal
	r,c,wr,l -> robot radius, clearance, wheel radius, wheel distance
	visualization -> (bool) if true, shows the animation
	'''
    
    state = [start]
    backtrack = {}
    cost = {start:0}
    
    if visualisation:
        img = np.ones((200,200,3))*255
        cv2.circle(img, (int(state[0].x*20)+100,int(state[0].y*20)+100),5, (255,255,0), -1)
        cv2.circle(img, (int(goal.x*20)+100,int(goal.y*20)+100),int(threshold*20), (255,0,255), -1)

    print('starting search')
    while euc_dist(state[-1], goal)**0.5> threshold:
        
        rndm_node = point(np.random.normal(goal.x,0.1),np.random.normal(goal.y,0.1),np.random.random()*2*pi-pi)
#         rndm_node = point(np.random.randint(-5,5),np.random.randint(-5,5),np.random.random()*2*pi-pi)
        nn = nearest_point(state, rndm_node)
        children = get_children(nn, rpm1,rpm2,r,c,wr,l, dt = 1)
        
        
        if len(children)!=0: 
            
            nearest_child = nearest_point(children, rndm_node)
            neighbours = nearest_neighbours(state, nearest_child, radius = 1.5)
            parent = best_node(neighbours, cost)

            backtrack[nearest_child] = parent
            cost[nearest_child] = cost[parent]+1
            state.append(nearest_child)
            
            
            if visualisation:
                img = cv2.line(img,(int(parent.x*20)+100,int(parent.y*20)+100),\
                               (int(nearest_child.x*20)+100,int(nearest_child.y*20)+100),(255,0,0), 1)
                cv2.imshow('rrt', img)
                cv2.waitKey(20)
    print('search ended, started backtracking')
    
    
    goal = state[-1]
    while not goal.is_same(state[0]):
        
        
        back = backtrack[goal]
        
        if visualisation:
            img = cv2.line(img,(int(back.x*20)+100,int(back.y*20)+100),\
                           (int(goal.x*20)+100,int(goal.y*20)+100),(0,0,255), 2)
            cv2.imshow('rrt', img)
            cv2.waitKey(20)
        
        goal = back
    
    if visualisation:
        cv2.imshow('rrt', img)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()
    
    return cost[state[-1]]