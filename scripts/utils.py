import numpy as np
import cv2
pi = np.pi


def img2cart(i,j):
	return (j-100)*1.0/20, (100-i)*1.0/20

def cart2img(x,y):
	return 100-y*20,x*20+100


class node():
	def __init__(self,x,y,t, parent=None):
		# assert isinstance(cord, point)
		assert isinstance(parent, node) or parent == None
		self.x = x
		self.y = y
		self.t = t
		self.parent = parent
		self.cost = 0
		self.children = []

	def add_child(self, cord):
		assert isinstance(cord, node)
		self.children.append(cord)
		cord.parent = self
	def __repr__(self):
		return 'Node({:.2f},{:.2f},{:.2f})'.format(self.x,self.y,self.t)
	def __str__(self):
		return 'Node({:.2f},{:.2f},{:.2f})'.format(self.x,self.y,self.t)
	def is_same(self, p):
		return p.x==self.x and p.y == self.y and p.t == self.t

class tree():
	def __init__(self, root):
		assert isinstance(root, node)
		self.root = root
		self.state = []
		self.add_children(root)

	def add_children(self, node):
		self.state.append(node)
		for i in node.children:
			self.add_children(i)

	def size(self):
		return len(self.state)

	def update_cost(self, node):
		for i in node.children:
			if i.parent != node:
				node.children.remove(i)
			else:
				i.cost = node.cost+euc_dist(i, node)
				self.update_cost(i)

	def reconfigure(self, radius):
		for i in self.state:
			neighbours = nearest_neighbours(self.state, i, radius = radius)
			parent = best_node(neighbours)
			i.parent = parent

		self.update_cost(self.root)

def euc_dist(p1, p2):
	return (p1.x-p2.x)**2 + (p1.y-p2.y)**2

def best_node(p):
	cost = np.inf
	for i in range(len(p)):
		if cost> p[i].cost:
			index = i
			cost = p[i].cost
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
	x<-5+r+c or x>5-r-c or y<-5+r+c or y>5-r-c or\
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
			new_state = node(cur_point.x+ul*dt*np.cos(cur_point.t)\
							 ,cur_point.y+ul*dt*np.sin(cur_point.t), \
							  cur_point.t)
		else:
			omega = (ur-ul)/l
			
			ir = (ur+ul)/(2*omega)
			
			iccx = cur_point.x - ir*np.sin(cur_point.t)
			iccy = cur_point.y + ir*np.cos(cur_point.t)
			
			new_state = node(0,0,0)
			new_state.x = (cur_point.x-iccx)*np.cos(omega*dt)-(cur_point.y-iccy)*np.sin(omega*dt)+iccx
			new_state.y = (cur_point.x-iccx)*np.sin(omega*dt)-(cur_point.y-iccy)*np.cos(omega*dt)+iccy
			new_state.t = omega*dt + cur_point.t
			
		while new_state.t<-pi: new_state.t +=2*pi
		while new_state.t>pi: new_state.t -=2*pi
			
		if not in_obstacle(new_state,r,c+0.4): children.append(new_state)
		
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
'''
def rrt_star(start, goal, rpm1, rpm2, threshold = 0.35,r = 0.105,c=0.1,wr = 0.066,l = 0.16, visualisation = True):
	
	Implements RRT*
	input:

	goal -> point object with goal cordinates
	start-> point object with goal cordinates
	rp1, rpm2-> wheel angular speeds
	theshold-> how close we should reach the goal
	r,c,wr,l -> robot radius, clearance, wheel radius, wheel distance
	visualization -> (bool) if true, shows the animation
	

	state = [start]
	backtrack = {}
<<<<<<< HEAD
=======
	rack = {}
>>>>>>> 2795e4d6974317e73698486ba8261747e7afcc97
	cost = {start:0}
	
	if visualisation:
		img = np.ones((200,200,3))*255
		startx, starty = cart2img(state[0].x,state[0].y)
		goalx, goaly = cart2img(goal.x,goal.y)
		cv2.circle(img, (int(starty), int(startx)) ,5, (255,255,0), -1)
		cv2.circle(img, (int(goaly),  int(goalx)),int(threshold*20), (255,0,255), -1)
		for i in range(img.shape[0]):
			for j in range(img.shape[1]):
				x,y = img2cart(i,j)
				obs_point = point(x,y,0)
				if in_obstacle(obs_point,r,c): img[i,j,:] = [0,0,0]

	print('starting search')
	while euc_dist(state[-1], goal)**0.5> threshold:
		
		# rndm_node = point(np.random.normal(goal.x,10),np.random.normal(goal.y,10),np.random.random()*2*pi-pi)
		rndm_node = point(np.random.randint(-5,5),np.random.randint(-5,5),np.random.random()*2*pi-pi)
		nn = nearest_point(state, rndm_node)
		children = get_children(nn, rpm1,rpm2,r,c,wr,l, dt = 1)
		
		
		if len(children)!=0: 
			
			nearest_child = nearest_point(children, rndm_node) # create node, vlue, parent, children
			neighbours = nearest_neighbours(state, nearest_child, radius = 1.5)
			parent = best_node(neighbours, cost)

			backtrack[nearest_child] = parent
			# nearest_child.parent = parent
			#parent.children.append(nearest_child)
			cost[nearest_child] = cost[parent]+1
			# if cost[nearest child]>11:
				# dont append
			# if cost is < 11:
			state.append(nearest_child)
			
			
			if visualisation:
				parentx, parenty = cart2img(parent.x, parent.y)
				childx, childy = cart2img(nearest_child.x,nearest_child.y)
				img = cv2.line(img,(int(parenty),int(parentx)),\
							   (int(childy),int(childx)),(255,0,0), 1)
				cv2.imshow('rrt', img)
				cv2.waitKey(20)
	print('search ended, started backtracking')
	
	
	goal = state[-1]
	waypoints = [goal]
	while not goal.is_same(state[0]):
		
		back = backtrack[goal]
		
		if visualisation:
			backx, backy = cart2img(back.x,back.y)
			goalx, goaly = cart2img(goal.x,goal.y)
			img = cv2.line(img,(int(backy),int(backx)),\
						   (int(goaly),int(goalx)),(0,0,255), 2)
			cv2.imshow('rrt', img)
			cv2.waitKey(20)
		
		goal = back
		waypoints.append(goal)
	
	if visualisation:
		cv2.imshow('rrt', img)
		cv2.waitKey(1000)
		# cv2.destroyAllWindows()
	
	return waypoints[::-1]

def config_dist(start, goal,alpha = 2):
	angle = np.arctan2((goal.y-start.y),(goal.x-start.x+0.01))
	angle = min(abs(angle - start.t),2*pi - abs(angle - start.t))
	return euc_dist(start, goal) + alpha*angle

'''