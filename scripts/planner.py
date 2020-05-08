from utils import *
import heapq as hq

class rrtplanner():
	def __init__(self, paths, goal, possible_paths = None,draw = None):
		assert isinstance(paths, tree)
		assert isinstance(goal, node)

		self.rrttree = paths
		self.goal = goal
		self.threshold = 0.3
		self.radius = 1.5
		if possible_paths==None:
			self.possible_paths = {}
		else:
			self.possible_paths = possible_paths
		self.r = 0.105
		self.c = 0.1
		self.wr = 0.066
		self.l = 0.16
		self.size = 500

		if draw is None:
			self.img = np.ones((self.size,self.size,3))*255
			self.draw_img()
		else:
			self.img = draw

		# cv2.imshow("Map", self.img)
		# cv2.waitKey(0)

		#,r = 0.105,c=0.1,wr = 0.066,l = 0.16

	def grow_tree(self, func = 1):

		added = False
		while not added:
			if func:
				rndm_node = node(np.random.normal(self.goal.x,3),np.random.normal(self.goal.y,3),np.random.random()*2*pi-pi)
			else:
				rndm_node = node(np.random.random()*10-5,np.random.random()*10-5,np.random.random()*2*pi-pi)
			nn = nearest_point(self.rrttree.state, rndm_node)
			children = get_children(nn, 10,20,self.r,self.c,self.wr,self.l, dt = 1)
			if len(children)!=0:
				nearest_child = nearest_point(children, rndm_node)
				neighbours = nearest_neighbours(self.rrttree.state, nearest_child, radius = self.radius)
				parent = best_node(neighbours)
				# print(parent)
				nearest_child.cost = parent.cost+ euc_dist(parent, nearest_child)**0.5
				parent.add_child(nearest_child)
				# print(parent)
				nearest_child.parent = parent
				self.rrttree.state.append(nearest_child)
				if euc_dist(nearest_child, self.goal)**0.5<self.threshold: self.possible_paths[nearest_child] = \
				nearest_child.cost
				added = True
		return nearest_child

	def plan(self):
		while not len(self.possible_paths):
			new_node = self.grow_tree()			

	def best_path(self):
		cost = np.inf
		for i,c in self.possible_paths.items():
			if c<cost:
				cost = c
				path = i
		return path, cost

	def reconfigure(self):
		self.rrttree.reconfigure(self.radius)
		for i in self.rrttree.state:
			if euc_dist(i, self.goal)<self.threshold:
				self.possible_paths[i] = i.cost

	def get_waypoints(self):
		path, _ = self.best_path()
		waypoints=[]
		while path!=None:
			waypoints.append(path)
			path = path.parent
		return waypoints[::-1]


	def draw_img(self):
		# print("Here")
		startx, starty = cart2img(self.rrttree.state[0].x,self.rrttree.state[0].y, size = self.size)
		goalx, goaly = cart2img(self.goal.x,self.goal.y, size = self.size)
		cv2.circle(self.img, (int(starty), int(startx)) ,5, (255,255,0), -1)
		cv2.circle(self.img, (int(goaly),  int(goalx)),int(self.threshold*self.size/10), (255,0,255), -1)
		for i in range(self.img.shape[0]):
			for j in range(self.img.shape[1]):
				x,y = img2cart(i,j, size = self.size)
				obs_point = node(x,y,0)
				if in_obstacle(obs_point,self.r,self.c): self.img[i,j,:] = [0,0,0]



	def update_img(self):
		# img = np.ones((200,200,3))*255
		
		# for i in self.rrttree.state:
		# 	if i.parent!= None:
		# 		# print(i)
		# 		parentx, parenty = cart2img(i.parent.x, i.parent.y, size = self.size)
		# 		childx, childy = cart2img(i.x,i.y, size = self.size)
		# 		self.img = cv2.line(self.img,(int(parenty),int(parentx)),\
		# 					   (int(childy),int(childx)),(0.8,0.8,0.8), 1)
		# 		# cv2.imshow("Map", self.img)
		# 		# cv2.waitKey(1)

		waypoints = self.get_waypoints()

		color = (np.random.random(),np.random.random(),np.random.random())
		for i in range(len(waypoints)-1):
			parentx, parenty = cart2img(waypoints[i].x, waypoints[i].y, size = self.size)
			childx, childy = cart2img(waypoints[i+1].x,waypoints[i+1].y,size = self.size)
			self.img = cv2.line(self.img,(int(parenty),int(parentx)),\
						   (int(childy),int(childx)),color, 2)

		# cv2.imshow("Map", self.img)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()

		return self.img
		
# start = node(-4,-4,0)

# rrtree = tree(start)

# planner = rrtplanner(rrtree, node(4,4,0))

# planner.plan()
# waypoints = planner.get_waypoints()
# print(planner.rrttree.size())
# print(planner.best_path())

# img = planner.update_img()
# cv2.imshow("Map", img)
# cv2.waitKey(1000)
# rrtree = tree(waypoints[1])
# planner = rrtplanner(rrtree, node(4,4,0), possible_paths =  planner.possible_paths, draw = planner.img)
# planner.plan()
# for i in range(2000):
# 	planner.grow_tree(func = 0)
# # planner.reconfigure()
# print(planner.rrttree.size())
# print(planner.best_path())
# img = planner.update_img()
# cv2.imshow("Map", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()