import random
import math
import copy
import sys
import pygame
import timeit
import numpy as np
viz_anim = True

x_range = 720
y_range = 500
windowSize = [x_range, y_range]

pygame.init()
fpsClock = pygame.time.Clock()

screen = pygame.display.set_mode(windowSize)
screen.fill((255, 255, 255))



class RRT():
    def __init__(self, start, goal, obstacle_list,
                 randArea, step_size=5.0, goal_sample_rate=15, maxNodes=200, maxIter=10000):
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.Xrand = randArea[0]
        self.Yrand = randArea[1]
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.maxNodes = maxNodes
        self.obstacle_list = obstacle_list
        self.maxIter = maxIter

    def Planning(self, animation=True):
        self.node_list = {0:self.start}
        i = 0
        m = 4

        while i<self.maxIter:
            i+=1
            rnd = self.random_sample_point()
            # get nearest node index
            nind = self.get_nearest_neighbor_nodeind(self.node_list, rnd) 
            # get new node from nearest node in direction of random point
            newNode = self.grow_tree_node(rnd, nind) 

            if self.check_obstacle(newNode, self.obstacle_list):
                # find nearest nodes to newNode
                nearinds = self.get_neighbor_nodeinds(newNode, 5) 
                # From that nearest nodes find the best parent
                newNode = self.get_best_parent(newNode, nearinds)
                self.node_list[newNode.parent].leaf = False

                self.node_list[i] = newNode # add new node to list
                 ## make newNode a parent of another node if found
                self.rewire(i, newNode, nearinds) 

                root = self.node_list[0]
                if (root.x == 0 and root.y == 0) or (root.x == x_range and root.y == 0) or (root.x == x_range and root.y == y_range) or (root.x == 0 and root.y == y_range):
                    m += 1
          
                ## If max limit is reached:
                if i > self.maxNodes:
                    ## Get all leaf nodes
                    leaves = [ key for key, node in self.node_list.items() if node.leaf == True ]
                    ## pick one leaf randomly
                    ind = leaves[random.randint(0, len(leaves)-1)]

                    # Cut the tree connection b/w selected leaf and its parent, basically make parent leaf
                    self.node_list[self.node_list[ind].parent].leaf = True
                    ## Check if there are other nodes which had the same parent, if yes, make the parent !leaf again
                    for value in self.node_list.values():
                        if value.parent == self.node_list[ind].parent and value != self.node_list[ind]:
                            self.node_list[self.node_list[ind].parent].leaf = False
                            break

                    self.node_list.pop(ind)

            if animation and i%10 == 0:
                self.draw_graph(rnd)

            
        lastIndex = self.get_best_goalind()
        if lastIndex is None:
            return None
        path = self.get_complete_path(lastIndex)
        return path

    ## Choose parent from neighbourhood nodes
    ## For every neigh. node, find closest node for which there is no collision along the path of 5 steps
    def get_best_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.node_list[i].x
            dy = newNode.y - self.node_list[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_obstacle_in_path(self.node_list[i], theta, d):
                dlist.append(self.node_list[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            return newNode

        newNode.cost = mincost
        newNode.parent = minind
        return newNode

    ## Regrow 
    def grow_tree_node(self, rnd, nind):
        nearestNode = self.node_list[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = copy.deepcopy(nearestNode)
        newNode.x += self.step_size * math.cos(theta)
        newNode.y += self.step_size * math.sin(theta)

        newNode.cost += self.step_size
        newNode.parent = nind
        newNode.leaf = True
        return newNode

    def random_sample_point(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(0, self.Xrand), random.uniform(0, self.Yrand)]
        else:
            rnd = [self.end.x, self.end.y]
        return rnd

    def get_best_goalind(self):
        disglist = [(key, self.calc_dist_to_goal(node.x, node.y)) for key, node in self.node_list.items()]
        goalinds = [key for key, distance in disglist if distance <= self.step_size]
        if len(goalinds) == 0:
            return None

        mincost = min([self.node_list[key].cost for key in goalinds])
        for i in goalinds:
            if self.node_list[i].cost == mincost:
                return i

        return None

    def get_complete_path(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.node_list[goalind].parent is not None:
            node = self.node_list[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    ## Find nodes in the neighbourhood. value defines number of steps that will be considered as neighbourhood
    def get_neighbor_nodeinds(self, newNode, value):
        r = self.step_size * value
        dlist = [(key, (node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2) for key, node in self.node_list.items()]
        nearinds = [key for key, distance in dlist if distance <= r ** 2]
        return nearinds

    def rewire(self, newNodeInd, newNode, nearinds):
        nnode = len(self.node_list)
        for i in nearinds:
            nearNode = self.node_list[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            ## If cost of going from newNode to nearNode is lesser than cost of nearNode,
            ## then rewire nearNode parent to be newNodeInd. Essentially, now nearNode will be reached
            ## from newNode
            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_obstacle_in_path(nearNode, theta, d):
                    self.node_list[nearNode.parent].leaf = True
                    for value in self.node_list.values():
                        if value.parent == nearNode.parent and value != nearNode:
                            self.node_list[nearNode.parent].leaf = False
                            break

                    nearNode.parent = newNodeInd
                    nearNode.cost = scost
                    newNode.leaf = False

    ## Check collision in every step of nearest node direction
    def check_obstacle_in_path(self, nearNode, theta, d):

        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.step_size)):
            tmpNode.x += self.step_size * math.cos(theta)
            tmpNode.y += self.step_size * math.sin(theta)
            if not self.check_obstacle(tmpNode, self.obstacle_list):
                return False

        return True

    def draw_graph(self, rnd=None):
        screen.fill((255, 255, 255))
        for node in self.node_list.values():
            if node.parent is not None:
                pygame.draw.line(screen,(0,255,0),[self.node_list[node.parent].x,self.node_list[node.parent].y],[node.x,node.y])

        for node in self.node_list.values():
            if node.leaf == True:
                pass

        for (ox, oy, size) in self.obstacle_list:
            pygame.draw.circle(screen, (0,0,0), [ox,oy], size)

        pygame.draw.circle(screen, (255,0,0), [self.start.x, self.start.y], 10)
        pygame.draw.circle(screen, (0,0,255), [self.end.x, self.end.y], 10)

        lastIndex = self.get_best_goalind()
        if lastIndex is not None:
            path = self.get_complete_path(lastIndex)

            ind = len(path)
            while ind > 1:
                pygame.draw.line(screen,(255,0,0),path[ind-2],path[ind-1])
                ind-=1

        pygame.display.update()


    def get_nearest_neighbor_nodeind(self, node_list, rnd):
        dlist = [ (key, (node.x - rnd[0]) ** 2 + (node.y - rnd[1])** 2) for key, node in node_list.items()]
        minind = min(dlist, key=lambda d: d[1])
        return minind[0]

    def check_obstacle(self, node, obstacle_list):
        for (ox, oy, size) in obstacle_list:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if d <= size ** 2:
                return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None
        self.leaf = True


def main():
    print("start RRT path planning")

    # ====Search Path with RRT====
    obstacle_list = [
        (50, 50, 10),
        (400, 200, 15),
        (400, 250, 25),
        (300, 180, 20),
        #(7, 5, 2),
        #(9, 5, 2)
    ]  # [x,y,size]
    # Set Initial parameters
    rrt = RRT(start=[1, 0], goal=[500, 300],
              randArea=[x_range, y_range], obstacle_list=obstacle_list)
    path = rrt.Planning(animation=viz_anim)


    # Draw final path
    if viz_anim:

        rrt.draw_graph()

        ind = len(path)
        while ind > 1:
            pygame.draw.line(screen,(255,0,0),path[ind-2],path[ind-1])
            ind-=1

        pygame.display.update()

        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT or (e.type == pygame.KEYUP and e.key == pygame.K_ESCAPE):
                    sys.exit("Exiting")

if __name__ == '__main__':
    main()