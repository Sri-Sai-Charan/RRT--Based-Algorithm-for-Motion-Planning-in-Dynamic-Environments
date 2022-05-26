import random
import math
import sys
import pygame
import timeit, time
import numpy as np
viz_anim = True

x_range = 800
y_range = 600
windowSize = [x_range, y_range]

pygame.init()
fpsClock = pygame.time.Clock()

screen = pygame.display.set_mode(windowSize)
pygame.display.set_caption('RRT* Fixed Nodes')


class RRT():
    def __init__(self, start, goal, obstacle_list,
                 randArea, step_size=15.0, goal_sample_rate=10, maxNodes=1500):

        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.Xrand = randArea[0]
        self.Yrand = randArea[1]
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.maxNodes = maxNodes
        self.obstacle_list = obstacle_list

    def Planning(self, animation=True):

        self.node_list = {0:self.start}
        i = 0
        while True:

            rnd = self.random_sample_point()
            # Get nearest node
            nind = self.get_nearest_neighbor_nodeind(rnd) 
            # generate new node from that nearest node in direction of random point
            newNode = self.grow_tree_node(rnd, nind) 

            if self.check_obstacle(newNode, self.obstacle_list): 
                # Find nearest nodes
                nearinds = self.get_neighbor_nodeinds(newNode, 5) 
                # From that nearest nodes find the best parent to newNode
                newNode = self.get_best_parent(newNode, nearinds) 
                # Add newNode
                self.node_list[i+100] = newNode 
                 # Make newNode a parent of another node if found
                self.rewire(i+100, newNode, nearinds)
                self.node_list[newNode.parent].children.add(i+100)

                if len(self.node_list) > self.maxNodes:
                    leaves = [ key for key, node in self.node_list.items() if len(node.children) == 0 and len(self.node_list[node.parent].children) > 1 ]
                    if len(leaves) > 1:
                        ind = leaves[random.randint(0, len(leaves)-1)]
                        self.node_list[self.node_list[ind].parent].children.discard(ind)
                        self.node_list.pop(ind)
                    else:
                        leaves = [ key for key, node in self.node_list.items() if len(node.children) == 0 ]
                        ind = leaves[random.randint(0, len(leaves)-1)]
                        self.node_list[self.node_list[ind].parent].children.discard(ind)
                        self.node_list.pop(ind)

            i+=1
            if animation and i%25 == 0:
                self.draw_graph(rnd)

            for e in pygame.event.get():
                if e.type == pygame.MOUSEBUTTONDOWN:
                    if e.button == 1:
                        self.obstacle_list.append((e.pos[0],e.pos[1],30,30))
                        self.validate_path()
                    elif e.button == 3:
                        self.end.x = e.pos[0]
                        self.end.y = e.pos[1]
                        self.validate_path()

    def validate_path(self):
        lastIndex = self.get_best_goalind()
        if lastIndex is not None:
            while self.node_list[lastIndex].parent is not None:
                nodeInd = lastIndex
                lastIndex = self.node_list[lastIndex].parent

                dx = self.node_list[nodeInd].x - self.node_list[lastIndex].x
                dy = self.node_list[nodeInd].y - self.node_list[lastIndex].y
                d = math.sqrt(dx ** 2 + dy ** 2)
                theta = math.atan2(dy, dx)
                if not self.check_obstacle_in_path(self.node_list[lastIndex].x, self.node_list[lastIndex].y, theta, d):
                    self.node_list[lastIndex].children.discard(nodeInd)
                    self.remove_branch(nodeInd)

    def remove_branch(self, nodeInd):
        for ix in self.node_list[nodeInd].children:
            self.remove_branch(ix)
        self.node_list.pop(nodeInd)

    def get_best_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.node_list[i].x
            dy = newNode.y - self.node_list[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_obstacle_in_path(self.node_list[i].x, self.node_list[i].y, theta, d):
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

    def grow_tree_node(self, rnd, nind):
        nearestNode = self.node_list[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = Node(nearestNode.x, nearestNode.y)
        newNode.x += self.step_size * math.cos(theta)
        newNode.y += self.step_size * math.sin(theta)
        newNode.cost = nearestNode.cost + self.step_size
        newNode.parent = nind 
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

    def get_neighbor_nodeinds(self, newNode, value):
        r = self.step_size * value
        dlist = np.subtract( np.array([ (node.x, node.y) for node in self.node_list.values() ]), (newNode.x,newNode.y))**2
        dlist = np.sum(dlist, axis=1)
        nearinds = np.where(dlist <= r ** 2)
        nearinds = np.array(list(self.node_list.keys()))[nearinds]
        return nearinds

    def rewire(self, newNodeInd, newNode, nearinds):
        for i in nearinds:
            nearNode = self.node_list[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_obstacle_in_path(nearNode.x, nearNode.y, theta, d):
                    self.node_list[nearNode.parent].children.discard(i)
                    nearNode.parent = newNodeInd
                    nearNode.cost = scost
                    newNode.children.add(i)

    def check_obstacle_in_path(self, nix, niy, theta, d):

        tmpNode = Node(nix,niy)

        for i in range(int(d/5)):
            tmpNode.x += 5 * math.cos(theta)
            tmpNode.y += 5 * math.sin(theta)
            if not self.check_obstacle(tmpNode, self.obstacle_list):
                return False

        return True

    def draw_graph(self, rnd=None):
        screen.fill((255, 255, 255))
        for node in self.node_list.values():
            if node.parent is not None:
                pygame.draw.line(screen,(0,255,0),[self.node_list[node.parent].x,self.node_list[node.parent].y],[node.x,node.y])

        for node in self.node_list.values():
            if len(node.children) == 0: 
                pygame.draw.circle(screen, (255,0,255), [int(node.x),int(node.y)], 2)
                
        for(sx,sy,ex,ey) in self.obstacle_list:
            pygame.draw.rect(screen,(0,0,0), [(sx,sy),(ex,ey)])

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


    def get_nearest_neighbor_nodeind(self, rnd):
        dlist = np.subtract( np.array([ (node.x, node.y) for node in self.node_list.values() ]), (rnd[0],rnd[1]))**2
        dlist = np.sum(dlist, axis=1)
        minind = list(self.node_list.keys())[np.argmin(dlist)]
        return minind

    def check_obstacle(self, node, obstacle_list):
        for(sx,sy,ex,ey) in obstacle_list:
            sx,sy,ex,ey = sx+2,sy+2,ex+2,ey+2
            if node.x > sx and node.x < sx+ex:
                if node.y > sy and node.y < sy+ey:
                    return False
        return True 


class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None
        self.children = set()


def main():
    
    obstacle_list = [
        (400, 380, 400, 20),
        (400, 220, 20, 180),
        (500, 280, 150, 20),
        (0, 500, 100, 20),
        (500, 450, 20, 150),
        (400, 100, 20, 80),
        (100, 100, 100, 20)
    ]  
    rrt = RRT(start=[20, 580], goal=[540, 150],
              randArea=[x_range, y_range], obstacle_list=obstacle_list)
    print("Executing RRT*FN now...")
    path = rrt.Planning(animation=viz_anim)


if __name__ == '__main__':
    main()
