#!/usr/bin/python
import math
import numpy as np
from scipy.optimize import linear_sum_assignment


class Tree:
    def __init__(self, filename):
        self.filename = filename
        self.graph_adj_list = {}
        self.graph_vertex_position = {}
        self.clients = []
        self.vertices = []
        self.readTree()
        pass
    def readTree(self):
        VERTEX, VERTEX_CONNECTION, VERTEX_POSITION = range(3)

        state = VERTEX
        vertex = -1
        with open(self.filename) as f: 
            for line in f: 
                line_data = line.split(" ")

                #read the vertex positions
                if(state == VERTEX_POSITION):
                    for i in range(len(self.graph_adj_list)):
                        self.graph_vertex_position[i] = (float(line_data[2*i]), float(line_data[2*i + 1]))

                    break

                #read the adjacence list
                for data in line_data:
                    #end of adjacence list
                    if(int(data) == -2):
                        state = VERTEX_POSITION
                        break

                    if(state == VERTEX): #read the vertex information
                        vertex = int(data)
                        self.graph_adj_list[vertex] = []
                        state = VERTEX_CONNECTION

                    elif(state == VERTEX_CONNECTION):
                        if(int(data) >= 0):
                            self.graph_adj_list[vertex].append(int(data))
                        
                        #change the read state
                        if(int(data) == -1):
                            state = VERTEX
        self.getClients__()
        pass
    def getClients__(self):
        for vertex in self.graph_adj_list:
            if(len(self.graph_adj_list[vertex]) == 1):
                self.clients.append(vertex)
            self.vertices.append(vertex)

    def getDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


    def getSize(self):
        distance = 0
        for vertex1 in self.graph_adj_list:
            for vertex2 in self.graph_adj_list[vertex1]:
                p1 = self.graph_vertex_position[vertex1]
                p2 = self.graph_vertex_position[vertex2]
                d = self.getDistance(p1, p2)
                distance += d

        return distance/2


    def print_(self):
        print(self.graph_adj_list)
        print(self.graph_vertex_position)
        print(self.clients)


class TreeSegmention:
    def __init__(self, tree):
        self.tree = tree
        self.segmentaion_paths = {}
        self.size = tree.getSize()

    #segment the tree minimazing the maximal segment distance
    def segmentTree(self):
        pass

    
    def getClients__(self, used_vertex):
        for vertex in tree.graph_adj_list:
            if(len(tree.graph_adj_list[vertex]) == 1):
                tree.clients.append(vertex)


    # def breadth_first_search(self):
    #     self.breadth_first_search_recursion(tree.clients[0], [tree.clients[0]])

    # def breadth_first_search_recursion(self, vertex, visited):
    #     leaf_flag = True
    #     for node in tree.graph_adj_list[vertex]:            
    #         if not node in visited:
    #             print ("Leaf",vertex, node)

    #             visited.append(node)
               
    #             self.breadth_first_search_recursion(node, visited)
    #             leaf_flag = False
               
    #             visited.remove(node)
        
    #     #if leaf_flag:
    #     #    print ("Leaf", visited)

    def findPath(self, first_node, nodes_to_find):
        path = []
        self.findPathRecursion(first_node, [first_node], path, nodes_to_find)
        return path

    def findPathRecursion(self, vertex, visited, path, nodes_to_find):

        for node in self.tree.graph_adj_list[vertex]:            
            if not node in visited:
                path.append((vertex, node))

                if(node in nodes_to_find):
                    return True

                visited.append(node)
               
                if(self.findPathRecursion(node, visited, path, nodes_to_find)):
                    return True
               
                visited.remove(node)
                path.remove((vertex, node))

        return False




    def breadthFirstSearch(self):
        path = []
        self.breadthFirstSearchRecursion(self.tree.clients[0], [self.tree.clients[0]], path)
        return path

    def breadthFirstSearchRecursion(self, vertex, visited, path):
        leaf_flag = True
        for node in self.tree.graph_adj_list[vertex]:            
            if not node in visited:
                path.append((vertex, node))
                #print ("Leaf",vertex, node)
                visited.append(node)
               
                self.breadthFirstSearchRecursion(node, visited, path)
                leaf_flag = False
               
                visited.remove(node)

    def getClosestPoints(self, vector, point, ray):
        distances = np.linalg.norm(vector-point,2,1)
        #print(point)
        return distances < ray



    def doAllocation(self, ray):
        path = self.breadthFirstSearch()

        resulting_allocation = np.matrix([0, 0])

        connection = path[0]
        intesection_points = np.matrix([self.tree.graph_vertex_position[connection[0]]])
        
        i = 0
        for connection in path:
            
            #get the closest point from the allocated position to the new allocation (within a distance ray)
            index_ = self.getClosestPoints(intesection_points, self.tree.graph_vertex_position[connection[0]], ray)            
            index = [i for i, x in enumerate(index_) if x] ## get the possible points for connection
            
            min_dist = np.Inf
            allocation_point = []
            #For all possible points for the allocation, get the one furthest from the current allocation
            for i in index:

                closest_point = (intesection_points[i,0], intesection_points[i,1])
                p, dist = self.getFirstPoint(closest_point, self.tree.graph_vertex_position[connection[0]], self.tree.graph_vertex_position[connection[1]], ray)
                if(dist < min_dist):
                    min_dist = dist
                    allocation_point = p

            #add point for the first allocation
            #if(len(resulting_allocation) == 0):
            #    all_point = np.matrix([allocation_point[0], allocation_point[1]])
            #    resulting_allocation = np.append(resulting_allocation, all_point, 0)

            #print(index_, connection[0])
            allocation = self.segmentTreeFromPoint(allocation_point, self.tree.graph_vertex_position[connection[1]], ray)
            intesection_points = np.append(intesection_points, allocation, 0)

            #concatenate all the allocation
            resulting_allocation = np.append(resulting_allocation, allocation, 0)
            #if( i == 1):
            #    break
            #i += 1

        return resulting_allocation[1:]
        pass

    def doBestAllocation(self, ray):
        allocation = self.doAllocation(ray)
        initial_ray = 0
        final_ray = ray
        new_ray = ray

        while(True):
            last_new_ray = new_ray
            new_ray = (initial_ray + final_ray)/2
            new_allocation = self.doAllocation(new_ray)
            if(allocation.shape[0] == new_allocation.shape[0]):
                final_ray = new_ray
                allocation = new_allocation
            else:
                initial_ray = new_ray

            print(new_ray, new_allocation.shape[0], allocation.shape[0])
            if(abs(last_new_ray - new_ray) < 1):
                return new_allocation


    def getDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def getFirstPoint(self, closest_point, p1, p2, ray):


        first_point_dist = math.sqrt((closest_point[0]-p1[0])**2 + (closest_point[1]-p1[1])**2)
        #print(closest_point, first_point_dist, p1, p2)
        if( first_point_dist > ray):
            print("ERRROR - getFirstPoint")
            return

        #define the vector direction and is nomalization
        vec = (p2[0] - p1[0], p2[1] - p1[1])
        vec_size = math.sqrt(vec[0]**2 + vec[1]**2)
        norm_vec = (vec[0]/vec_size, vec[1]/vec_size)

        ##
        x = closest_point[0] - p1[0]
        y = closest_point[1] - p1[1]
        ## direction vector
        v1 = norm_vec[0]
        v2 = norm_vec[1]
        ##

        alphas = np.roots([(v1**2 + v2**2), -(2*x*v1 + 2*y*v2), x**2 + y**2 - ray**2])

        new_point1 = (p1[0] + alphas[0]*v1, p1[1] + alphas[0]*v2)
        new_point2 = (p1[0] + alphas[1]*v1, p1[1] + alphas[1]*v2)

        d1 = self.getDistance(new_point1, p2)
        d2 = self.getDistance(new_point2, p2)
        if(d1 < d2):
            return new_point1, d1
        else:
            return new_point2, d2
       


    def segmentTreeFromPoint(self, p1, p2, ray):

        resulting_points = np.matrix([0, 0])

        #define the vector direction and is nomalization
        vec = (p2[0] - p1[0], p2[1] - p1[1])
        vec_size = math.sqrt(vec[0]**2 + vec[1]**2)
        norm_vec = (vec[0]/vec_size, vec[1]/vec_size)

        get_next_point = True
        alpha = 0
        while(get_next_point):
            new_point = np.matrix([p1[0] + alpha*ray*norm_vec[0], p1[1] + alpha*ray*norm_vec[1]])
            resulting_points = np.append(resulting_points, new_point, 0)
            alpha += 1

            #the last point has its distance less than than ray to p2
            point_dist = math.sqrt((new_point[0,0]-p2[0])**2 + (new_point[0,1]-p2[1])**2)
            if(point_dist < ray):
                get_next_point = False
                continue


        
        return resulting_points[1:]

    ###
    ###
    ###
    def getLeafs(self, visited_set):
        leafs = []
        for vertex in self.tree.graph_adj_list:            
            if(len(self.tree.graph_adj_list[vertex])-visited_set.count(vertex)*2 == 1):
                leafs.append(vertex)
        return (leafs)




    def segmentation_search(self, closed_list, paths):
        
        leafs = self.getLeafs(closed_list)
        #print(leafs)
        seg_flag = False
        for node in leafs:
            self.segmentation_search_recursion(node, [node], closed_list, leafs, paths)
            seg_flag = True
        return seg_flag



    def segmentation_search_recursion(self, vertex, visited, closed_list, leafs, paths):
        leaf_flag = True

        for node in self.tree.graph_adj_list[vertex]:            
            if not node in visited and (not node in closed_list or node in leafs) :

                new_visited = visited[:]
                new_visited.append(node)         
                self.segmentation_search_recursion(node, new_visited, closed_list, leafs, paths)
                leaf_flag = False

        if leaf_flag and len(visited) > 1:

            new_closed_list = closed_list[:]
            new_closed_list.extend(visited)
            
            new_paths = paths[:]
            new_paths.append(visited)
            if(not self.segmentation_search(new_closed_list, new_paths)): # segmentation has ended
                i = len(self.segmentaion_paths)
                self.segmentaion_paths[i] = new_paths



    def get_path_cost_from_segments(self, path):
        total_distance = 0
        for i in range(0,len(path)):
            p1 = self.tree.graph_vertex_position[path[i][0]]
            p2 = self.tree.graph_vertex_position[path[i][1]]
            distance = self.getDistance(p1, p2)
            total_distance += distance
        return total_distance


    def get_path_cost(self, path):
        total_distance = 0
        for i in range(0,len(path)-1):
            p1 = self.tree.graph_vertex_position[path[i]]
            p2 = self.tree.graph_vertex_position[path[i+1]]
            distance = self.getDistance(p1, p2)
            total_distance += distance
        return total_distance

    def verifyAngle(self, path, radius):
        print(path)
        first = self.tree.graph_vertex_position[path[0]]
        last  = self.tree.graph_vertex_position[path[-1]]

        for i in range(1, len(path)):
            p1 = self.tree.graph_vertex_position[path[i]]
            p2 = self.tree.graph_vertex_position[path[i-1]]
            n = self.getDistance(p1, p2)/radius
            for j in range(int(n)):
                x = p1[0] + j*(p2[0] - p1[0])
                y = p1[1] + j*(p2[1] - p1[1])

                ang1 = math.atan2(p1[1] - y, p1[0] -x)
                ang2 = math.atan2(p2[1] - y, p2[0] -x)

                if(ang1 > ang2):
                    if (ang1-ang2) < 3.14/2.0:
                        return True

                if(ang1 < ang2):
                    if (ang2-ang1) < 3.14/2.0:
                        return True
        return False

    # def evaluate_segmentation(self, radius):

    #     segmentation_costs = {}
    #     segmentation_total_cost = {}
    #     min_cost = float('inf')
    #     min_cost_index = -1
    #     for i in self.segmentaion_paths:
    #         segmentation_costs[i] = []
    #         segmentation = self.segmentaion_paths[i]
    #         segmentation_cost = 0
    #         for path in segmentation:
    #             cost = self.get_path_cost(path)
    #             segmentation_costs[i].append(cost)
    #             segmentation_cost += cost
    #         segmentation_total_cost[i] = segmentation_cost
    #         if(segmentation_cost < min_cost):
    #             min_cost = segmentation_cost

    #     #get the one with approximates more the mean distance of the paths
    #     max_diff = float('inf')
    #     max_diff_index = -1
    #     for i in self.segmentaion_paths:
    #         if(segmentation_total_cost[i] != min_cost):
    #             continue
    #         mean_distance = min_cost/len(segmentation_costs[i])

    #         max_from_seg = 0
    #         for cost in segmentation_costs[i]:
    #             max_from_seg = max(max_from_seg, cost)

    #         if(max_diff > abs(mean_distance-max_from_seg)):
    #             max_diff = abs(mean_distance-max_from_seg)
    #             max_diff_index = i


    #     #print(segmentation_costs[max_diff_index])
    #     return (self.segmentaion_paths[max_diff_index])

    def evaluate_segmentation(self, radius):

        segmentation_costs = {}
        segmentation_total_cost = {}
        min_cost = float('inf')
        min_cost_index = -1
        for i in self.segmentaion_paths:
            segmentation_costs[i] = []
            segmentation = self.segmentaion_paths[i]
            segmentation_cost = 0
            for path in segmentation:
                cost = self.get_path_cost(path)
                segmentation_costs[i].append(cost)
                segmentation_cost += cost
            segmentation_total_cost[i] = segmentation_cost
            if(segmentation_cost < min_cost):
                min_cost = segmentation_cost

        #get the one with approximates more the mean distance of the paths
        max_diff = float('inf')
        max_diff_index = -1
        for i in self.segmentaion_paths:
            not_selected = False


            if(int(segmentation_total_cost[i]) != int(self.size)):
                continue

            #verify angle
            #for path in (self.segmentaion_paths[i]):
            #    if(self.verifyAngle(path, radius)):
            #        not_selected = True
            #        break

            if(not_selected):
                continue   

            if(max_diff_index == -1):
                max_diff_index = i
            #print(i, segmentation_costs[i])
            diff = float('inf')
            for cost in segmentation_costs[i]:
                if(cost/radius < 1):
                    not_selected = True
                    break
                diff_ = radius - cost/math.ceil(cost/radius)
                diff = min(diff, diff_)

            if(not_selected):
                continue

            # mean_distance = min_cost/len(segmentation_costs[i])

            # max_from_seg = 0
            # for cost in segmentation_costs[i]:
            #     max_from_seg = max(max_from_seg, cost)

            if(diff < max_diff):
                print('max diff', diff)
                max_diff = diff
                max_diff_index = i


        #print(self.segmentaion_paths[max_diff_index])
        return (self.segmentaion_paths[max_diff_index])


                
    def intersect_paths(self, path1, path2):
        for node in path1:
            if( node in path2):
                return True
        return False

            
    def comparePathSegmentations(self, path1, path2):
        p1 = []
        p2 = []
        for i in range(len(path1)):
            p1.append(path1[i][:])
            #print(path1[i])
            p1[i].sort()
        for i in range(len(path2)):
            p2.append(path2[i][:])
            p2[i].sort()
        p1.sort()
        p2.sort()
        return p1 == p2
        pass