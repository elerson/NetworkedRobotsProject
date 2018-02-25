#!/usr/bin/python
import math
import numpy as np

class Tree:
    def __init__(self, filename):
        self.filename = filename
        self.graph_adj_list = {}
        self.graph_vertex_position = {}
        self.clients = []
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
                        self.graph_vertex_position[i] = (int(line_data[2*i]), int(line_data[2*i + 1]))

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


    def print_(self):
        print(self.graph_adj_list)
        print(self.graph_vertex_position)
        print(self.clients)


class TreeSegmention:
    def __init__(self, tree):
        self.tree = tree
        self.segmentaion_paths = {}

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



    def breadthFirstSearch(self):
        path = []
        self.breadthFirstSearchRecursion(tree.clients[0], [tree.clients[0]], path)
        return path

    def breadthFirstSearchRecursion(self, vertex, visited, path):
        leaf_flag = True
        for node in tree.graph_adj_list[vertex]:            
            if not node in visited:
                path.append((vertex, node))
                #print ("Leaf",vertex, node)
                visited.append(node)
               
                self.breadthFirstSearchRecursion(node, visited, path)
                leaf_flag = False
               
                visited.remove(node)

    def getClosestPoints(self, vector, point, ray):
        distances = np.linalg.norm(vector-point,2,1)
        return distances < ray



    def doAllocation(self, ray):
        path = self.breadthFirstSearch()


        connection = path[0]
        intesection_points = np.matrix([tree.graph_vertex_position[connection[0]], tree.graph_vertex_position[connection[0]]])
        for connection in path:
            

            index_ = self.getClosestPoints(intesection_points, connection[0], ray)
            index = [i for i, x in enumerate(index_) if x] ## get the possible points for connection
            
            min_dist
            for i in index:
                p, dist = getFirstPoint(intesection_points[i], tree.graph_vertex_position[connection[0]], tree.graph_vertex_position[connection[1]], ray)
                


            allocation = self.segmentTreeFromPoint(p1, tree.graph_vertex_position[connection[1]], ray)


        pass

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
        alpha = 1
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
        for vertex in tree.graph_adj_list:            
            if(len(tree.graph_adj_list[vertex])-visited_set.count(vertex)*2 == 1):
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

        for node in tree.graph_adj_list[vertex]:            
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
                #print(new_paths)
                i = len(self.segmentaion_paths)
                if(i == 0):
                    self.segmentaion_paths[0] = new_paths
                else:
                    if(not self.comparePathSegmentations(self.segmentaion_paths[i-1], new_paths)):
                        self.segmentaion_paths[i] = new_paths


                
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
            print(path1[i])
            p1[i].sort()
        for i in range(len(path2)):
            p2.append(path2[i][:])
            p2[i].sort()
        p1.sort()
        p2.sort()
        return p1 == p2
        pass

        



if __name__ == "__main__":
    tree = Tree("/home/elerson/catkin_ws/all/networked_robots_experimetal_data/1euclideanexperiment/steinerData1.dat")
    tree.readTree()
    #tree.print_()
    segmentation = TreeSegmention(tree)
    ray = 10
    segmentation.doAllocation(ray)
    #path = segmentation.breadthFirstSearch()
    #print(path)
    #ray = 10
    #mat = segmentation.segmentTreeFromPoint(tree.graph_vertex_position[4],tree.graph_vertex_position[3], ray)
    #next_p = segmentation.getFirstPoint((479.70989495, 272.96907802), tree.graph_vertex_position[3],tree.graph_vertex_position[1], ray)
    #print(next_p)
    #print(tree.graph_vertex_position[4], tree.graph_vertex_position[3])
    #a = [[3, 4], [6, 0, 1], [7, 2, 1, 3, 5]]
    #b = [[4, 3], [6, 0, 1], [7, 2, 1, 3, 5]]
    #print (segmentation.segmentaion_paths)