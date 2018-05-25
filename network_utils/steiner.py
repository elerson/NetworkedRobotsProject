from scipy import misc
from UnionFind import UnionFind
from Graphs import isUndirected
from math import floor, sqrt, sin, cos, pi
import numpy as np


class Steiner:
    def __init__(self, graph, terminals):
        self.graph            = graph
        self.createFullConnectedGraph(terminals)
        self.steiner = self.minimumSpanningTree(self.full_graph)
        self.getSteinerPoints(terminals)


    def minimumSpanningTree(self, G):
        """
        Return the minimum spanning tree of an undirected graph G.
        G should be represented in such a way that iter(G) lists its
        vertices, iter(G[u]) lists the neighbors of u, G[u][v] gives the
        length of edge u,v, and G[u][v] should always equal G[v][u].
        The tree is returned as a list of edges.
        """
        if not isUndirected(G):
            raise ValueError("MinimumSpanningTree: input is not undirected")
        for u in G:
            for v in G[u]:
                if G[u][v] != G[v][u]:
                    raise ValueError("MinimumSpanningTree: asymmetric weights")

        # Kruskal's algorithm: sort edges by weight, and add them one at a time.
        # We use Kruskal's algorithm, first because it is very simple to
        # implement once UnionFind exists, and second, because the only slow
        # part (the sort) is sped up by being built in to Python.
        subtrees = UnionFind()
        tree = []
        for W,u,v in sorted((G[u][v],u,v) for u in G for v in G[u]):
            if subtrees[u] != subtrees[v]:
                tree.append((u,v))
                subtrees.union(u,v)
        return tree

    def dijkstra(self, graph, src, dest, result = {}, visited=[],distances={},predecessors={}):
        """ calculates a shortest path tree routed in src
        """    
        # a few sanity checks
        if src not in graph:
            raise TypeError('The root of the shortest path tree cannot be found')
        if dest not in graph:
            raise TypeError('The target of the shortest path cannot be found')    
        # ending condition
        if src == dest:
            # We build the shortest path and display it
            path=[]
            pred=dest
            while pred != None:
                path.append(pred)
                pred=predecessors.get(pred,None)

            result['path'] = path
            result['cost'] = distances[dest]

        else :     
            # if it is the initial  run, initializes the cost
            if not visited: 
                distances[src]=0
            # visit the neighbors
            for neighbor in graph[src] :
                if neighbor not in visited:
                    new_distance = distances[src] + graph[src][neighbor]
                    if new_distance < distances.get(neighbor,float('inf')):
                        distances[neighbor] = new_distance
                        predecessors[neighbor] = src
            # mark as visited
            visited.append(src)
            # now that all neighbors have been visited: recurse                         
            # select the non visited node with lowest distance 'x'
            # run Dijskstra with src='x'
            unvisited={}
            for k in graph:
                if k not in visited:
                    unvisited[k] = distances.get(k,float('inf'))        
            x=min(unvisited, key=unvisited.get)
            self.dijkstra(graph,x,dest,result,visited,distances,predecessors)


    def getShortestPath(self, src, dst):
        result = {}
        self.dijkstra(self.graph, src, dst, result, [], {}, {})
        return result['path']

    def createFullConnectedGraph(self, nodes):
        self.full_graph      = {}
        self.paths           = {}
        self.costs           = {}
        for i in nodes:
            graph_ = {}
            for j in nodes:
                if i == j:
                    continue
                result = {}
                self.dijkstra(self.graph, i, j, result, [], {}, {})
                graph_[j] = result['cost']
                self.paths[(i, j)] = result['path'][::-1]
                self.costs[(i, j)] = result['cost']
            self.full_graph[i] = graph_

        return self.full_graph

    def getSteinerPoints(self, terminals):
        self.steiner_vertices = set([])
        for v in self.steiner:
            self.steiner_vertices = self.steiner_vertices.union(set(self.paths[v]))
        self.steiner_vertices =  self.steiner_vertices - set(terminals)
        self.steiner_vertices = list(self.steiner_vertices)

        