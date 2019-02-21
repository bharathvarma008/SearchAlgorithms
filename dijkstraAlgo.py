# Let's do some Python Coding.. 

# Load dependencies first
import sys #Library for INT_MAX

class Graph():
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)] for row in range(vertices)]
    
    def printSolution(self, dist):
        print("Vertex tDistance from Source")
        for node in range(self.V):
            print(node, "t", dist[node])
  
  # A utility function to find the vertex with minimum distance value, from the set of 
  # vertices not yet included in the shortest path tree (SPT)

    def minDistance(self, dist, sptSet):
        # Initialize minimum distance for next vertex
        min_ = sys.maxsize

        # Search non nearest vertex which is not in the Shortest Path Tree
        for v in range(self.V):
            if dist[v] < min_ and sptSet[v] == False:
                min_ = dist[v]
                min_index = v

        return min_index
  
  # Function that implements Dijkstra's single shortest path algorithm for a graph 
  # represented using adjacency matrix representation
  
    def dijkstra(self, src):
    
        dist = [sys.maxsize] * self.V
        dist[src] = 0
        sptSet = [False] * self.V

        for cout in range(self.V):

            # Pick the min distance vertex from the set of vertices not yet processed.
            # u is always equal to src in the first iteration
            u =  self.minDistance(dist, sptSet)

            # Put the minimum distance vertex in the SPT
            sptSet[u] = True

            # Udpate the dist value of the adjacent vertices of the picked vertex only if 
            # the curent distance is greater than the new distance and the vertex is not in 
            # the SPT
            for v in range(self.V):
                if self.graph[u][v] > 0 and sptSet[v] == False and dist[v] > dist[u] + self.graph[u][v]:
                    dist[v] = dist[u] + self.graph[u][v]

        self.printSolution(dist)
        
# Driver program 
g = Graph(9)
g.graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0], 
           [4, 0, 8, 0, 0, 0, 0, 11, 0], 
           [0, 8, 0, 7, 0, 4, 0, 0, 2], 
           [0, 0, 7, 0, 9, 14, 0, 0, 0], 
           [0, 0, 0, 9, 0, 10, 0, 0, 0], 
           [0, 0, 4, 14, 10, 0, 2, 0, 0], 
           [0, 0, 0, 0, 0, 2, 0, 1, 6], 
           [8, 11, 0, 0, 0, 0, 1, 0, 7], 
           [0, 0, 2, 0, 0, 0, 6, 7, 0]]

g.dijkstra(0)

#--------------------------------
## Output you'll get
# Vertex tDistance from Source
# 0 t 0
# 1 t 4
# 2 t 12
# 3 t 19
# 4 t 21
# 5 t 11
# 6 t 9
# 7 t 8
# 8 t 14

# ===============
# Takeover points
# 1) The code calculates shortest distance, but doesn’t calculate the path information. 
#     We can create a parent array, update the parent array when distance is updated 
#     (like prim’s implementation) and use it show the shortest path from source to 
#     different vertices.
# 2) The code is for undirected graph, same dijkstra function works for directed graphs too.
# 3) The code finds shortest distances from source to all vertices. If we are interested 
#    only in shortest distance from the source to a single target, we can break the for 
#    the loop when the picked minimum distance vertex is equal to target.
# 4) Time Complexity of the implementation is O(V^2). If the input graph is represented 
#    using adjacency list, it can be reduced to O(E log V) with the help of binary heap. 
#    Please see Dijkstra’s Algorithm for Adjacency List Representation for more details.
# 5) Dijkstra’s algorithm doesn’t work for graphs with negative weight edges. 
#    For graphs with negative weight edges, Bellman–Ford algorithm can be used, 
#    we will soon be discussing it as a separate post.
# ===============
