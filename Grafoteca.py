import heapq
from collections import deque
class Graph:
    def __init__(self,adj_list):
        vertices = set(adj_list.keys())
        self.adj = {v: adj_list.get(v, {}) for v in vertices}
        self.time = 0
        self.visited = {v: False for v in self.adj}
        self.predecessors = {v: None for v in self.adj}
        self.init_time = {v: None for v in self.adj}
        self.finish_time = {v: None for v in self.adj}
    
    def n(self):
        vertex=set()
        for a in self.adj:
            vertex.add(a)
        print("O número de vértices é:",len(vertex))
    
    def m(self):
        edges=0
        for a in self.adj:
            edges+=len(self.adj[a])
        print("O numero de arestas é:",edges)
    
    def v(self,v):
        if v in self.adj:
            return self.adj[v]
        else:
            return "Vértice não encontrado"
    
    def w(self,u,v):
        if u in self.adj and v in self.adj[u]:
            return self.adj[u][v]
        else:
            return "Aresta não encontrada"
    
    def mind(self):
        vertex_min_weight = float ('inf')
        min_degree = None
        for v in self.adj:
            degree = len(self.adj[v])
            if degree < min_degree:
                min_degree = degree
                vertex_min_weight = v
        print("O vértice de menor grau é: ",vertex_min_weight," com grau: .",min_degree)       
    
    def maxd(self):
        vertex_max_weight = None
        max_degree = -1
        for v in self.adj:
            degree = len(self.adj[v])
            if degree > max_degree:
                max_degree = degree
                vertex_max_weight = v
        print("O vértice de menor grau é: ",vertex_max_weight," com grau: .",max_degree)  
    

    def bfs(self,s):
        dist = {v: float('inf') for v in self.adj}

        pi = {v: None for v in self.adj}
        
        queue = deque ()
        dist [s] = 0

        queue.append(s)

        while queue: 
            u = queue.popleft()

            for v in self.adj[u]:
                if dist[v] == 'inf':
                    dist[v] = dist [u] + 1
                    pi [v] = u
                    queue.append(v)

            return dist, pi         

    def dfs(self,v,parent=None):
        self.time = 0
        self.visited = {vertex: False for vertex in self.adj}
        self.predecessors = {vertex: None for vertex in self.adj}
        self.init_time = {vertex: None for vertex in self.adj}
        self.finish_time = {vertex: None for vertex in self.adj}
        
        self.time += 1
        self.init_time[v]=self.time
        self.visited[v]=True
        self.predecessors[v]=parent
        
        for u in self.adj[v]:
            if not self.visited[u]:
                self.dfs(u,v)
        self.time += 1
        self.finish_time[v]= self.time 
    
        return {
           "init_time":self.init_time,
           "finish_time":self.finish_time,
           "predecessors":self.predecessors
    }        
        
    def dijkstra(self,src):
        initial_distance= float('inf')
        dist={v: initial_distance for v in self.adj}
        dist[src]=0
        pq=[(0,src)]
        while pq:
            d ,u = heapq.heappop(pq)
            if d>dist[u]:
                continue
            else:
                for v in self.adj[u]:
                    w= self.adj[u][v]
                    if dist[u]+w<dist[v]:
                        dist[v]=dist[u]+w
                        heapq.heappush(pq,(dist[v],v))
        return dist

class DiGraph:
    def __init__(self, adj_list):
        vertex = set(adj_list.keys())
        self.adj = {v: adj_list.get(v, {}) for v in vertex}
        self.time = 0
        self.visited = {v: False for v in self.adj}
        self.predecessors = {v: None for v in self.adj}
        self.init_time = {v: None for v in self.adj}
        self.finish_time = {v: None for v in self.adj}
    
    def n(self):
        vertex = set()
        for a in self.adj:
            vertex.add(a)
        print("O número de vértices é:", len(vertex))
    
    def m(self):
        edges = 0
        seen_edges = set()  
        for u in self.adj:
            for v in self.adj[u]:
                edge = (u, v)
                if edge not in seen_edges:
                    seen_edges.add(edge)
                    edges += 1
        print("O número de arcos (arestas direcionadas) é:", edges)
    
    def v(self, v):
        if v in self.adj:
            return self.adj[v]
        else:
            return "Vértice não encontrado"

    def w(self, u, v):
        if u in self.adj and v in self.adj[u]:
            return self.adj[u][v]
        else:
            return "Arco não encontrado"
    
    def mind(self):
        out_degree = {v: len(self.adj[v] for v in self.ad)}
        in_degree = {v: 0 for v in self.adj}

        for u in self.adj:
            for v in self.adj[u]:
                in_degree[v] += 1
        
        total_degree = {v: out_degree[v] + in_degree for v in self.adj}
        min_v = min(total_degree, key=lambda x: total_degree[x]) 
        min_degree = total_degree[min_degree]

        print(f"O vértice de menor grau total é: {min_v}, com grau {min_degree} (in-degree: {in_degree[min_v]}, out-degree: {out_degree[min_v]})")
    
    def maxd(self):

        out_degree = {v: len(self.adj.get(v, {})) for v in self.adj}

        in_degree = {v: 0 for v in self.adj}
        for u in self.adj:
            for v in self.adj[u]:
                in_degree[v] += 1
        
        total_degree = {v: out_degree[v] + in_degree[v] for v in self.adj}
        max_degree = -1
        vertex = None
        for v in total_degree:
            if total_degree[v] > max_degree:
                max_degree = total_degree[v]
                vertex = v
        print(f"O vértice de maior grau total é: {vertex}, com grau {max_degree} (in-degree: {in_degree[vertex]}, out-degree: {out_degree[vertex]})")
    
    def bfs (self,s):
        dist = {v: float ('inf' for v in self.adj)}

        pi = {v: None for v in self.adj}

        queue = deque()
        dist [s] = 0
        queue.append(s)

        while queue:
            u = queue.popleft()
            for v in self.adj[u]:
                if dist == 'inf':
                    dist[v] = dist[u] + 1
                    pi[v] = u 
                    queue.append(v)
        
        return dist, pi

    def dfs(self, v, parent=None):
        self.time = 0
        self.visited = {vertex: False for vertex in self.adj}
        self.predecessors = {vertex: None for vertex in self.adj}
        self.init_time = {vertex: None for vertex in self.adj}
        self.finish_time = {vertex: None for vertex in self.adj}
        
        self.time += 1
        self.init_time[v] = self.time
        self.visited[v] = True
        self.predecessors[v] = parent
        
        for u in self.adj[v]:
            if not self.visited[u]:
                self.dfs(u, v)
        self.time += 1
        self.finish_time[v] = self.time 
        
        return {
            "init_time": self.init_time,
            "finish_time": self.finish_time,
            "predecessors": self.predecessors
        }
        
    def dijkstra(self, src):
        initial_distance = float('inf')
        dist = {v: initial_distance for v in self.adj}
        dist[src] = 0
        pq = [(0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            else:
                for v in self.adj[u]:
                    w = self.adj[u][v]
                    if dist[u] + w < dist[v]:
                        dist[v] = dist[u] + w
                        heapq.heappush(pq, (dist[v], v))
        return dist
