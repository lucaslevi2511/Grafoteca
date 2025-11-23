import heapq

class Graph:
    def __init__(self,adj_list):
        vertices = set(adj_list.keys())
        self.adj = {v: adj_list.get(v, {}) for v in vertices}
        self.time = 0
        self.visited = {v: False for v in self.adj}
        self.predecessors = {v: None for v in self.adj}
        self.init_time = {v: None for v in self.adj}
        self.finish_time = {v: None for v in self.adj}
    
    def count_vertexs(self):
        vertex=set()
        for a in self.adj:
            vertex.add(a)
        print("O número de vértices é:",len(vertex))
    
    def count_edges(self):
        edges=0
        for a in self.adj:
            edges+=len(self.adj[a])
        print("O numero de arestas é:",edges)
    
    def see_neighbors(self,v):
        if v in self.adj:
            return self.adj[v]
        else:
            return "Vértice não encontrado"
    
    def see_edge_weight(self,u,v):
        if u in self.adj and v in self.adj[u]:
            return self.adj[u][v]
        else:
            return "Aresta não encontrada"
    
    def see_biggest(self):
        edges=-1
        vertex=None
        for a in self.adj:
            if(len(self.adj[a])>edges):
               edges=len(self.adj[a])
               vertex=a 
        print("O vértice de maior grau é:",vertex,"com grau",edges)  
    
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
    
    def count_vertexs(self):
        vertex = set()
        for a in self.adj:
            vertex.add(a)
        print("O número de vértices é:", len(vertex))
    
    def count_edges(self):
        edges = 0
        seen_edges = set()  
        for u in self.adj:
            for v in self.adj[u]:
                edge = (u, v)
                if edge not in seen_edges:
                    seen_edges.add(edge)
                    edges += 1
        print("O número de arcos (arestas direcionadas) é:", edges)
    
    def see_neighbors(self, v):
        if v in self.adj:
            return self.adj[v]
        else:
            return "Vértice não encontrado"
    
    def see_edge_weight(self, u, v):
        if u in self.adj and v in self.adj[u]:
            return self.adj[u][v]
        else:
            return "Arco não encontrado"
    
    def see_biggest(self):

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
