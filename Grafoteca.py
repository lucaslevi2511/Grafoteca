import heapq
from collections import deque
class Graph:
    def __init__(self,adj_list):
        self.adj = {v: dict(adj_list[v]) for v in adj_list}
        self.time = 0
        self.visited = {v: False for v in self.adj}
        self.predecessors = {v: None for v in self.adj}
        self.init_time = {v: None for v in self.adj}
        self.finish_time = {v: None for v in self.adj}
        self.dist = {v: float('inf') for v in self.adj}
    
    def n(self):
        vertex=set()
        for a in self.adj:
            vertex.add(a)
        return len(vertex)
    
    #Agora ele tá contando o número de vizinhos, antes ele tava contando as strings 'neighbors'
    def m(self):
     edges= sum(len(self.adj[v]) for v in self.adj) // 2
     return edges

    
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
        min_degree = float('inf')
        for v in self.adj:
            degree = len(self.adj[v])
            if degree < min_degree:
                min_degree = degree
                vertex_min_weight = v
        
        return vertex_min_weight, min_degree       
    
    def maxd(self):
        vertex_max_weight = None
        max_degree = -1
        for v in self.adj:
            degree = len(self.adj[v])
            if degree > max_degree:
                max_degree = degree
                vertex_max_weight = v
        return vertex_max_weight,max_degree  
    

    def bfs(self,s):
        
        dist= self.dist = {v: float('inf') for v in self.adj}

        pi = self.predecessors
        
        queue = deque ()
        dist [s] = 0

        queue.append(s)

        while queue: 
            u = queue.popleft()

            for v in self.adj[u]:
                if dist[v] == float('inf'):
                    dist[v] = dist [u] + 1
                    pi [v] = u
                    queue.append(v)

        return dist, pi         

    def dfs(self, start=None):
        self.time = 0
        self.visited = {vertex: False for vertex in self.adj}
        self.predecessors = {vertex: None for vertex in self.adj}
        self.init_time = {vertex: None for vertex in self.adj}
        self.finish_time = {vertex: None for vertex in self.adj}

        if start is None:
            for vertex in self.adj:         
                if not self.visited[vertex]:
                    self._dfs_visit(vertex, None)
        else:
            if not self.visited[start]:
                self._dfs_visit(start, None)

        return {
            "init_time": self.init_time,
            "finish_time": self.finish_time,
            "predecessors": self.predecessors
        }


    def _dfs_visit(self, vertex, parent):
        self.time += 1
        self.init_time[vertex] = self.time
        self.visited[vertex] = True
        self.predecessors[vertex] = parent

        for u in self.adj[vertex]:
            if not self.visited[u]:
                    self._dfs_visit(u, vertex)

        self.time += 1
        self.finish_time[vertex] = self.time
 
             

    def bf(self, src):
        dist= self.dist = {v: float('inf') for v in self.adj}
        pi = self.predecessors
        dist[src] = 0

        edges = []
        for u in self.adj: 
            for v,w in self.adj[u].items():
                edges.append((u,v,w))
                
        V = len(self.adj)

        for _ in range(V-1):
            updated = False
            for u,v,w in edges:
                if dist[u] != float('inf') and dist [u] + w < dist [v]:
                    dist [v] = dist[u] + w
                    pi[v] = u
                    updated = True
            if not updated:
                break        
        
        negative_cycle = False

        for u, v, w in edges:
            if dist[u] != float('inf') and dist[u] + w < dist[v]:
                negative_cycle = True
                break
        return dist, pi,  negative_cycle  

    def dijkstra(self,src):
        dist= self.dist = {v: float('inf') for v in self.adj}
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
    
    def coloracao_propria(self):
        coloring = {v: None for v in self.adj}
        saturation = {v: 0 for v in self.adj}
        degree = {v: len(self.adj[v]) for v in self.adj}
        uncolored = set(self.adj.keys())

        def update_saturation(v):
            colors_used = {coloring[u] for u in self.adj[v] if coloring[u] is not None}
            saturation[v] = len(colors_used)

        while uncolored:
            v_selected = max(uncolored,key=lambda v: (saturation[v], degree[v], str(v)))

            used_colors = {coloring[u] for u in self.adj[v_selected]
                           if coloring[u] is not None}

            color = 0
            while color in used_colors:
                color += 1

            coloring[v_selected] = color
            uncolored.remove(v_selected)

            for u in self.adj[v_selected]:
                if u in uncolored:
                    update_saturation(u)

        num_colors = len({c for c in coloring.values() if c is not None})
        return coloring, num_colors

class DiGraph:
    def __init__(self, adj_list):
        self.adj = {v: dict(adj_list[v]) for v in adj_list}
        self.time = 0
        self.visited = {v: False for v in self.adj}
        self.predecessors = {v: None for v in self.adj}
        self.init_time = {v: None for v in self.adj}
        self.finish_time = {v: None for v in self.adj}
        self.dist = {v: float('inf') for v in self.adj}
        
    def n(self):
        vertex = set()
        for a in self.adj:
            vertex.add(a)
        return len(vertex)
    
    def m(self):
        edges = 0
        seen_edges = set()  
        for u in self.adj:
            for v in self.adj[u]:
                edge = (u, v)
                if edge not in seen_edges:
                    seen_edges.add(edge)
                    edges += 1
        return edges
    
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
        out_degree = {v: len(self.adj[v]) for v in self.adj}
        
        
        in_degree = {v: 0 for v in self.adj}
        
        
        for u in self.adj:
            for v in self.adj[u]:
                in_degree[v] += 1
        
        
        total_degree = {v: out_degree[v] + in_degree[v] for v in self.adj}
        
        min_v = min(total_degree, key=lambda x: total_degree[x])
        min_degree = total_degree[min_v]
        
        return min_v, min_degree,in_degree[min_v],out_degree[min_v]

    
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
        return vertex, max_degree, in_degree[vertex], out_degree[vertex]
    #Erro corrigido na função
    def bfs (self,s):
        dist = {v: float('inf') for v in self.adj} 

        pi = {v:None for v in self.adj}

        queue = deque()
        dist [s] = 0
        queue.append(s)

        while queue:
            u = queue.popleft()
            for v in self.adj[u]:
                if dist[v] == float('inf'):
                    dist[v] = dist[u] + 1
                    pi[v] = u 
                    queue.append(v)
        
        return dist, pi

    def dfs(self, start=None):
        self.time = 0
        self.visited = {vertex: False for vertex in self.adj}
        self.predecessors = {vertex: None for vertex in self.adj}
        self.init_time = {vertex: None for vertex in self.adj}
        self.finish_time = {vertex: None for vertex in self.adj}

        if start is None:
            for vertex in self.adj:          
                if not self.visited[vertex]:
                    self._dfs_visit(vertex, None)
        else:
            if not self.visited[start]:
                self._dfs_visit(start, None)

        return {
            "init_time": self.init_time,
            "finish_time": self.finish_time,
            "predecessors": self.predecessors
        }


    def _dfs_visit(self, vertex, parent):
        self.time += 1
        self.init_time[vertex] = self.time
        self.visited[vertex] = True
        self.predecessors[vertex] = parent

        for u in self.adj[vertex]:
            if not self.visited[u]:
                    self._dfs_visit(u, vertex)

        self.time += 1
        self.finish_time[vertex] = self.time
       

    def bf(self, src):
        dist = {v: float('inf') for v in self.adj}
        pi = {v: None for v in self.adj}  # inicializa predecessores
        dist[src] = 0

        edges = []
        for u in self.adj:
            for v, w in self.adj[u].items():
                edges.append((u, v, w))

        V = len(self.adj)
        for i in range(V - 1):
            updated = False
            for u, v, w in edges:
                if dist[u] != float('inf') and dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    pi[v] = u
                    updated = True
            if not updated:
                break

        negative_cycle = False
        for u, v, w in edges:
            if dist[u] != float('inf') and dist[u] + w < dist[v]:
                negative_cycle = True
                break

        return dist, pi, negative_cycle
    
    
    def dijkstra(self, src):
        dist= self.dist = {v: float('inf') for v in self.adj}
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
    
    def coloracao_propria(self):
        coloring = {v: None for v in self.adj}
        saturation = {v: 0 for v in self.adj}
        indegree = {v: 0 for v in self.adj}
        for u in self.adj:
            for v in self.adj[u]:
                indegree[v] += 1

        degree = {v: indegree[v] + len(self.adj[v]) for v in self.adj}

        uncolored = set(self.adj.keys())

        def update_saturation(v):
            colors_used = set()

            for u in self.adj[v]:
                if coloring[u] is not None:
                    colors_used.add(coloring[u])

            for u in indegree:
                if v in self.adj[u] and coloring[u] is not None:
                    colors_used.add(coloring[u])

            saturation[v] = len(colors_used)

        while uncolored:
            v_selected = max(uncolored,key=lambda v: (saturation[v], degree[v], str(v)))

            neighbors_out = {u for u in self.adj[v_selected]}
            neighbors_in = {u for u in self.adj if v_selected in self.adj[u]}
            neighbors = neighbors_out | neighbors_in

            used_colors = {coloring[u] for u in neighbors if coloring[u] is not None}

            color = 0
            while color in used_colors:
                color += 1

            coloring[v_selected] = color
            uncolored.remove(v_selected)

            for u in neighbors:
                if u in uncolored:
                    update_saturation(u)

        num_colors = len({c for c in coloring.values() if c is not None})
        return coloring, num_colors

