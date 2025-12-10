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
    
    #Função de contar os vértices
    def n(self):
        vertex=set()
        for a in self.adj:
            vertex.add(a)
        return len(vertex)
    
    #Função de contar as arestas
    def m(self):
     edges= sum(len(self.adj[v]) for v in self.adj) // 2
     return edges

    #Função de retornar a vizinhança    
    def v(self,v):
        if v in self.adj:
            return self.adj[v]
        else:
            return "Vértice não encontrado"
    
    #Função para retornar o grau do vértice
    def d(self,v):
        if v in self.adj[v]:
            return len(self.adj[v])
        else:
            return "Vértice não encontrado"

    #Função de retornar o peso da aresta
    def w(self,u,v):
        if u in self.adj and v in self.adj[u]:
            return self.adj[u][v]
        else:
            return "Aresta não encontrada"
    
    #Menor grau do Grafo
    def mind(self):
        vertex_min_weight = float ('inf')
        min_degree = float('inf')
        for v in self.adj:
            degree = len(self.adj[v])
            if degree < min_degree:
                min_degree = degree
                vertex_min_weight = v
        
        return vertex_min_weight, min_degree       
    
    #Maior grau do Grafo
    def maxd(self):
        vertex_max_weight = None
        max_degree = -1
        for v in self.adj:
            degree = len(self.adj[v])
            if degree > max_degree:
                max_degree = degree
                vertex_max_weight = v
        return vertex_max_weight,max_degree  
    
    #Algoritmo de busca em largura 
    def bfs(self,s):
        
        dist= self.dist = {v: float('inf') for v in self.adj}

        pi = self.predecessors = {v: None for v in self.adj}
        
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

    #Algoritmo da busca em profundidade feita de forma iterativa(evitar erro de recursão)
    def dfs(self):
        self.time = 0
        self.visited = {v: False for v in self.adj}
        self.predecessors = {v: None for v in self.adj}
        self.init_time = {v: None for v in self.adj}
        self.finish_time = {v: None for v in self.adj}

        for start in self.adj:
            if self.visited[start]:
                continue

            stack = [(start, iter(self.adj.get(start, [])))]
            self.visited[start] = True
            self.predecessors[start] = None
            self.time += 1
            self.init_time[start] = self.time

            while stack:
                node, neighbors = stack[-1]
                try:
                    v = next(neighbors)
                    if not self.visited.get(v, False):
                        self.visited[v] = True
                        self.predecessors[v] = node
                        self.time += 1
                        self.init_time[v] = self.time
                        stack.append((v, iter(self.adj.get(v, []))))
                except StopIteration:
                    stack.pop()
                    self.time += 1
                    self.finish_time[node] = self.time
        
        return {
            "init_time": self.init_time,
            "finish_time": self.finish_time,
            "predecessors": self.predecessors
        }

    #Função para extra para responder o item d do trabalho, usa o DFS.
    def encontrar_ciclo_minimo(self, minimo=5):
        self.dfs()

        for u in self.adj:
            for v in self.adj[u]:
                if self.init_time.get(v) is None or self.init_time.get(u) is None:
                    continue

                if self.init_time[v] < self.init_time[u] and self.finish_time[v] > self.finish_time[u]:
                    
                    chain = []
                    curr = u
                    
                    steps = 0
                    max_steps = len(self.adj)
                    
                    while curr is not None and steps < max_steps:
                        chain.append(curr)
                        if curr == v:
                            break
                        curr = self.predecessors.get(curr)
                        steps += 1

                    if chain and chain[-1] == v:
                        chain.reverse() 
                        
                        if len(chain) >= minimo:
                            return chain + [v] 

        return None        

    #Algoritmo de Bellman-Ford
    def bf(self, src):
        dist= self.dist = {v: float('inf') for v in self.adj}
        pi = self.predecessors = {v: None for v in self.adj}
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

    #Algoritmo de dijkastra
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
    
    #Algoritmo DSatur para colorir o grafo
    def coloracao_propria(self):
        reverse_adj = {v: set() for v in self.adj}
        for u in self.adj:
            for v in self.adj[u]:
                if v in reverse_adj:
                    reverse_adj[v].add(u)
        
        neighbor_colors = {v: set() for v in self.adj}
        coloring = {v: None for v in self.adj}
    
        degree = {v: len(self.adj[v]) + len(reverse_adj[v]) for v in self.adj}
        pq = []
        for v in self.adj:
            heapq.heappush(pq, (0, -degree[v], v))

        while pq:
            neg_sat, neg_deg, v_selected = heapq.heappop(pq)
            
            if coloring[v_selected] is not None:
                continue

            neighbors = set(self.adj[v_selected].keys()) | reverse_adj[v_selected]

            used_colors = neighbor_colors[v_selected]
            
            color = 0
            while color in used_colors:
                color += 1

            coloring[v_selected] = color

            for u in neighbors:
                if coloring[u] is None:
                    if color not in neighbor_colors[u]:
                        neighbor_colors[u].add(color)
                        new_sat = len(neighbor_colors[u])
                        
                        heapq.heappush(pq, (-new_sat, -degree[u], u))

        num_colors = len({c for c in coloring.values() if c is not None})
        return coloring, num_colors

# Na classe Digraph foram adotados todas as funções com o mesmo significado da classe Graph.
# A única diferença é que em algumas tivemos adaptações devido ao grafo ser direcionado.

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
    
    def d(self,v):
      if v in self.adj:
        out_degree= len(self.adj[v])
        in_degree=0
        for u in self.adj:
            if v in self.adj[u]:
                in_degree+=1
        return out_degree + in_degree
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
    
    def bfs (self,s):
        dist = {v: float('inf') for v in self.adj} 

        pi = self.predecessors= {v:None for v in self.adj}

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

    def dfs(self):
        self.time = 0
        self.visited = {v: False for v in self.adj}
        self.predecessors = {v: None for v in self.adj}
        self.init_time = {v: None for v in self.adj}
        self.finish_time = {v: None for v in self.adj}

        for start in self.adj:
            if self.visited[start]:
                continue

            stack = [(start, iter(self.adj.get(start, [])))]
            self.visited[start] = True
            self.predecessors[start] = None
            self.time += 1
            self.init_time[start] = self.time

            while stack:
                node, neighbors = stack[-1]
                try:
                    v = next(neighbors)
                    if not self.visited.get(v, False):
                        self.visited[v] = True
                        self.predecessors[v] = node
                        self.time += 1
                        self.init_time[v] = self.time
                        stack.append((v, iter(self.adj.get(v, []))))
                except StopIteration:
                    stack.pop()
                    self.time += 1
                    self.finish_time[node] = self.time
        return {
            "init_time": self.init_time,
            "finish_time": self.finish_time,
            "predecessors": self.predecessors
        }

    def encontrar_ciclo_minimo(self, minimo=5):

        self.dfs()

        for u in self.adj:
            for v in self.adj[u]:
                if self.init_time.get(v) is None or self.init_time.get(u) is None:
                    continue

                if self.init_time[v] < self.init_time[u] and self.finish_time[v] > self.finish_time[u]:
                    chain = []
                    x = u
                    steps = 0
                    max_steps = len(self.adj) + 5  # proteção contra loops estranhos

                    while x is not None and steps <= max_steps:
                        chain.append(x)
                        if x == v:
                            break
                        x = self.predecessors.get(x)
                        steps += 1

                    if chain[-1] != v:
                        continue

                    chain.reverse()  
                    num_arestas = len(chain)
                    if num_arestas >= minimo:
                        ciclo = chain + [chain[0]]  
                        return ciclo

        return None   

    def bf(self, src):
        dist = {v: float('inf') for v in self.adj}
        pi = {v: None for v in self.adj}
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
        
        reverse_adj = {v: set() for v in self.adj}
        for u in self.adj:
            for v in self.adj[u]:
                if v in reverse_adj:
                    reverse_adj[v].add(u)
        
        neighbor_colors = {v: set() for v in self.adj}
        coloring = {v: None for v in self.adj}

        degree = {v: len(self.adj[v]) + len(reverse_adj[v]) for v in self.adj}
 
        pq = []
        for v in self.adj:
            heapq.heappush(pq, (0, -degree[v], v))

        while pq:

            neg_sat, neg_deg, v_selected = heapq.heappop(pq)
            

            if coloring[v_selected] is not None:
                continue

            neighbors = set(self.adj[v_selected].keys()) | reverse_adj[v_selected]

            used_colors = neighbor_colors[v_selected]
            
            color = 0
            while color in used_colors:
                color += 1

            coloring[v_selected] = color

            for u in neighbors:
                if coloring[u] is None:
                    if color not in neighbor_colors[u]:
                        neighbor_colors[u].add(color)
                        new_sat = len(neighbor_colors[u])

                        heapq.heappush(pq, (-new_sat, -degree[u], u))

        num_colors = len({c for c in coloring.values() if c is not None})
        return coloring, num_colors