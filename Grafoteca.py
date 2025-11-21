global time
time=0

global predecessors
predecessors={}*None

global visited
visited={}*False

global init_time, finish_time
init_time=[]
finish_time=[]

class Graph:
    def __init__(self,adj_list):
        self.adj = adj_list
    
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
        init_time[v]=time + 1
        visited[v]=True
        predecessors[v]=parent
        
        for u in self.adj[v]:
            if not visited[u]:
                self.dfs(u,v)
        finish_time[v]= time + 1
    
        return {
        init_time,
        finish_time,
        predecessors
    }        
        
class DiGraph:
    def __init__(self,adj_list):
        self.adj = adj_list