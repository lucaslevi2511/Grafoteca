from Grafoteca import Graph

def reconstruct_path(src, dest, pi):
    if dest not in pi or pi[dest] is None and dest != src:
        return None

    path = []
    v = dest
    visited = set()  # evita loops infinitos
    while v is not None:
        if v in visited:
            return None  # loop detectado
        visited.add(v)
        path.append(v)
        v = pi[v]

    path.reverse()
    return path


def load_dimacs_gr(path):
    adj = {}
    
    with open(path, "r") as f:
        for line in f:
            if line.startswith("a "):
                _, u, v, w = line.split()
                if u not in adj:
                    adj[u] = {}
                if v not in adj:
                    adj[v] = {}
                adj[u][v] = int(w)
                adj[v][u] = int(w)  # grafo não-direcionado
    
    return adj


def main():
    # Carregar grafo DIMACS
    path_file=r"C:\Users\cleyb\Downloads\USA-road-d.NY\USA-road-d.NY.gr"
    adj = load_dimacs_gr(path_file)
    g = Graph(adj)

    # (a) mind
    print("\n=== (a) mind ===")
    v_min, deg_min = g.mind()
    print("Vértice de menor grau:", v_min)
    print("Grau:", deg_min)

    # (c) caminho com >= 10 arestas (BFS)
    print("\n=== (c) Caminho com pelo menos 10 arestas ===")
    src = "1"
    dist, pi = g.bfs(src)

    dest = None
    for v in dist:
        if dist[v] >= 10 and dist[v] != float("inf"):
            dest = v
            break

    if dest is None:
        print("Nenhum vértice está a 10 arestas de distância.")
    else:
        path = reconstruct_path(src, dest, pi)
        print("Destino encontrado:", dest)
        print("Nº de arestas:", dist[dest])
        print("Caminho:", path)

    # (e) vértice mais distante do 129 (por peso), usando Bellman-Ford
    print("\n=== (e) Vértice mais distante de 129 (por peso) ===")
    src = "129"
    dist, pi, neg = g.bf(src)

    # Encontrar o vértice mais distante
    farthest = None
    max_dist = -float('inf')

    for v in dist:
        if dist[v] != float("inf") and dist[v] > max_dist:
            max_dist = dist[v]
            farthest = v

    if neg:
        print("Ciclo negativo detectado — distância pode não ser confiável.")

    print("Vértice mais distante:", farthest)
    print("Distância:", max_dist)



if __name__ == "__main__":
    main()
