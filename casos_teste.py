from Grafoteca import Graph, DiGraph

def reconstruct_path(src, dest, pi):
    if dest not in pi or (pi[dest] is None and dest != src):
        return None

    path = []
    v = dest
    visited = set()
    while v is not None:
        if v in visited:
            return None 
        visited.add(v)
        path.append(v)
        v = pi[v]

    path.reverse()
    return path

def load_dimacs_gr(path, directed=False):
    adj = {}
    
    with open(path, "r") as f:
        for line in f:
            if line.startswith("a "):
                parts = line.split()
                u, v, w = parts[1], parts[2], parts[3]
                
                if u not in adj:
                    adj[u] = {}
                if v not in adj:
                    adj[v] = {}
                
                adj[u][v] = int(w)
                
                if not directed:
                    adj[v][u] = int(w)
    
    return adj

def main():
    print("Selecione o tipo de Grafo:")
    print("1 - Graph (Não-direcionado)")
    print("2 - DiGraph (Direcionado)")
    choice = input("Opção: ").strip()

    is_directed = (choice == '2')
    
    # Ajuste o caminho do arquivo conforme necessário
    path_file = r"USA-road-d.NY.gr"
    
    print(f"Carregando grafo {'DIRECIONADO' if is_directed else 'NÃO-DIRECIONADO'}...")
    adj = load_dimacs_gr(path_file, directed=is_directed)

    if is_directed:
        g = DiGraph(adj)
    else:
        g = Graph(adj)

    # (a) Mind (Vértice de menor grau)
    print("\n=== (a) Vértice de menor grau (mind) ===")
    mind_result = g.mind()
    
    if is_directed:
        v_min, deg_min, in_d, out_d = mind_result
        print(f"Vértice: {v_min}")
        print(f"Grau Total: {deg_min} (Entrada: {in_d}, Saída: {out_d})")
    else:
        v_min, deg_min = mind_result
        print(f"Vértice: {v_min}")
        print(f"Grau: {deg_min}")
    
    # (b) Mind (Vértice de menor grau) 
    print("\n=== (b) Vértice de maior grau (maxd) ===")
    mind_result = g.maxd()
    
    if is_directed:
        v_max, deg_max, in_d, out_d = mind_result
        print(f"Vértice: {v_max}")
        print(f"Grau Total: {deg_max} (2Entrada: {in_d}, Saída: {out_d})")
    else:
        v_min, deg_min = mind_result
        print(f"Vértice: {v_min}")
        print(f"Grau: {deg_min}")

    # (c) Caminho BFS >= 10 
    print("\n=== (c) Caminho BFS com >= 10 arestas ===")
    src = "1" 
    if g.v(src) == "Vértice não encontrado":
         print(f"Erro: Vértice {src} não existe.")
    else:
        dist, pi = g.bfs(src)
        dest = None
        for v in dist:
            if dist[v] != float("inf") and dist[v] >= 10:
                dest = v
                break

        if dest is None:
            print("Nenhum vértice a 10 arestas de distância.")
        else:
            path = reconstruct_path(src, dest, pi)
            print("Destino:", dest)
            print("Distância (arestas):", dist[dest])
            # Exibe resumo se for muito grande
            if path and len(path) > 15:
                print(f"Caminho: {path[:3]} ... {path[-3:]} (Total: {len(path)})")
            else:
                print("Caminho:", path)
    
    # (d) Ciclo Mínimo
    
    print("\n=== (d) Ciclo com pelo menos 5 arestas ===")
    
    if hasattr(g, 'encontrar_ciclo_minimo'):
        ciclo = g.encontrar_ciclo_minimo(minimo=5) 
        if ciclo is None:
            print("Nenhum ciclo encontrado com 5 ou mais arestas.")
        else:
            print("Ciclo encontrado (fechado):", ciclo)
            print("Número de arestas:", len(ciclo) - 1)
    else:
        print("AVISO: O método 'encontrar_ciclo_minimo' não foi encontrado na classe.")

    # (e) Vértice mais distante de 129 (Bellman-Ford)
    
    print("\n=== (e) Vértice mais distante de 129 (peso) ===")
    src = "129"
    if g.v(src) == "Vértice não encontrado":
        print(f"Erro: Vértice {src} não existe.")
    else:
        dist, pi, neg = g.bf(src)
        farthest = None
        max_dist = -float('inf')

        for v in dist:
            if dist[v] != float("inf") and dist[v] > max_dist:
                max_dist = dist[v]
                farthest = v

        if neg:
            print("ALERTA: Ciclo negativo detectado!")

        print("Vértice mais distante:", farthest)
        print("Custo total:", max_dist)
    
        # (f) Coloração do Grafo (DSatur)

        print("\n=== (f) Coloração do grafo (DSATUR) ===")
        coloring, num_colors = g.coloracao_propria()
        print("Número de cores usadas:", num_colors)
        print("Exemplo de cores (primeiros 5 nós):", list(coloring.items())[:5])

if __name__ == "__main__":
    main()