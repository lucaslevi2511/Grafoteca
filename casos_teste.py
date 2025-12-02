from Grafoteca import Graph, DiGraph  # Assumindo que as classes estão neste arquivo ou módulo

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

# 1. Alteração: Adicionado parâmetro 'directed'
def load_dimacs_gr(path, directed=False):
    adj = {}
    
    with open(path, "r") as f:
        for line in f:
            if line.startswith("a "):
                parts = line.split()
                # Tratamento para arquivos que podem ter espaços extras
                u, v, w = parts[1], parts[2], parts[3]
                
                if u not in adj:
                    adj[u] = {}
                if v not in adj:
                    adj[v] = {}
                
                adj[u][v] = int(w)
                
                # Se NÃO for direcionado, cria a aresta de volta
                if not directed:
                    adj[v][u] = int(w) 
    
    return adj

def main():
    # 2. Alteração: Menu de escolha
    print("Selecione o tipo de Grafo:")
    print("1 - Graph (Não-direcionado)")
    print("2 - DiGraph (Direcionado)")
    choice = input("Opção: ").strip()

    is_directed = (choice == '2')

    path_file = r"C:\Users\cleyb\Downloads\USA-road-d.NY\USA-road-d.NY.gr"
    
    # Carrega o grafo com a flag correta
    print(f"Carregando grafo {'DIRECIONADO' if is_directed else 'NÃO-DIRECIONADO'}...")
    adj = load_dimacs_gr(path_file, directed=is_directed)

    # Instancia a classe correta
    if is_directed:
        g = DiGraph(adj)
    else:
        g = Graph(adj)

    # (a) mind
    print("\n=== (a) mind ===")
    mind_result = g.mind()
    
    # Tratamento porque Graph retorna 2 valores e DiGraph retorna 4
    if is_directed:
        v_min, deg_min, in_d, out_d = mind_result
        print(f"Vértice de menor grau TOTAL: {v_min}")
        print(f"Grau Total: {deg_min} (In: {in_d}, Out: {out_d})")
    else:
        v_min, deg_min = mind_result
        print(f"Vértice de menor grau: {v_min}")
        print(f"Grau: {deg_min}")

    # (c) caminho com >= 10 arestas (BFS)
    print("\n=== (c) Caminho com pelo menos 10 arestas ===")
    src = "1"
    
    # Verifica se o vértice existe antes de rodar, para evitar crash
    if g.v(src) == "Vértice não encontrado":
        print(f"Erro: O vértice de origem {src} não existe no grafo carregado.")
    else:
        dist, pi = g.bfs(src)

        dest = None
        for v in dist:
            if dist[v] != float("inf") and dist[v] >= 10:
                dest = v
                break

        if dest is None:
            print("Nenhum vértice está a 10 arestas de distância.")
        else:
            path = reconstruct_path(src, dest, pi)
            print("Destino encontrado:", dest)
            print("Nº de arestas:", dist[dest])
            # Imprime apenas os primeiros e últimos nós se o caminho for gigante
            if path and len(path) > 20: 
                print(f"Caminho: {path[:5]} ... {path[-5:]} (Total: {len(path)} nós)")
            else:
                print("Caminho:", path)

    # (e) vértice mais distante do 129 (por peso), usando Bellman-Ford
    print("\n=== (e) Vértice mais distante de 129 (por peso) ===")
    src = "129"
    
    if g.v(src) == "Vértice não encontrado":
        print(f"Erro: O vértice de origem {src} não existe.")
    else:
        dist, pi, neg = g.bf(src)

        farthest = None
        max_dist = -float('inf')

        for v in dist:
            if dist[v] != float("inf") and dist[v] > max_dist:
                max_dist = dist[v]
                farthest = v

        if neg:
            print("ALERTA: Ciclo negativo detectado — distância pode não ser confiável.")

        if farthest:
            print("Vértice mais distante:", farthest)
            print("Distância:", max_dist)
        else:
            print("Não foi possível encontrar vértices alcançáveis a partir de 129.")

if __name__ == "__main__":
    main()