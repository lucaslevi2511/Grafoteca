# test_grafoteca.py
from Grafoteca import Graph

# lista de adjacência (grafo não direcionado com pesos)
adj = {
    'A': {'B': 1, 'C': 2},
    'B': {'A': 1, 'C': 1, 'D': 3},
    'C': {'A': 2, 'B': 1, 'E': 4},
    'D': {'B': 3, 'E': 1},
    'E': {'C': 4, 'D': 1}
}
negative_adj = {
    'A': {'B': 1, 'C': 2},
    'B': {'A': 1, 'C': 1, 'D': 3},
    'C': {'A': -3, 'B': 1, 'E': 4}, 
    'D': {'B': 3, 'E': 1},
    'E': {'C': 4, 'D': 1}
}
coloring_adj = {
    'A': {'B': 1, 'C': 1},
    'B': {'A': 1, 'C': 1, 'D': 1},
    'C': {'A': 1, 'B': 1, 'D': 1, 'E': 1},
    'D': {'B': 1, 'C': 1, 'E': 1, 'F': 1},
    'E': {'C': 1, 'D': 1, 'F': 1},
    'F': {'D': 1, 'E': 1}
}
def main():
    # instancia para operações que imprimem (n, m, mind, maxd, v, w)
    g_print = Graph(adj)
    print("=== n() e m() ===")
    g_print.n()
    g_print.m()

    print("\n=== v('A') ===")
    print(g_print.v('A'))        # vizinhos de A

    print("\n=== w('A','B') ===")
    print(g_print.w('A','B'))    # peso de A->B

    print("\n=== mind() e maxd() ===")
    g_print.mind()
    g_print.maxd()

    # BFS: use uma instância separada para evitar interferência de dist persistente
    g_bfs = Graph(adj)
    dist_bfs, pi_bfs = g_bfs.bfs('A')
    print("\n=== BFS a partir de 'A' (dist, predecessors) ===")
    print("dist:", dist_bfs)
    print("predecessors:", pi_bfs)

    # DFS: instância separada para garantir reinicialização de tempo e visited
    g_dfs = Graph(adj)
    dfs_res = g_dfs.dfs()   # percorre todo o grafo
    print("\n=== DFS (init_time, finish_time, predecessors) ===")
    print("init_time:", dfs_res["init_time"])
    print("finish_time:", dfs_res["finish_time"])
    print("predecessors:", dfs_res["predecessors"])

    # Bellman-Ford (a partir de 'A')
    g_bf = Graph(adj)
    dist_bf, pi_bf, neg_cycle = g_bf.bf('A')
    print("\n=== Bellman-Ford a partir de 'A' ===")
    print("dist:", dist_bf)
    print("predecessors:", pi_bf)
    print("negative_cycle:", neg_cycle)

    # Bellman-Ford com ciclo negativo (a partir de 'A')
    g_bf = Graph(negative_adj)
    dist_bf, pi_bf, neg_cycle = g_bf.bf('A')
    print("\n=== Bellman-Ford a partir de 'A' com ciclo negativo ===")
    print("dist:", dist_bf)
    print("predecessors:", pi_bf)
    print("negative_cycle:", neg_cycle)

    # Dijkstra (a partir de 'A')
    g_dij = Graph(adj)
    dist_dij = g_dij.dijkstra('A')
    print("\n=== Dijkstra a partir de 'A' ===")
    print("dist:", dist_dij)

    #Dsatur
    g_color = Graph(coloring_adj)
    coloring, num_colors = g_color.coloracao_propria()
    print("\n=== Coloring ===")
    print("Coloring: ", coloring)
    print("Colors used: ", num_colors)

if __name__ == "__main__":
    main()
