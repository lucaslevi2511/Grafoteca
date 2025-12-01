# test_digraph.py
from Grafoteca import DiGraph

# lista de adjacência (digrafo orientado e ponderado)
adj = {
    'A': {'B': 1, 'C': 2},
    'B': {'C': 1, 'D': 3},
    'C': {'E': 4},
    'D': {'E': 1},
    'E': {}
}
negative_adj = {
    'A': {'B': 1, 'C': 2},
    'B': {'C': 1, 'D': 3},
    'C': {'E': 4},
    'D': {'E': 1},
    'E': {'B': -5} 
}
color_adj = {
    'A': {'B': 1, 'C': 1},
    'B': {'D': 1},
    'C': {'D': 1, 'E': 1},
    'D': {'F': 1},
    'E': {'C': 1, 'F': 1},
    'F': {'A': 1}
}

def main():
    # n() e m()
    g_print = DiGraph(adj)
    print("=== n() e m() ===")
    g_print.n()
    g_print.m()

    # v() e w()
    print("\n=== v('A') ===")
    print(g_print.v('A'))

    print("\n=== w('A','B') ===")
    print(g_print.w('A','B'))

    # mind() e maxd()
    g_deg = DiGraph(adj)
    print("\n=== mind() e maxd() ===")
    g_deg.mind()
    g_deg.maxd()

    # BFS (a partir de 'A')
    g_bfs = DiGraph(adj)
    dist_bfs, pi_bfs = g_bfs.bfs('A')
    print("\n=== BFS a partir de 'A' (dist, predecessors) ===")
    print("dist:", dist_bfs)
    print("predecessors:", pi_bfs)

    # DFS (percorrendo todo o grafo)
    g_dfs = DiGraph(adj)
    dfs_res = g_dfs.dfs()   # percorre todo o grafo
    print("\n=== DFS (init_time, finish_time, predecessors) ===")
    print("init_time:", dfs_res["init_time"])
    print("finish_time:", dfs_res["finish_time"])
    print("predecessors:", dfs_res["predecessors"])

    # Bellman-Ford a partir de 'A'
    g_bf = DiGraph(adj)
    dist_bf, pi_bf, neg_cycle = g_bf.bf('A')
    print("\n=== Bellman-Ford a partir de 'A' ===")
    print("dist:", dist_bf)
    print("predecessors:", pi_bf)
    print("negative_cycle:", neg_cycle)

    # Bellman-Ford a partir de 'A'
    g_bf = DiGraph(negative_adj)
    dist_bf, pi_bf, neg_cycle = g_bf.bf('A')
    print("\n=== Bellman-Ford a partir de 'A' com ciclo negativo ===")
    print("dist:", dist_bf)
    print("predecessors:", pi_bf)
    print("negative_cycle:", neg_cycle)

    # Dijkstra a partir de 'A'
    g_dij = DiGraph(adj)
    dist_dij = g_dij.dijkstra('A')
    print("\n=== Dijkstra a partir de 'A' ===")
    print("dist:", dist_dij)

    #teste dsatur
    dg = DiGraph(color_adj)
    coloring, num_colors = dg.coloracao_propria()
    print("\n=== Coloring ===")
    print("Coloring:", coloring)
    print("Colors used:", num_colors)

if __name__ == "__main__":
    main()
