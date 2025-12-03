from Grafoteca import DiGraph

def load_dimacs_gr(path):
    adj = {}

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            if line.startswith("a "):
                _, u, v, w = line.split()
                u, v, w = int(u), int(v), int(w)

                if u not in adj:
                    adj[u] = {}
                if v not in adj:
                    adj[v] = {}

                adj[u][v] = w

    return adj


def main():

    path = r"c:\Users\Ícaro\Downloads\USA-road-d.NY.gr"

    adj = load_dimacs_gr(path)

    G = DiGraph(adj)

    ciclo = G.encontrar_caminhos_minimos(minimo=5)
    if ciclo is None:
        print("Nenhum ciclo encontrado com 5 ou mais arestas.")
    else:
        print("Ciclo encontrado (fechado):", ciclo)
        print("Número de arestas:", len(ciclo)-1)
 

if __name__ == "__main__":
    main()