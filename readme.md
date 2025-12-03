# 📚 Grafoteca: Biblioteca de Algoritmos em Grafos

Este projeto consiste no desenvolvimento de uma biblioteca em Python para manipulação e análise de grafos (Direcionados e Não-Direcionados). O trabalho foi desenvolvido como parte da avaliação da **Disciplina de Grafos**.

**Autores:**
* 🎓 **Lucas Levi**
* 🎓 **Antônio Ícaro**

---

## 📂 Estrutura do Projeto

O projeto é dividido em dois componentes principais:

1.  **`Grafoteca.py`**: O núcleo do projeto. Contém a implementação das classes `Graph` e `DiGraph` e todos os algoritmos solicitados.
2.  **`casos_teste.py`** (Main): O arquivo de execução principal, responsável por carregar grandes bases de dados (formato DIMACS) e executar os testes de desempenho e corretude.

---

## 🛠️ A Biblioteca (`Grafoteca.py`)

A biblioteca implementa duas classes principais: `Graph` (para grafos não direcionados) e `DiGraph` (para grafos direcionados). Abaixo estão as funções disponíveis e seus retornos:

### 🔹 Métodos Básicos
* **`n()`**: Retorna o número de vértices do grafo.
* **`m()`**: Retorna o número de arestas (ou arcos).
* **`v(vertice)`**: Retorna os vizinhos de um vértice específico.
* **`w(u, v)`**: Retorna o peso da aresta entre `u` e `v`.
* **`mind()`**: Retorna o vértice de **grau mínimo** e seu valor.
    * *No DiGraph, retorna também o grau de entrada e saída.*
* **`maxd()`**: Retorna o vértice de **grau máximo** e seu valor.

### 🔹 Buscas e Caminhos
* **`bfs(s)`**: Executa a **Busca em Largura** a partir da origem `s`.
    * *Retorno:* Uma tupla contendo `(distancias, predecessores)`.
* **`dfs(s)`**: Executa a **Busca em Profundidade**.
    * *Retorno:* Dicionário com `tempos de descoberta`, `tempos de finalização` e `predecessores`.
* **`bf(s)`** (Bellman-Ford): Calcula caminhos mínimos aceitando pesos negativos.
    * *Retorno:* Tupla `(distancias, predecessores, detectou_ciclo_negativo)`.
* **`dijkstra(s)`**: Calcula caminhos mínimos para grafos com pesos não negativos (usando Heap).
    * *Retorno:* Dicionário de distâncias mínimas.

### 🔹 Algoritmos Avançados
* **`coloracao_propria()`**: Implementação do algoritmo **DSATUR** otimizado com *Heap (Fila de Prioridade)* e *Lazy Removal*.
    * *Retorno:* Tupla contendo `(dicionario_de_cores, numero_cromatico)`.
* **`encontrar_ciclo_minimo(minimo=k)`**: Busca ciclos fechados no grafo com tamanho maior ou igual a `k`.
    * *Retorno:* Lista de vértices representando o ciclo ou `None`.

---

## 🚀 Como Executar e Testar

O projeto possui duas formas de verificação, destinadas a propósitos diferentes:

### 1. Testes Unitários (`if __name__ == "__main__"` nas classes)
Dentro do arquivo da biblioteca ou arquivos de classe individuais, existem blocos `main` menores.
* **Objetivo:** Testes rápidos e depuração de grafos pequenos e hardcoded (criados manualmente no código).
* **Uso:** Validar se a lógica básica de uma função (ex: Dijkstra) está correta antes de rodar em grafos gigantes.

### 2. Casos de Teste Reais (`casos_teste.py`)
Este é o ponto de entrada principal para a avaliação do trabalho.
* **Objetivo:** Processar o arquivo de dados reais **USA-road-d.NY.gr** (Ruas de Nova York - formato DIMACS).
* **Fluxo de Execução:**
    1.  O usuário escolhe se o grafo será instanciado como `Graph` ou `DiGraph`.
    2.  O script carrega centenas de milhares de vértices e arestas.
    3.  Executa as tarefas solicitadas (Vértice de menor grau, Caminhos longos via BFS, Ciclos, Coloração e Bellman-Ford).

### 📋 Requisitos para rodar o Teste Final
Certifique-se de que o arquivo de dados esteja no caminho correto especificado no código:
```python
path_file = r"C:\...\USA-road-d.NY.gr"

python casos_teste.py