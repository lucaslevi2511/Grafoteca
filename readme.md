# 📚 Grafoteca: Biblioteca de Algoritmos em Grafos

Este projeto consiste no desenvolvimento de uma biblioteca em Python para manipulação e análise de grafos (Direcionados e Não-Direcionados). O trabalho foi desenvolvido como parte da avaliação da **Disciplina de Grafos**.

**Autores:**
* 🎓 **Lucas Levi**
* 🎓 **Antônio Ícaro**

---

## 📂 Estrutura do Projeto

O projeto é dividido em componentes principais:

1.  **`Grafoteca.py`**: O núcleo do projeto. Contém a implementação das classes `Graph` e `DiGraph` e todos os algoritmos solicitados.
2.  **`casos_teste.py`**: O arquivo principal para testes de carga com grandes volumes de dados.
3.  **Arquivos Auxiliares de Teste**: `main(classe Graph).py` e `main(classe DiGraph).py`.

---

## 🛠️ A Biblioteca (`Grafoteca.py`)

A biblioteca implementa duas classes principais: `Graph` (para grafos não direcionados) e `DiGraph` (para grafos direcionados). Abaixo estão as funções disponíveis e seus retornos:

### 🔹 Métodos Básicos
* **`n()`**: Retorna o número de vértices do grafo.
* **`m()`**: Retorna o número de arestas (ou arcos).
* **`v(vertice)`**: Retorna os vizinhos de um vértice específico.
* **`d(grau)`**: Retorna o grau de um vértice específico.
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

O projeto foi estruturado para permitir tanto testes rápidos e simples quanto testes robustos com dados reais.

### 1. Testes Simplificados (`main` específicas)
Para verificar a lógica básica e depurar algoritmos em grafos pequenos e controlados (hardcoded), utilize os arquivos específicos para cada classe:

* **`main(classe Graph).py`**: Executa testes simples focados na classe de grafos não-direcionados.
* **`main(classe DiGraph).py`**: Executa testes simples focados na classe de grafos direcionados.

Esses arquivos servem para garantir que as funções estão respondendo corretamente antes de submetê-las a grandes volumes de dados.

### 2. Teste Final / Benchmark (`casos_teste.py`)
Este é o ponto de entrada principal para a avaliação do trabalho com o dataset real.
* **Objetivo:** Processar o arquivo de dados **USA-road-d.NY.gr** (Ruas de Nova York - formato DIMACS).
* **Fluxo de Execução:**
    1.  O usuário escolhe via terminal se o grafo será instanciado como `Graph` ou `DiGraph`.
    2.  O script carrega centenas de milhares de vértices.
    3.  Executa as tarefas complexas (Vértice de menor grau, Caminhos longos, Ciclos, Coloração DSATUR e Bellman-Ford).

### 📋 Requisitos para rodar o Teste Final
Certifique-se de que o arquivo de dados esteja no caminho correto especificado no código:
```python
path_file = r"C:\...\USA-road-d.NY.gr"

python casos_teste.py