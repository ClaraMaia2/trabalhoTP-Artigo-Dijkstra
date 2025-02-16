import heapq
import networkx as nx
import matplotlib.pyplot as plt

def dijkstraTowers(graph, start, target):
    # Inicializando a lista de caminhos abertos, do formato: (custo_acumulado, nó_atual, caminho_percorrido)
    open_list = [(0, start, [])]
    heapq.heapify(open_list)
    
    # Inicializando o conjunto de nós já visitados (caminho fechado)
    closed_list = set()
    
    # Inicializando o dicionário que armazena o menor caminho até cada nó do grafo
    cost_min = {node: float('inf') for node in graph}
    cost_min[start] = 0     # O caminho mínimo do nó de início até ele mesmo é 0
    
    # Dicionário para armazenar o caminho percorrido com pesos
    path_edges = {}
    
    while open_list:
        cost, node, path = heapq.heappop(open_list)
        
        # Ignora se o nó já foi processado (incluso na lista de caminhos fechados)
        if node in closed_list:
            continue
        #endif
        
        # Marcando o nó atual como fechado (já processado)
        closed_list.add(node)
        
        # Atualiza o caminho percorrido, se houver um caminho válido
        if path:
            path_edges[(path[-1], node)] = cost - cost_min[path[-1]]
        
        # Atualizando o caminho percorrido
        path = path + [node]
        
        # Se o nó atual for o destino, ele retorna o caminho e o custo mínimo encontrado
        if node == target:
            formatted_path = " -> ".join(n[0] for n in path)
            edges_costs = [f"{a[0]}->{b[0]}({w})" for (a, b), w in path_edges.items() if a != b]
            
            return formatted_path, edges_costs, cost, path_edges, graph
        #endif
        
        # Expande os nós vizinhos
        for neighbor, (cost_edge, height_restriction) in graph[node].items():
            # Ignora nós vizinhos já processados (incluso no caminho fechado)
            if neighbor in closed_list:
                continue
            #endif
            
            # Verifica a restrição de altura antes de prosseguir (somente se existir restrição)
            if height_restriction and neighbor[1] < height_restriction:
                continue
            #endif
            
            # Calculando o novo custo para alcançar o vizinho
            new_cost = cost + cost_edge
            
            # Verifica se o novo custo é menor que o custo registrado
            if new_cost < cost_min[neighbor]:
                # Atualiza o custo mínimo como o novo custo
                cost_min[neighbor] = new_cost
                
                # Adiciona o novo custo, o nó vizinho e o caminho dentro da lista de nós abertos
                open_list.append((new_cost, neighbor, path))
            #endif
        #endfor
    #endwhile
    
    # Retorna nulo e custo infinito se não há caminho possível
    return None, None, float('inf')
#enddef

def drawGraph(graph, path_edges):
    # Cria o gráfico com NetworkX
    G = nx.DiGraph()

    # Adiciona os nós e as arestas ao gráfico
    for node, neighbors in graph.items():
        for neighbor, (cost, _) in neighbors.items():
            G.add_edge(node, neighbor, weight=cost)
        #endfor
    #endfor
    
    # Desenha o gráfico com posições otimizadas
    pos = nx.spring_layout(G, k=0.4, seed=42)
    plt.figure(figsize=(12, 10))

    # Desenha as arestas
    nx.draw_networkx_edges(G, pos, edgelist=G.edges(), width=1, alpha=0.7, edge_color="skyblue", style="dashed")

    # Desenha os nós
    nx.draw_networkx_nodes(G, pos, node_size=1200, node_color="lightgreen", alpha=0.9, edgecolors="darkgreen")

    # Desenha os rótulos dos nós
    nx.draw_networkx_labels(G, pos, font_size=7, font_weight="bold", font_color="darkblue")

    # Destaca o caminho ótimo em rosa
    path_edges_set = set(path_edges.keys())
    nx.draw_networkx_edges(G, pos, edgelist=path_edges_set, width=2, edge_color="pink", alpha=1)

    # Desenha os pesos das arestas
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=12, font_color="black")

    # Exibe o gráfico com título
    plt.title("Grafo com Caminho Ótimo em Destaque", fontsize=16)
    plt.axis('off')  # Remove os eixos
    plt.show()
#enddef

# Exemplo de uso com torres de transmissão
graph = {
    ('A', 10): {('B', 15): (4, 12), ('C', 20): (2, 18)},
    ('B', 15): {('A', 10): (4, None), ('C', 20): (5, 18), ('D', 30): (10, 25), ('F', 50): (8, None)},
    ('C', 20): {('A', 10): (2, None), ('B', 15): (5, None), ('D', 30): (3, 25)},
    ('D', 30): {('B', 15): (10, None), ('C', 20): (3, None), ('E', 40): (8, 35)},
    ('E', 40): {('D', 30): (8, None), ('F', 50): (7, 45), ('A', 10): (10, None)},
    ('F', 50): {('E', 40): (7, None)}
}

start_node = ('A', 10)
target_node = ('F', 50)
formatted_path, edges_costs, cost, path_edges, graph = dijkstraTowers(graph, start_node, target_node)
print(f"Caminho ótimo: {formatted_path}")
print(f"Arestas e custos: {', '.join(edges_costs)}")
print(f"Custo total: {cost}")

drawGraph(graph, path_edges)