from heapq import heappop, heappush

class Graph:
    def __init__(self, nodes, edges, fire_intensity):
        self.nodes = nodes
        self.edges = edges
        self.fire_intensity = fire_intensity

    def neighbors(self, node):
        return self.edges[node].keys()

    def add_edge(self, node1, node2, weight):
        if node1 not in self.edges:
            self.edges[node1] = {}
        self.edges[node1][node2] = weight

    def get_weight(self, node1, node2):
        return self.edges[node1].get(node2, float("inf"))

def dijkstra_modified(graph, source, goal, safe_threshold, alpha, beta, full_path=True):
    distances = {node: float("inf") for node in graph.nodes}
    distances[source] = 0
    parents = {node: None for node in graph.nodes}
    pq = [(0, source)]  # (estimated distance, node)

    while pq:
        cost, current = heappop(pq)
        if current == goal:
            break
        if cost > distances[current]:
            continue

        for neighbor in graph.neighbors(current):
            # Check if edge exists before accessing weight
            if neighbor not in graph.edges[current]:
                continue

            edge_weight = graph.get_weight(current, neighbor)
            new_fire_intensity = graph.fire_intensity[neighbor]
            cumulative_fire_intensity = distances[current] * beta

            # Modified distance calculation
            new_distance = edge_weight + alpha * new_fire_intensity + cumulative_fire_intensity

            if new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                parents[neighbor] = current
                heappush(pq, (new_distance, neighbor))

    # Validate safe path and print
    path = []
    node = goal
    while node:
        path.append(node)
        # Check fire intensity for every node in the path
        if graph.fire_intensity[node] > safe_threshold:
            print("Safe path not found! Fire intensity exceeds threshold on node", node)
            return None
        node = parents[node]
    path.reverse()
    print(f"Safe Path: {path}")

# Example usage
graph_dict = {
    "A": {"B": 5, "C": 2},
    "B": {"C": 3, "D": 4},
    "C": {"D": 1, "E": 7},
    "D": {"E": 6},
    "E": {},
}
fire_intensity = {"A": 1, "B": 2, "C": 3, "D": 5, "E": 4}

safe_threshold = 7
alpha = 0.5
beta = 0.2

nodes = list(graph_dict.keys())
edges = graph_dict
graph = Graph(nodes, edges, fire_intensity)

dijkstra_modified(graph, "A", "E", safe_threshold, alpha, beta)
