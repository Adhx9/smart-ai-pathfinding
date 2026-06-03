
import heapq
import math
import random

# -------------------- Map Utilities --------------------

def create_empty_grid(width=40, height=25, border_walls=True):
    grid = [[0 for _ in range(width)] for _ in range(height)]
    if border_walls:
        for y in range(height):
            grid[y][0] = 1
            grid[y][-1] = 1
        for x in range(width):
            grid[0][x] = 1
            grid[-1][x] = 1
    return grid

def scatter_walls(grid, density=0.30, keep=set()):
    """Scatter random walls (1) across free cells (0). Keep cells in `keep` free."""
    rows = len(grid); cols = len(grid[0])
    for y in range(rows):
        for x in range(cols):
            if (x, y) in keep:
                continue
            # never place on borders if they are already walls; otherwise respect current value
            if grid[y][x] == 0 and random.random() < density:
                grid[y][x] = 1
            elif grid[y][x] == 1 and random.random() < 0.05:
                # occasionally open up a wall to avoid total blockage
                grid[y][x] = 0

def copy_grid(grid):
    return [row[:] for row in grid]

# -------------------- A* (grid, 4-neighborhood) --------------------

class AStar:
    def __init__(self, grid, heuristic=None):
        self.grid = grid
        self.h = heuristic if heuristic else (lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1]))
        self.last_nodes_expanded = 0

    def in_bounds(self, x, y):
        return 0 <= y < len(self.grid) and 0 <= x < len(self.grid[0])

    def passable(self, x, y):
        return self.grid[y][x] == 0

    def neighbors(self, node):
        x, y = node
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx, ny = x+dx, y+dy
            if self.in_bounds(nx, ny) and self.passable(nx, ny):
                yield (nx, ny)

    def search(self, start, goal):
        self.last_nodes_expanded = 0
        open_heap = []
        heapq.heappush(open_heap, (self.h(start, goal), 0, start))
        came_from = {}
        gscore = {start: 0}
        closed = set()

        while open_heap:
            f, g, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)
            self.last_nodes_expanded += 1
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            for nb in self.neighbors(current):
                tentative_g = gscore[current] + 1
                if tentative_g < gscore.get(nb, float('inf')):
                    came_from[nb] = current
                    gscore[nb] = tentative_g
                    heapq.heappush(open_heap, (tentative_g + self.h(nb, goal), tentative_g, nb))
        return []

# -------------------- D* Lite (grid, 4-neighborhood) --------------------

class PriorityQueue:
    def __init__(self):
        self.heap = []
        self.entry_finder = {}
        self.REMOVED = '<removed>'
        self.counter = 0

    def push(self, key, item):
        if item in self.entry_finder:
            self.remove(item)
        entry = [key, self.counter, item]
        self.counter += 1
        self.entry_finder[item] = entry
        heapq.heappush(self.heap, entry)

    def remove(self, item):
        entry = self.entry_finder.pop(item, None)
        if entry:
            entry[-1] = self.REMOVED

    def pop(self):
        while self.heap:
            key, _, item = heapq.heappop(self.heap)
            if item is not self.REMOVED:
                self.entry_finder.pop(item, None)
                return key, item
        return None, None

    def top_key(self):
        while self.heap:
            key, _, item = self.heap[0]
            if item is self.REMOVED:
                heapq.heappop(self.heap)
                continue
            return key
        return (float('inf'), float('inf'))

    def empty(self):
        return not any(e[-1] is not self.REMOVED for e in self.heap)

class DStarLite:
    def __init__(self, grid, start, goal, heuristic=None):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.start = start
        self.goal = goal
        self.h = heuristic if heuristic else (lambda a,b: abs(a[0]-b[0]) + abs(a[1]-b[1]))
        self.km = 0
        self.U = PriorityQueue()
        self.rhs = {}
        self.g = {}
        self.nodes_expanded = 0
        self.last = start
        self._init_vars()

    def _init_vars(self):
        for y in range(self.rows):
            for x in range(self.cols):
                self.rhs[(x,y)] = float('inf')
                self.g[(x,y)] = float('inf')
        self.rhs[self.goal] = 0
        self.U.push(self.calculate_key(self.goal), self.goal)

    def _in_bounds(self, s):
        x, y = s
        return 0 <= x < self.cols and 0 <= y < self.rows

    def cost(self, a, b):
        if not self._in_bounds(b):
            return float('inf')
        if self.grid[b[1]][b[0]] == 1:
            return float('inf')
        return 1

    def neighbors(self, s):
        x, y = s
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nb = (x+dx, y+dy)
            if self._in_bounds(nb):
                yield nb

    def predecessors(self, s):
        return list(self.neighbors(s))

    def calculate_key(self, s):
        val = min(self.g[s], self.rhs[s])
        return (val + self.h(self.start, s) + self.km, val)

    def update_vertex(self, u):
        if u != self.goal:
            best = float('inf')
            for s in self.neighbors(u):
                c = self.cost(u, s)
                best = min(best, c + self.g[s])
            self.rhs[u] = best
        if u in self.U.entry_finder:
            self.U.remove(u)
        if self.g[u] != self.rhs[u]:
            self.U.push(self.calculate_key(u), u)

    def compute_shortest_path(self, max_iters=5_000_000):
        iters = 0
        while True:
            iters += 1
            if iters > max_iters:
                break
            topk = self.U.top_key()
            startk = self.calculate_key(self.start)
            if topk >= startk and self.rhs[self.start] == self.g[self.start]:
                break
            key, u = self.U.pop()
            if u is None:
                break
            if key < self.calculate_key(u):
                self.U.push(self.calculate_key(u), u)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for p in self.predecessors(u):
                    self.update_vertex(p)
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)
                for p in self.predecessors(u):
                    self.update_vertex(p)
            self.nodes_expanded += 1

    def get_path(self):
        path = []
        s = self.start
        if self.g[s] == float('inf') and self.rhs[s] == float('inf'):
            return []
        path.append(s)
        visited = set()
        while s != self.goal:
            visited.add(s)
            best = None
            bestv = float('inf')
            for nb in self.neighbors(s):
                c = self.cost(s, nb)
                v = c + self.g[nb]
                if v < bestv:
                    bestv = v
                    best = nb
            if best is None or best in visited:
                return []
            s = best
            path.append(s)
        return path

    def update_edge(self, cell):
        self.update_vertex(cell)
        for n in self.neighbors(cell):
            self.update_vertex(n)

    def reduce_heuristic_distance(self, new_start):
        self.km += self.h(self.last, new_start)
        self.last = new_start
        self.start = new_start
