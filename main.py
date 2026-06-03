
import pygame
import sys
import time
import math
import random

from pathfinding import create_empty_grid, scatter_walls, AStar, DStarLite
from ui_controls import Button, ToggleButton, draw_text, draw_panel

# -------------------- Configuration --------------------
FPS = 60
ROBOT_SPEED = 4.0  # tiles per second

# Colors
COLOR_BG = (25, 25, 35)
COLOR_WALL = (60, 60, 80)
COLOR_FLOOR = (220, 220, 230)
COLOR_START = (86, 188, 137)  # Green
COLOR_GOAL = (78, 137, 193)   # Blue
COLOR_ASTAR_PATH = (241, 196, 83)  # Yellow
COLOR_DSTAR_PATH = (170, 220, 170)  # Light green
COLOR_ROBOT = (231, 76, 60)   # Red
COLOR_TEXT = (240, 240, 240)

# -------------------- Helpers --------------------

def find_path_astar(grid, start, goal):
    ast = AStar(grid)
    t0 = time.perf_counter()
    path = ast.search(start, goal)
    t1 = time.perf_counter()
    metrics = {'algo':'A*', 'time_ms':(t1-t0)*1000, 'nodes':ast.last_nodes_expanded}
    return path, metrics

def init_dstar(grid, start, goal):
    dstar = DStarLite(grid, start, goal)
    t0 = time.perf_counter()
    dstar.compute_shortest_path()
    t1 = time.perf_counter()
    path = dstar.get_path()
    metrics = {'algo':'D* Lite', 'time_ms':(t1-t0)*1000, 'nodes':dstar.nodes_expanded}
    return dstar, path, metrics

def ensure_reachable_with_scatter(grid, start, goal, attempts=10, density=0.30):
    """Try scattering walls until a path exists (max attempts)."""
    rows = len(grid); cols = len(grid[0])
    keep = set()
    if start: keep.add(start)
    if goal: keep.add(goal)
    best = None
    for _ in range(attempts):
        test = [row[:] for row in grid]
        scatter_walls(test, density=density, keep=keep)
        # check reachability via A*
        path, _m = find_path_astar(test, start, goal) if start and goal else ([(1,1)], {})
        if start is None or goal is None or path:
            best = test
            break
    return best if best else grid

# -------------------- Main --------------------

def main():
    pygame.init()
    info = pygame.display.Info()
    screen_width = max(900, info.current_w - 60)
    screen_height = max(700, info.current_h - 120)

    # Grid
    grid = create_empty_grid(40, 25, border_walls=True)
    rows, cols = len(grid), len(grid[0])

    # Layout
    ui_panel_height = 130
    map_height = screen_height - ui_panel_height
    tile_w = screen_width // cols
    tile_h = map_height // rows
    TILE_SIZE = min(tile_w, tile_h)
    map_display_width = cols * TILE_SIZE
    map_display_height = rows * TILE_SIZE
    map_offset_x = (screen_width - map_display_width) // 2
    map_offset_y = (screen_height - map_display_height - ui_panel_height) // 2

    screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)
    pygame.display.set_caption('A* vs D* Lite — Pathfinding Simulator (Modular)')
    clock = pygame.time.Clock()

    # UI Components
    ui_panel = pygame.Rect(0, screen_height - ui_panel_height, screen_width, ui_panel_height)
    button_w, button_h = 130, 36
    margin = 16
    button_y = screen_height - ui_panel_height + 50

    # State
    algorithm_options = ["A*", "D* Lite"]
    algo_index = 0  # default A*
    running = False
    paused = True
    start = None
    goal = None
    robot_tile = None
    robot_pos = None
    robot_path_tiles = []
    last_metrics = {}
    dstar = None

    def set_algo_index(i):
        nonlocal algo_index, dstar, last_metrics, robot_path_tiles
        algo_index = i
        # refresh planning if start/goal set
        if start and goal:
            if algorithm_options[algo_index] == "A*":
                path, last_metrics = find_path_astar(grid, start, goal)
                if path:
                    robot_tile = path[0]
                    robot_path_tiles[:] = path[1:]
            else:
                dstar, path, last_metrics = init_dstar(grid, start, goal)
                if path:
                    robot_tile = path[0]
                    robot_path_tiles[:] = path[1:]

    # Buttons
    buttons = []
    buttons.append(Button(margin, button_y, button_w, button_h, "Start", lambda: "start"))
    buttons.append(Button(margin*2 + button_w, button_y, button_w, button_h, "Stop", lambda: "stop"))
    buttons.append(Button(margin*3 + button_w*2, button_y, button_w, button_h, "Move 1 Step", lambda: "step"))
    buttons.append(Button(margin*4 + button_w*3, button_y, button_w, button_h, "Restart", lambda: "restart"))
    buttons.append(Button(margin*5 + button_w*4, button_y, button_w, button_h, "Generate Walls", lambda: "gen_walls"))
    # Toggle on right side
    algo_toggle = ToggleButton(screen_width - button_w - margin, button_y, button_w, button_h,
                               algorithm_options, lambda: algo_index, set_algo_index)
    buttons.append(Button(screen_width - button_w - margin, button_y + button_h + 10, button_w, button_h, "Quit", lambda: "quit"))

    def recompute_if_needed():
        nonlocal dstar, robot_tile, robot_pos, robot_path_tiles, last_metrics
        if not (start and goal):
            return
        if algorithm_options[algo_index] == "A*":
            path, last_metrics = find_path_astar(grid, start, goal)
            if path:
                robot_tile = path[0]
                robot_pos = (robot_tile[0]*TILE_SIZE + TILE_SIZE/2 + map_offset_x,
                             robot_tile[1]*TILE_SIZE + TILE_SIZE/2 + map_offset_y)
                robot_path_tiles = path[1:]
        else:
            dstar, path, last_metrics = init_dstar(grid, start, goal)
            if path:
                robot_tile = path[0]
                robot_pos = (robot_tile[0]*TILE_SIZE + TILE_SIZE/2 + map_offset_x,
                             robot_tile[1]*TILE_SIZE + TILE_SIZE/2 + map_offset_y)
                robot_path_tiles = path[1:]

    # Main loop
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        mouse_pos = pygame.mouse.get_pos()
        hover_tile = None
        mx, my = mouse_pos
        # tile under mouse
        if map_offset_x <= mx < map_offset_x + map_display_width and map_offset_y <= my < map_offset_y + map_display_height:
            tx = int((mx - map_offset_x) // TILE_SIZE)
            ty = int((my - map_offset_y) // TILE_SIZE)
            if 0 <= tx < cols and 0 <= ty < rows:
                hover_tile = (tx, ty)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.VIDEORESIZE:
                screen_width, screen_height = event.w, event.h
                screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)
                ui_panel = pygame.Rect(0, screen_height - ui_panel_height, screen_width, ui_panel_height)
                button_y = screen_height - ui_panel_height + 50
                # remake buttons with updated positions
                buttons = []
                buttons.append(Button(margin, button_y, button_w, button_h, "Start", lambda: "start"))
                buttons.append(Button(margin*2 + button_w, button_y, button_w, button_h, "Stop", lambda: "stop"))
                buttons.append(Button(margin*3 + button_w*2, button_y, button_w, button_h, "Move 1 Step", lambda: "step"))
                buttons.append(Button(margin*4 + button_w*3, button_y, button_w, button_h, "Restart", lambda: "restart"))
                buttons.append(Button(margin*5 + button_w*4, button_y, button_w, button_h, "Generate Walls", lambda: "gen_walls"))
                algo_toggle = ToggleButton(screen_width - button_w - margin, button_y, button_w, button_h,
                                           algorithm_options, lambda: algo_index, set_algo_index)
                buttons.append(Button(screen_width - button_w - margin, button_y + button_h + 10, button_w, button_h, "Quit", lambda: "quit"))
                # recompute layout sizes
                map_height = screen_height - ui_panel_height
                tile_w = screen_width // cols
                tile_h = map_height // rows
                TILE_SIZE = min(tile_w, tile_h)
                map_display_width = cols * TILE_SIZE
                map_display_height = rows * TILE_SIZE
                map_offset_x = (screen_width - map_display_width) // 2
                map_offset_y = (screen_height - map_display_height - ui_panel_height) // 2
                if robot_tile:
                    robot_pos = (robot_tile[0]*TILE_SIZE + TILE_SIZE/2 + map_offset_x,
                                 robot_tile[1]*TILE_SIZE + TILE_SIZE/2 + map_offset_y)

            # UI events
            for b in buttons:
                b.check_hover(mouse_pos)
                res = b.handle_event(event)
                if res == "start":
                    paused = False
                    if not robot_path_tiles and start and goal:
                        recompute_if_needed()
                elif res == "stop":
                    paused = True
                elif res == "step":
                    paused = True
                    # move one tile if path exists
                    if robot_pos and robot_path_tiles:
                        target = robot_path_tiles.pop(0)
                        robot_tile = target
                        robot_pos = (target[0]*TILE_SIZE + TILE_SIZE/2 + map_offset_x,
                                     target[1]*TILE_SIZE + TILE_SIZE/2 + map_offset_y)
                        if algorithm_options[algo_index] == "D* Lite" and dstar:
                            dstar.reduce_heuristic_distance(robot_tile)
                            dstar.compute_shortest_path()
                            # refresh remaining path from current location
                            dstar.start = robot_tile
                            new_path = dstar.get_path()
                            if new_path and robot_tile in new_path:
                                idx = new_path.index(robot_tile)
                                robot_path_tiles = new_path[idx+1:]
                elif res == "restart":
                    # reset to clean empty grid (with border walls)
                    grid[:] = create_empty_grid(cols, rows, border_walls=True)
                    start = goal = None
                    robot_tile = None
                    robot_pos = None
                    robot_path_tiles = []
                    last_metrics = {}
                    dstar = None
                    paused = True
                elif res == "gen_walls":
                    # scatter but keep start/goal free
                    keep = set()
                    if start: keep.add(start)
                    if goal: keep.add(goal)
                    new_grid = ensure_reachable_with_scatter(grid, start, goal, attempts=10, density=0.30)
                    grid[:] = new_grid
                    # If D* is active, update edges and replan; A* stays static to show difference
                    if algorithm_options[algo_index] == "D* Lite" and dstar and start and goal and robot_tile:
                        # update many edges: easiest is to re-init and recompute for correctness
                        dstar, path, last_metrics = init_dstar(grid, robot_tile, goal)
                        if path:
                            robot_path_tiles = path[1:]
                    else:
                        # invalidate A* path (force manual recompute to highlight difference)
                        robot_path_tiles = []

                elif res == "quit":
                    running = False

            algo_toggle.check_hover(mouse_pos)
            t_res = algo_toggle.handle_event(event)
            if t_res == "toggled":
                # recompute path for new algorithm
                recompute_if_needed()
                paused = True  # pause after switching to let the user decide

            # Map interactions
            if event.type == pygame.MOUSEBUTTONDOWN and hover_tile:
                tx, ty = hover_tile
                if event.button == 1:
                    # left-click: set start then goal, then toggle walls
                    if start is None and grid[ty][tx] == 0:
                        start = (tx, ty)
                        robot_tile = start
                        robot_pos = (tx*TILE_SIZE + TILE_SIZE/2 + map_offset_x,
                                     ty*TILE_SIZE + TILE_SIZE/2 + map_offset_y)
                    elif goal is None and grid[ty][tx] == 0 and (tx,ty) != start:
                        goal = (tx, ty)
                        # plan initial path
                        recompute_if_needed()
                    else:
                        if (tx,ty) != start and (tx,ty) != goal:
                            # toggle walls
                            grid[ty][tx] = 1 if grid[ty][tx] == 0 else 0
                            if algorithm_options[algo_index] == "D* Lite" and dstar and robot_tile:
                                dstar.grid = grid
                                dstar.update_edge((tx,ty))
                                dstar.reduce_heuristic_distance(robot_tile)
                                dstar.compute_shortest_path()
                                new_path = dstar.get_path()
                                if new_path and robot_tile in new_path:
                                    idx = new_path.index(robot_tile)
                                    robot_path_tiles = new_path[idx+1:]
                            else:
                                # Invalidate A* to show that it doesn't automatically replan
                                robot_path_tiles = []
                elif event.button == 3:
                    # right-click: toggle walls anytime (same logic as above)
                    if (tx,ty) != start and (tx,ty) != goal:
                        grid[ty][tx] = 1 if grid[ty][tx] == 0 else 0
                        if algorithm_options[algo_index] == "D* Lite" and dstar and robot_tile:
                            dstar.grid = grid
                            dstar.update_edge((tx,ty))
                            dstar.reduce_heuristic_distance(robot_tile)
                            dstar.compute_shortest_path()
                            new_path = dstar.get_path()
                            if new_path and robot_tile in new_path:
                                idx = new_path.index(robot_tile)
                                robot_path_tiles = new_path[idx+1:]
                        else:
                            robot_path_tiles = []

        # Movement (continuous)
        if not paused and robot_pos and robot_path_tiles:
            target_tile = robot_path_tiles[0]
            txc = target_tile[0]*TILE_SIZE + TILE_SIZE/2 + map_offset_x
            tyc = target_tile[1]*TILE_SIZE + TILE_SIZE/2 + map_offset_y
            px, py = robot_pos
            vx = txc - px; vy = tyc - py
            dist = math.hypot(vx, vy)
            step = ROBOT_SPEED * TILE_SIZE * dt
            if step >= dist or dist < 1e-6:
                # snap & consume tile
                robot_tile = target_tile
                robot_pos = (txc, tyc)
                robot_path_tiles.pop(0)
                if algorithm_options[algo_index] == "D* Lite" and dstar:
                    dstar.reduce_heuristic_distance(robot_tile)
                    dstar.compute_shortest_path()
                    new_path = dstar.get_path()
                    if new_path and robot_tile in new_path:
                        idx = new_path.index(robot_tile)
                        robot_path_tiles = new_path[idx+1:]
            else:
                robot_pos = (px + vx/dist*step, py + vy/dist*step)

        # -------------------- Drawing --------------------
        screen.fill(COLOR_BG)
        map_bg_rect = pygame.Rect(map_offset_x, map_offset_y, map_display_width, map_display_height)
        pygame.draw.rect(screen, (50, 50, 60), map_bg_rect)

        # tiles
        for y in range(rows):
            for x in range(cols):
                rect = pygame.Rect(x*TILE_SIZE + map_offset_x, y*TILE_SIZE + map_offset_y, TILE_SIZE-1, TILE_SIZE-1)
                if grid[y][x] == 1:
                    pygame.draw.rect(screen, COLOR_WALL, rect)
                else:
                    pygame.draw.rect(screen, COLOR_FLOOR, rect)

        # path
        if robot_tile and (start and goal):
            # visualize planned path ahead
            if algorithm_options[algo_index] == "A*":
                color = COLOR_ASTAR_PATH
            else:
                color = COLOR_DSTAR_PATH
            # current tile
            cur = pygame.Rect(robot_tile[0]*TILE_SIZE + map_offset_x + 4, robot_tile[1]*TILE_SIZE + map_offset_y + 4,
                              TILE_SIZE-8, TILE_SIZE-8)
            pygame.draw.rect(screen, color, cur, border_radius=3)
            for p in robot_path_tiles:
                rect = pygame.Rect(p[0]*TILE_SIZE + map_offset_x + 4, p[1]*TILE_SIZE + map_offset_y + 4,
                                   TILE_SIZE-8, TILE_SIZE-8)
                pygame.draw.rect(screen, color, rect, border_radius=3)

        # start/goal
        if start:
            rect = pygame.Rect(start[0]*TILE_SIZE + map_offset_x + 2, start[1]*TILE_SIZE + map_offset_y + 2,
                               TILE_SIZE-4, TILE_SIZE-4)
            pygame.draw.rect(screen, COLOR_START, rect, border_radius=5)
            draw_text(screen, "S", (rect.centerx, rect.centery), 16, align="center")
        if goal:
            rect = pygame.Rect(goal[0]*TILE_SIZE + map_offset_x + 2, goal[1]*TILE_SIZE + map_offset_y + 2,
                               TILE_SIZE-4, TILE_SIZE-4)
            pygame.draw.rect(screen, COLOR_GOAL, rect, border_radius=5)
            draw_text(screen, "G", (rect.centerx, rect.centery), 16, align="center")

        # robot
        if robot_pos:
            pygame.draw.circle(screen, COLOR_ROBOT, (int(robot_pos[0]), int(robot_pos[1])), TILE_SIZE//3)

        # UI panel & buttons
        draw_panel(screen, ui_panel, "Pathfinding Controls")
        for b in buttons:
            b.draw(screen)
        algo_toggle.draw(screen)

        # status
        status_y = screen.get_height() - ui_panel_height + 10
        draw_text(screen, f"Algorithm: {algorithm_options[algo_index]}   |   Running: {not paused}",
                  (screen.get_width()//2, status_y), 18, align="center")
        if last_metrics:
            draw_text(screen, f"{last_metrics['algo']}: {last_metrics['time_ms']:.2f} ms, {last_metrics['nodes']} nodes",
                      (screen.get_width()//2, status_y + 24), 16, align="center")
        draw_text(screen, "Instructions: Left click to set Start → Goal, then toggle walls. Right click toggles walls anytime.",
                  (screen.get_width()//2, screen.get_height() - 16), 14, align="center")

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
