import heapq
from math import inf
import math
import json
import os
import telebot

# --- 1) Graph & Coordinates ---
# 1=Top-Left, 2=Mid-Left, 3=Bot-Left, 4=Top-Right, 5=Mid-Right, 6=Bot-Right
graph = {
    1: [(2, 230)],
    2: [(1, 230), (3, 234), (5, 575)],
    3: [(2, 234)],
    4: [(5, 230)],
    5: [(4, 230), (6, 234), (2, 575)],
    6: [(5, 234)],
}

coords = {
    1: (332, 160), 2: (332, 390), 3: (332, 624),
    4: (907, 160), 5: (907, 390), 6: (907, 624),
}

# --- 2) Helpers (Unchanged) ---
def dist(u, v):
    x1, y1 = coords[u]
    x2, y2 = coords[v]
    return math.hypot(x2 - x1, y2 - y1)

def add_undirected_edge(u, v, weight=None):
    if weight is None: weight = dist(u, v)
    graph.setdefault(u, []).append((v, weight))
    graph.setdefault(v, []).append((u, weight))

def remove_undirected_edge(u, v):
    graph[u] = [(n, w) for (n, w) in graph.get(u, []) if n != v]
    graph[v] = [(n, w) for (n, w) in graph.get(v, []) if n != u]

def point_on_segment(u, v, t):
    x1, y1 = coords[u]
    x2, y2 = coords[v]
    return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))

def add_between_node(node_name, u, v, t=0.5):
    if not (0.0 < t < 1.0): raise ValueError("t must be between 0 and 1 (exclusive).")
    coords[node_name] = point_on_segment(u, v, t)
    remove_undirected_edge(u, v)
    add_undirected_edge(u, node_name)
    add_undirected_edge(node_name, v)

# --- 3) Routing Logic (Unchanged) ---
def shortest_path(graph, start, goal):
    dist_map = {node: inf for node in graph}
    prev = {node: None for node in graph}
    dist_map[start] = 0
    pq = [(0, start)]

    while pq:
        d, u = heapq.heappop(pq)
        if u == goal: break
        if d != dist_map[u]: continue
        for v, w in graph[u]:
            nd = d + w
            if nd < dist_map[v]:
                dist_map[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path if path and path[0] == start else None

def turn_direction(prev_node, node, next_node, coords, y_down=True):
    x0, y0 = coords[prev_node]
    x1, y1 = coords[node]
    x2, y2 = coords[next_node]
    ax, ay = (x1 - x0), (y1 - y0)
    bx, by = (x2 - x1), (y2 - y1)
    cross = ax * by - ay * bx
    if abs(cross) < 1e-9: return "Straight"
    if y_down: return "Left" if cross > 0 else "Right"
    else: return "Right" if cross > 0 else "Left"

def directions_for_path(path, coords):
    if not path or len(path) < 2: return []
    lines = []
    lines.append(f"Straight from {path[0]} to {path[1]}")
    for i in range(1, len(path) - 1):
        td = turn_direction(path[i - 1], path[i], path[i + 1], coords, y_down=False)
        if td != "Straight":
            lines.append(f"{td} at {path[i]}")
        lines.append(f"Straight from {path[i]} to {path[i + 1]}")
    return lines

# --- 4) Beacon mapping & route file ---

# Maps graph node ID → beacon name in ble_scanner.py / hallway_nav.py
NODE_TO_BEACON = {
    1: "corner_1",
    5: "corner_2",
    3: "corner_3",
}

ROUTE_FILE = "/tmp/jetbot_route.json"

def write_route(start, goal, path, waypoints):
    """Write the computed route to the shared route file (atomic)."""
    tmp = ROUTE_FILE + ".tmp"
    payload = {
        "start": str(start),
        "goal":  str(goal),
        "path":  [str(n) for n in path],
        "waypoints": waypoints,   # [{"node": n, "beacon": "...", "direction": "Left"/"Right"}, ...]
    }
    with open(tmp, "w") as f:
        json.dump(payload, f, indent=2)
    os.replace(tmp, ROUTE_FILE)

# --- 5) TELEGRAM BOT SETUP ---

# Initialize your custom nodes
# Added HomeBase on the middle horizontal line (between 2 and 5)
#add_between_node("HomeBase", 2, 5, t=0.7) 

# (Note: Node B was previously between the bottom two nodes, but since that path is blocked 
# by a door, I have commented it out so it doesn't accidentally create an invalid shortcut)
# add_between_node("B", 3, 6, t=0.5)

# Initialize the bot (PASTE YOUR TOKEN HERE)
API_TOKEN = '8420334590:AAHkRkPGk91udFfQF6jhAE5xeeyoJCmsaU0'
bot = telebot.TeleBot(API_TOKEN)

def parse_input(val):
    """Converts input to int if it's a number, else keeps it a string."""
    return int(val) if val.isdigit() else val

@bot.message_handler(commands=['start', 'help'])
def send_welcome(message):
    bot.reply_to(message, "🤖 Jetbot Routing Online!\n\nSend a command like:\n`/route HomeBase 6`\nor\n`/route 1 B`", parse_mode="Markdown")

@bot.message_handler(commands=['route'])
def handle_route(message):
    # Split the message into parts: ['/route', 'HomeBase', '6']
    parts = message.text.split()

    if len(parts) != 3:
        bot.reply_to(message, "⚠️ Format error. Please use: /route <start> <destination>")
        return

    start_val = parse_input(parts[1])
    goal_val = parse_input(parts[2])

    # Check if nodes exist in the graph
    if start_val not in graph or goal_val not in graph:
        bot.reply_to(message, f"❌ One or both nodes are invalid.")
        return

    # Calculate the path
    path = shortest_path(graph, start_val, goal_val)

    if path:
        dirs = directions_for_path(path, coords)

        # Build ordered waypoints — only beacon nodes that require a turn
        waypoints = []
        for i in range(1, len(path) - 1):
            node = path[i]
            beacon = NODE_TO_BEACON.get(node)
            if beacon is None:
                continue  # no beacon at this node, nav can't gate on it
            direction = turn_direction(path[i - 1], path[i], path[i + 1], coords, y_down=False)
            if direction == "Straight":
                continue  # no turn needed, skip
            waypoints.append({"node": node, "beacon": beacon, "direction": direction})

        # Write route file for hallway_nav.py
        write_route(start_val, goal_val, path, waypoints)

        # Print to Jetbot console
        print("\n" + "="*30)
        print(f"NEW ROUTE COMMAND RECEIVED")
        print(f"Source: {start_val} | Destination: {goal_val}")
        print(f"Path: {path}")
        print("Directions:")
        for d in dirs:
            print(f"  - {d}")
        print(f"Waypoints written to {ROUTE_FILE}:")
        for wp in waypoints:
            print(f"  Node {wp['node']} ({wp['beacon']}): {wp['direction']}")
        print("="*30 + "\n")

        # Confirm back to Telegram
        route_summary = "\n".join(f"• {d}" for d in dirs)
        bot.reply_to(
            message,
            f"✅ Route {start_val} → {goal_val} sent to Jetbot.\n\nSteps:\n{route_summary}"
        )

    else:
        bot.reply_to(message, "❌ No path could be found between those nodes.")
        print(f"\n⚠️ FAILED ROUTE: Could not find path from {start_val} to {goal_val}\n")

# This keeps the script running and listening for messages
print("Bot is polling. Open Telegram and send /start to your bot!")
bot.infinity_polling()