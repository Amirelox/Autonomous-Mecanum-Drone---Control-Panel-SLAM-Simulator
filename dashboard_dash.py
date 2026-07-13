"""
Dash-based HMI Dashboard for Autonomous Mecanum Drone.
Replaces Streamlit for better performance (no full-page re-render).
"""
import dash
from dash import dcc, html, Input, Output, State, callback, no_update, ctx
import plotly.graph_objects as go
import numpy as np
import cv2
import math
import time as tm
import base64
import json
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(name)s] %(levelname)s: %(message)s')
log = logging.getLogger("dashboard")

# Reuse the same client/controller as before
import client
from client import controller, prob_map, command_queue
from config import ROBOT_W_WIDTH, ROBOT_L_LENGTH

DISPLAY_SIZE = (450, 450)

# ============================================================
# IMAGE RENDERING HELPERS
# ============================================================
_maze_cache = {"key": None, "img": None}

def _img_to_src(img_bgr):
    """Convert OpenCV BGR image to base64 data URI for html.Img."""
    _, buf = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
    b64 = base64.b64encode(buf).decode('utf-8')
    return f"data:image/jpeg;base64,{b64}"

def render_real_world():
    """Render the real-world simulator view."""
    global _maze_cache
    if controller.phys_maze_data is None:
        return None

    maze_key = id(controller.phys_maze_data)
    if maze_key != _maze_cache["key"]:
        maze_arr = np.array(controller.phys_maze_data, dtype=np.uint8)
        h, w = maze_arr.shape
        bg = np.zeros((h, w, 3), dtype=np.uint8)
        bg[maze_arr == 0] = [38, 38, 38]
        bg[maze_arr == 1] = [255, 255, 255]
        g_size = int(ROBOT_W_WIDTH)
        gx, gy = client.logic_to_phys(controller.goal_cell[0], controller.goal_cell[1])
        cv2.rectangle(bg,
                      (int(gx - g_size//2), int(gy - g_size//2)),
                      (int(gx + g_size//2), int(gy + g_size//2)),
                      (255, 0, 0), -1)
        _maze_cache = {"key": maze_key, "img": bg}

    img = _maze_cache["img"].copy()

    if hasattr(controller, 'laser_data') and controller.laser_data:
        from client import NUM_SENSORS, RAYS_PER_SENSOR, SENSOR_ANGLES_RAD, SENSOR_RADIUS, ray_angles_rad
        idx = 0
        for i in range(NUM_SENSORS):
            s_ang = controller.heading + SENSOR_ANGLES_RAD[i]
            sx = controller.pos_x + SENSOR_RADIUS * math.cos(s_ang)
            sy = controller.pos_y + SENSOR_RADIUS * math.sin(s_ang)
            for ang_rad in ray_angles_rad:
                if idx >= len(controller.laser_data):
                    break
                r_ang = s_ang + ang_rad
                d = controller.laser_data[idx]["d"]
                ex = int(sx + math.cos(r_ang) * d)
                ey = int(sy + math.sin(r_ang) * d)
                cv2.line(img, (int(sx), int(sy)), (ex, ey), (255, 50, 50), 1)
                idx += 1

    rx, ry = int(controller.pos_x), int(controller.pos_y)
    rw, rl = int(ROBOT_W_WIDTH), int(ROBOT_L_LENGTH)
    if 0 <= ry < img.shape[0] and 0 <= rx < img.shape[1]:
        cv2.rectangle(img,
                      (rx - rw//2, ry - rl//2),
                      (rx + rw//2, ry + rl//2),
                      (30, 144, 255), -1)

    view = cv2.resize(img, DISPLAY_SIZE, interpolation=cv2.INTER_NEAREST)
    return _img_to_src(view)

def render_slam_map():
    """Render the SLAM probability map."""
    gray = (1.0 - client.prob_map) * 255
    gray = np.clip(gray, 0, 255).astype(np.uint8)
    rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

    rx, ry = int(controller.pos_x), int(controller.pos_y)
    rw, rl = int(ROBOT_W_WIDTH), int(ROBOT_L_LENGTH)
    if 0 <= ry < rgb.shape[0] and 0 <= rx < rgb.shape[1]:
        cv2.rectangle(rgb,
                      (rx - rw//2, ry - rl//2),
                      (rx + rw//2, ry + rl//2),
                      (30, 144, 255), -1)

    margin = 60
    min_x = max(0, int(controller.min_seen_x - margin))
    max_x = min(client.PHYS_W, int(controller.max_seen_x + margin))
    min_y = max(0, int(controller.min_seen_y - margin))
    max_y = min(client.PHYS_H, int(controller.max_seen_y + margin))

    if max_x > min_x and max_y > min_y:
        crop = rgb[min_y:max_y, min_x:max_x]
        view = cv2.resize(crop, DISPLAY_SIZE, interpolation=cv2.INTER_NEAREST)
    else:
        view = cv2.resize(rgb, DISPLAY_SIZE, interpolation=cv2.INTER_NEAREST)

    return _img_to_src(view)

def _reset_explorer():
    """Reset controller state."""
    global _maze_cache
    _maze_cache = {"key": None, "img": None}
    controller.finished = True
    controller.fast_run = False
    controller.teleporting = False
    controller.optimized_path = []
    controller.exploration_path = [(1, 1)]
    controller.run_start_time = None
    controller.exploration_duration = 0.0
    controller.fast_run_duration = 0.0
    controller.fast_run_start_time = None
    controller.min_seen_x = float(client.logic_to_phys(1, 1)[0])
    controller.min_seen_y = float(client.logic_to_phys(1, 1)[1])
    controller.max_seen_x = controller.min_seen_x
    controller.max_seen_y = controller.min_seen_y
    controller.current_logic_pos = (1, 1)
    controller.target_logic = None
    controller.target_phys = None
    controller.last_vx = 0.0
    controller.last_vy = 0.0
    command_queue.put("reset")

# ============================================================
# DASH APP
# ============================================================
app = dash.Dash(__name__)
app.title = "Mecanum Drone Dashboard"

app.layout = html.Div([
    html.H1("🛸 Autonomous Mecanum Drone - Operations Dashboard",
            style={"textAlign": "center", "marginBottom": "10px"}),

    # Main display row
    html.Div([
        html.Div([
            html.H3("🌍 Simulator - Real World", style={"textAlign": "center"}),
            html.Img(id='real-world-img',
                     style={"width": "450px", "height": "450px",
                            "border": "1px solid #ccc", "backgroundColor": "#222"}),
        ], style={"display": "inline-block", "verticalAlign": "top", "padding": "10px"}),
        html.Div([
            html.H3("🗺️ Client - SLAM Grid Map", style={"textAlign": "center"}),
            html.Img(id='slam-map-img',
                     style={"width": "450px", "height": "450px",
                            "border": "1px solid #ccc", "backgroundColor": "#222"}),
        ], style={"display": "inline-block", "verticalAlign": "top", "padding": "10px"}),
    ], style={"textAlign": "center"}),

    html.Hr(),

    # Controls + Timers row
    html.Div([
        # Left column: controls
        html.Div([
            html.H4("🤖 Autonomy Control"),
            html.Button("🚀 START DFS RUN", id='btn-start',
                        style={"width": "100%", "padding": "8px",
                               "marginBottom": "5px", "cursor": "pointer"}),
            html.Button("📊 START BENCHMARK MODE (50 RUNS)", id='btn-benchmark',
                        style={"width": "100%", "padding": "8px",
                               "marginBottom": "5px", "cursor": "pointer"}),
            html.Button("🛑 STOP / EMERGENCY", id='btn-stop',
                        style={"width": "100%", "padding": "8px", "marginBottom": "5px",
                               "backgroundColor": "#ff4444", "color": "white", "cursor": "pointer"}),
            html.Button("🔄 GENERATE NEW MAZE & RESET", id='btn-reset',
                        style={"width": "100%", "padding": "8px", "cursor": "pointer"}),
            html.Hr(),
            html.H5("💻 Manual JSON API Console"),
            html.Div([
                dcc.Input(id='api-input', type='text',
                          value='{"vx": 0.0, "vy": 0.6, "w": 0.0}',
                          style={"width": "75%", "padding": "5px", "fontFamily": "monospace"}),
                html.Button("📤 Inject", id='btn-api-inject',
                            style={"padding": "5px 10px", "cursor": "pointer"}),
            ]),
            html.Div(id='api-status', style={"color": "green", "fontSize": "12px", "marginTop": "5px"}),
        ], style={"display": "inline-block", "width": "32%", "verticalAlign": "top", "padding": "10px"}),

        # Right column: timers + stats
        html.Div([
            html.H4("⏱️ Chronometr Przejazdu"),
            html.Div(id='timer-explore', style={"fontSize": "18px", "fontWeight": "bold"}),
            html.Div(id='timer-fastrun', style={"fontSize": "18px", "fontWeight": "bold"}),
            html.Hr(),
            html.H4("📈 Benchmark Stats"),
            html.Div(id='benchmark-stats'),
            html.Div(id='benchmark-chart'),
        ], style={"display": "inline-block", "width": "63%", "verticalAlign": "top", "padding": "10px"}),
    ]),

    # Hidden dummy divs for button callbacks (each Output needs unique owner)
    html.Div(id='dummy-start', style={'display': 'none'}),
    html.Div(id='dummy-benchmark', style={'display': 'none'}),
    html.Div(id='dummy-stop', style={'display': 'none'}),
    html.Div(id='dummy-reset', style={'display': 'none'}),
    html.Div(id='dummy-api', style={'display': 'none'}),

    # Interval for periodic updates
    dcc.Interval(id='interval', interval=200, n_intervals=0),
])


# ============================================================
# PERIODIC UPDATE CALLBACK
# ============================================================
@app.callback(
    [Output('real-world-img', 'src'),
     Output('slam-map-img', 'src'),
     Output('timer-explore', 'children'),
     Output('timer-fastrun', 'children'),
     Output('benchmark-stats', 'children'),
     Output('benchmark-chart', 'children')],
    [Input('interval', 'n_intervals')]
)
def update_dashboard(n):
    real_src = render_real_world()
    slam_src = render_slam_map()
    if real_src is None:
        real_src = "data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"

    if not controller.finished:
        if not controller.fast_run:
            if getattr(controller, 'run_start_time', None) is not None:
                elapsed = tm.time() - controller.run_start_time
                explore_txt = f"🔍 DFS Exploration (LIVE): {elapsed:.2f}s"
            else:
                explore_txt = "🔍 DFS Exploration: waiting..."
            fast_txt = "⚡ Fast Run: waiting..."
        else:
            explore_txt = f"🏁 Last DFS: {controller.exploration_duration:.2f}s" if controller.exploration_duration else "🏁 Last DFS: ---"
            if getattr(controller, 'fast_run_start_time', None) is not None:
                elapsed = tm.time() - controller.fast_run_start_time
                fast_txt = f"⚡ Fast Run (LIVE): {elapsed:.2f}s"
            else:
                fast_txt = "⚡ Fast Run: waiting..."
    else:
        explore_txt = f"🏁 Last DFS: {controller.exploration_duration:.2f}s" if controller.exploration_duration else "🏁 Last DFS: ---"
        fast_txt = f"🏆 Last Fast Run: {controller.fast_run_duration:.2f}s" if controller.fast_run_duration else "🏆 Last Fast Run: ---"

    stats_div = html.Div("No benchmark data yet.")
    chart_div = html.Div()

    if controller.fast_run_times:
        times = controller.fast_run_times
        mean_t = np.mean(times)
        min_t = np.min(times)
        max_t = np.max(times)
        std_t = np.std(times)
        progress = len(times)

        stats_div = html.Div([
            f"Progress: {progress}/50 runs completed.",
            html.Div([
                html.Span(f"Mean: {mean_t:.3f}s  ", style={"marginRight": "15px"}),
                html.Span(f"Min: {min_t:.3f}s  ", style={"marginRight": "15px"}),
                html.Span(f"Max: {max_t:.3f}s  ", style={"marginRight": "15px"}),
                html.Span(f"Std: {std_t:.3f}s"),
            ], style={"marginTop": "5px"})
        ])

        fig = go.Figure(data=go.Scatter(y=times, mode='lines+markers', name='Fast Run Times'))
        fig.update_layout(
            title="Fast Run Times per Trial",
            xaxis_title="Trial #",
            yaxis_title="Time (s)",
            height=200,
            margin=dict(l=10, r=10, t=30, b=10),
        )
        chart_div = dcc.Graph(figure=fig, style={"height": "200px"})

    return real_src, slam_src, explore_txt, fast_txt, stats_div, chart_div


# ============================================================
# BUTTON CALLBACKS (each with unique dummy output)
# ============================================================
@app.callback(
    Output('dummy-start', 'children'),
    Input('btn-start', 'n_clicks'),
    prevent_initial_call=True
)
def on_start(n):
    if controller.finished or not controller.fast_run:
        controller.finished = False
        controller.last_vx = 0.0
        controller.last_vy = 0.0
        controller.run_start_time = tm.time()
        controller.exploration_duration = 0.0
        controller.fast_run_duration = 0.0
        controller.fast_run_start_time = None
        controller.update_target()
        log.info("🚀 START DFS RUN (Dash)")
    return ""


@app.callback(
    Output('dummy-benchmark', 'children'),
    Input('btn-benchmark', 'n_clicks'),
    prevent_initial_call=True
)
def on_benchmark(n):
    controller.benchmark_mode = True
    controller.current_benchmark_run = 1
    controller.fast_run_times = []
    controller.run_start_time = tm.time()
    controller.exploration_duration = 0.0
    controller.fast_run_duration = 0.0
    controller.reset_logic_and_maps(keep_benchmark=True)
    command_queue.put("reset")
    log.info("📊 Benchmark mode started (Dash)")
    return ""


@app.callback(
    Output('dummy-stop', 'children'),
    Input('btn-stop', 'n_clicks'),
    prevent_initial_call=True
)
def on_stop(n):
    controller.finished = True
    controller.benchmark_mode = False
    log.warning("🛑 Emergency stop (Dash)")
    return ""


@app.callback(
    Output('dummy-reset', 'children'),
    Input('btn-reset', 'n_clicks'),
    prevent_initial_call=True
)
def on_reset(n):
    controller.phys_maze_data = None
    _reset_explorer()
    log.info("🔄 New maze generated (Dash)")
    return ""


@app.callback(
    Output('dummy-api', 'children'),
    Input('btn-api-inject', 'n_clicks'),
    State('api-input', 'value'),
    prevent_initial_call=True
)
def on_api_inject(n, payload):
    try:
        cmd = json.loads(payload)
        if not controller.finished:
            controller.last_vx = cmd.get("vx", 0.0)
            controller.last_vy = cmd.get("vy", 0.0)
        command_queue.put(cmd)
        log.info(f"📤 API inject: {payload}")
        return ""
    except json.JSONDecodeError:
        log.warning(f"❌ Invalid JSON: {payload}")
        return ""


# ============================================================
# MAIN
# ============================================================
if __name__ == '__main__':
    log.info("Starting Dash dashboard on http://127.0.0.1:8501")
    app.run(host='127.0.0.1', port=8501, debug=False)
