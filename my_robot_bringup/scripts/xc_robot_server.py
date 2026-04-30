#!/usr/bin/env python3
"""
XC Robot Server - HTTP REST :8080 + WebSocket :8081
"""

import asyncio
import json
import math
import os
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
from aiohttp import web
import websockets

# ---------------------------------------------------------------------------
# 全局状态
# ---------------------------------------------------------------------------

nav_state = "idle"        # idle | navigating | arrived | failed
nav_id = None
target_point_id = None
failed_point_id = None    # 失败时保留目标点
nav_cancelled = False     # /stop 标志，防止 do_navigate 覆盖状态
cancel_event = threading.Event()  # 通知 do_navigate 在自己线程内取消
state_lock = threading.Lock()

current_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}

ws_clients = set()
asyncio_loop = None
navigator = None
executor = ThreadPoolExecutor(max_workers=1)

poi_map = {}

# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
POI_MAP_PATH = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "config", "poi_map.json"))


def load_poi_map():
    with open(POI_MAP_PATH, "r", encoding="utf-8") as f:
        data = json.load(f)
    return {p["point_id"]: p for p in data["points"]}


def create_pose(x, y, theta):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(float(theta) / 2)
    pose.pose.orientation.w = math.cos(float(theta) / 2)
    return pose


# ---------------------------------------------------------------------------
# WebSocket 推送
# ---------------------------------------------------------------------------

async def push_ws(msg: dict):
    if not ws_clients:
        return
    data = json.dumps(msg, ensure_ascii=False)
    await asyncio.gather(
        *[client.send(data) for client in list(ws_clients)],
        return_exceptions=True,
    )


def push_nav_status(state: str, nid: str, point_id: str):
    msg = {
        "type": "nav_status",
        "timestamp": int(time.time() * 1000),
        "data": {
            "nav_state": state,
            "nav_id": nid,
            "target_point_id": point_id,
        },
    }
    asyncio.run_coroutine_threadsafe(push_ws(msg), asyncio_loop)


# ---------------------------------------------------------------------------
# 导航子线程
# ---------------------------------------------------------------------------

def do_navigate(point_id: str, point: dict, nid: str):
    global nav_state, nav_id, target_point_id, failed_point_id, nav_cancelled

    pose = create_pose(point["x"], point["y"], point["theta"])
    navigator.goToPose(pose)

    while not navigator.isTaskComplete():
        if cancel_event.is_set():
            navigator.cancelTask()
            cancel_event.clear()
            break
        feedback = navigator.getFeedback()
        if feedback:
            p = feedback.current_pose.pose
            q = p.orientation
            siny = 2 * (q.w * q.z + q.x * q.y)
            cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
            current_pose["x"] = round(p.position.x, 3)
            current_pose["y"] = round(p.position.y, 3)
            current_pose["theta"] = round(math.atan2(siny, cosy), 4)

    # 被 /stop 打断，状态已由 handle_stop 处理
    with state_lock:
        if nav_cancelled:
            nav_cancelled = False
            return

    result = navigator.getResult()

    with state_lock:
        nav_id = None
        target_point_id = None
        if result == TaskResult.SUCCEEDED:
            nav_state = "arrived"
            failed_point_id = None
        else:
            nav_state = "failed"
            failed_point_id = point_id

    new_state = "arrived" if result == TaskResult.SUCCEEDED else "failed"
    print(f"[NAV] {point_id} -> {new_state}")
    push_nav_status(new_state, nid, point_id)

    # arrived 是瞬间事件，推完立刻回到 idle
    if new_state == "arrived":
        with state_lock:
            nav_state = "idle"


# ---------------------------------------------------------------------------
# HTTP 处理器
# ---------------------------------------------------------------------------

async def handle_navigate(request):
    global nav_state, nav_id, target_point_id, nav_cancelled

    try:
        body = await request.json()
    except Exception:
        return web.json_response(
            {"code": 1002, "msg": "PARSE_ERROR", "data": {"detail": "invalid JSON"}},
            status=400,
        )

    point_id = body.get("point_id")
    if not point_id or point_id not in poi_map:
        return web.json_response(
            {"code": 2004, "msg": "POINT_NOT_FOUND", "data": {"point_id": point_id}},
            status=400,
        )

    with state_lock:
        if nav_state == "navigating":
            return web.json_response(
                {"code": 2006, "msg": "NAV_IN_PROGRESS", "data": {"current_target": target_point_id}},
                status=409,
            )
        nid = f"NAV-{uuid.uuid4().hex[:6].upper()}"
        nav_state = "navigating"
        nav_id = nid
        target_point_id = point_id
        nav_cancelled = False

    executor.submit(do_navigate, point_id, poi_map[point_id], nid)

    return web.json_response({
        "code": 0,
        "msg": "accepted",
        "data": {
            "nav_id": nid,
            "point_id": point_id,
            "point_name": poi_map[point_id]["name"],
        },
    })


async def handle_stop(request):
    global nav_state, nav_id, target_point_id, nav_cancelled

    with state_lock:
        if nav_state != "navigating":
            return web.json_response({"code": 0, "msg": "stopped"})
        nid = nav_id
        point_id = target_point_id
        nav_state = "idle"
        nav_id = None
        target_point_id = None
        nav_cancelled = True

    cancel_event.set()
    push_nav_status("idle", nid, point_id)
    print(f"[STOP] 导航已取消: {point_id}")

    return web.json_response({"code": 0, "msg": "stopped"})


async def handle_status(request):
    with state_lock:
        state = nav_state
        t_point = target_point_id
        f_point = failed_point_id

    nav_info = {"state": state}
    if state == "navigating":
        nav_info["target_point_id"] = t_point
    elif state == "failed":
        nav_info["target_point_id"] = f_point
    else:
        nav_info["target_point_id"] = None

    return web.json_response({
        "code": 0,
        "msg": "ok",
        "data": {
            "pose": dict(current_pose),
            "nav": nav_info,
            "battery": {"level": 100, "is_charging": False},
            "errors": [],
        },
    })


# ---------------------------------------------------------------------------
# WebSocket 处理器
# ---------------------------------------------------------------------------

async def ws_handler(websocket):
    ws_clients.add(websocket)
    print(f"[WS] 客户端连接: {websocket.remote_address}")
    try:
        async for _ in websocket:
            pass
    finally:
        ws_clients.discard(websocket)
        print(f"[WS] 客户端断开: {websocket.remote_address}")


# ---------------------------------------------------------------------------
# 异步主循环
# ---------------------------------------------------------------------------

async def main_async():
    global asyncio_loop
    asyncio_loop = asyncio.get_event_loop()

    app = web.Application()
    app.router.add_post("/api/v1/navigate", handle_navigate)
    app.router.add_post("/api/v1/stop", handle_stop)
    app.router.add_get("/api/v1/status", handle_status)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", 8080)
    await site.start()
    print("[HTTP] 监听 :8080  (navigate / stop / status)")

    async with websockets.serve(ws_handler, "0.0.0.0", 8081):
        print("[WS]   监听 :8081")
        await asyncio.Future()


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def main():
    global navigator, poi_map

    poi_map = load_poi_map()
    print(f"[POI] 加载点位: {list(poi_map.keys())}")

    rclpy.init()
    navigator = BasicNavigator()

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    initial_pose_pub = navigator.create_publisher(
        PoseWithCovarianceStamped, "/initialpose", qos
    )
    home = poi_map["home"]
    init_pose = create_pose(home["x"], home["y"], home["theta"])
    init_msg = PoseWithCovarianceStamped()
    init_msg.header = init_pose.header
    init_msg.pose.pose = init_pose.pose

    time.sleep(0.5)
    for _ in range(5):
        initial_pose_pub.publish(init_msg)
        time.sleep(0.2)
    print("[INIT] 已发布初始位姿")

    navigator.waitUntilNav2Active()
    print("[INIT] Nav2 已激活，等待 AMCL 定位...")
    time.sleep(2)
    print("[INIT] 初始化完成，启动服务器")

    asyncio.run(main_async())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
