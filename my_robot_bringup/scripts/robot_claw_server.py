#!/usr/bin/env python3
"""
Robot Claw Server - HTTP REST :8080 + WebSocket :8081
Consumer API V1.1
"""

import asyncio
import json
import math
import os
import subprocess
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta

import cv2
import rclpy
import rclpy.node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.executors import SingleThreadedExecutor
from cv_bridge import CvBridge
from aiohttp import web
import websockets

# ---------------------------------------------------------------------------
# 速度档位配置
# ---------------------------------------------------------------------------

SPEED_LINEAR = {"slow": 0.1, "normal": 0.2, "fast": 0.3}
SPEED_ANGULAR = {"slow": 0.3, "normal": 0.5, "fast": 0.8}
WATCHDOG_TIMEOUT = 2.0  # 手动控制超时自动停车（秒）

# ---------------------------------------------------------------------------
# 全局状态
# ---------------------------------------------------------------------------

# robot_state: idle | manual | navigating | failed
robot_state = "idle"

# nav_state: idle | navigating | succeeded | failed | stopped
nav_state = "idle"
target_point_id = None
nav_cancelled = False
cancel_event = threading.Event()

current_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}

state_lock = threading.Lock()

ws_clients = set()
asyncio_loop = None
navigator = None
cmd_vel_pub = None
executor = ThreadPoolExecutor(max_workers=1)

poi_map = {}

# 手动控制 watchdog
watchdog_timer = None
watchdog_lock = threading.Lock()

# ---------------------------------------------------------------------------
# 相机配置
# ---------------------------------------------------------------------------

CAPTURE_DIR = "/home/sunrise/claw_image"
CAPTURE_TTL = 600        # 图片保留时间（秒）
MAX_CAPTURES = 100       # 最多保留图片数量

latest_rgb = None
latest_depth = None
camera_lock = threading.Lock()

image_store = {}         # image_id -> {created_at, rgb_path, has_depth, depth_path}
image_store_lock = threading.Lock()

bridge = CvBridge()
camera_node = None

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


def publish_twist(linear_x: float, angular_z: float):
    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    cmd_vel_pub.publish(msg)


def stop_robot():
    publish_twist(0.0, 0.0)


# ---------------------------------------------------------------------------
# 手动控制 Watchdog
# ---------------------------------------------------------------------------

def _watchdog_timeout():
    global robot_state
    print("[WATCHDOG] 手动控制超时，自动停车")
    stop_robot()
    with state_lock:
        if robot_state == "manual":
            robot_state = "idle"


def reset_watchdog():
    global watchdog_timer
    with watchdog_lock:
        if watchdog_timer is not None:
            watchdog_timer.cancel()
        watchdog_timer = threading.Timer(WATCHDOG_TIMEOUT, _watchdog_timeout)
        watchdog_timer.start()


def cancel_watchdog():
    global watchdog_timer
    with watchdog_lock:
        if watchdog_timer is not None:
            watchdog_timer.cancel()
            watchdog_timer = None


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


def push_nav_status(state: str, point_id: str):
    msg = {
        "type": "nav_status",
        "timestamp": int(time.time() * 1000),
        "data": {
            "nav_state": state,
            "target_point_id": point_id,
        },
    }
    asyncio.run_coroutine_threadsafe(push_ws(msg), asyncio_loop)


# ---------------------------------------------------------------------------
# 导航子线程
# ---------------------------------------------------------------------------

def do_navigate(point_id: str, point: dict):
    global robot_state, nav_state, target_point_id, nav_cancelled

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

    with state_lock:
        if nav_cancelled:
            nav_cancelled = False
            return

    result = navigator.getResult()

    with state_lock:
        if result == TaskResult.SUCCEEDED:
            nav_state = "succeeded"
            robot_state = "idle"
        else:
            nav_state = "failed"
            robot_state = "failed"
        target_point_id = point_id  # 终态保留目标点，直到下次导航覆盖

    new_nav_state = "succeeded" if result == TaskResult.SUCCEEDED else "failed"
    print(f"[NAV] {point_id} -> {new_nav_state}")
    push_nav_status(new_nav_state, point_id)


# ---------------------------------------------------------------------------
# 手动控制处理器
# ---------------------------------------------------------------------------

def _parse_speed(body: dict, move_type: str):
    speed_level = body.get("speed_level", "normal")
    if speed_level not in SPEED_LINEAR:
        speed_level = "normal"
    if move_type in ("forward", "backward"):
        explicit = body.get("speed_mps")
        speed = float(explicit) if explicit is not None else SPEED_LINEAR[speed_level]
        return speed if move_type == "forward" else -speed, 0.0
    else:
        explicit = body.get("angular_radps")
        angular = float(explicit) if explicit is not None else SPEED_ANGULAR[speed_level]
        return 0.0, angular if move_type == "left" else -angular


async def handle_move(request, direction: str):
    global robot_state, nav_state

    try:
        body = await request.json() if request.content_length else {}
    except Exception:
        body = {}

    with state_lock:
        if robot_state == "navigating":
            return web.json_response(
                {"code": 2006, "msg": "NAV_IN_PROGRESS", "data": {"current_target": target_point_id}},
                status=409,
            )
        robot_state = "manual"

    linear_x, angular_z = _parse_speed(body, direction)
    publish_twist(linear_x, angular_z)
    reset_watchdog()

    msg_map = {
        "forward": "moving_forward",
        "backward": "moving_backward",
        "left": "turning_left",
        "right": "turning_right",
    }

    return web.json_response({
        "code": 0,
        "msg": msg_map[direction],
        "data": {"robot_state": "manual", "direction": direction},
    })


async def handle_move_forward(request):
    return await handle_move(request, "forward")

async def handle_move_backward(request):
    return await handle_move(request, "backward")

async def handle_move_left(request):
    return await handle_move(request, "left")

async def handle_move_right(request):
    return await handle_move(request, "right")


# ---------------------------------------------------------------------------
# 停止处理器
# ---------------------------------------------------------------------------

async def handle_stop(request):
    global robot_state, nav_state, target_point_id, nav_cancelled

    cancel_watchdog()
    stop_robot()

    with state_lock:
        prev_state = robot_state
        prev_nav_target = target_point_id

        if robot_state == "navigating":
            nav_state = "stopped"
            nav_cancelled = True
            cancel_event.set()

        robot_state = "idle"

    if prev_state == "navigating":
        push_nav_status("stopped", prev_nav_target)
        print(f"[STOP] 导航已取消: {prev_nav_target}")

    return web.json_response({"code": 0, "msg": "stopped", "data": {"robot_state": "idle"}})


# ---------------------------------------------------------------------------
# 导航处理器
# ---------------------------------------------------------------------------

async def handle_navigate(request):
    global robot_state, nav_state, target_point_id, nav_cancelled

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
        if robot_state == "navigating":
            return web.json_response(
                {"code": 2006, "msg": "NAV_IN_PROGRESS", "data": {"current_target": target_point_id}},
                status=409,
            )
        # 手动控制中先停车再切换导航
        if robot_state == "manual":
            cancel_watchdog()
            stop_robot()

        robot_state = "navigating"
        nav_state = "navigating"
        target_point_id = point_id
        nav_cancelled = False

    executor.submit(do_navigate, point_id, poi_map[point_id])

    return web.json_response({
        "code": 0,
        "msg": "accepted",
        "data": {
            "point_id": point_id,
            "point_name": poi_map[point_id]["name"],
            "nav_state": "navigating",
        },
    })




async def handle_navigate_pose(request):
    global robot_state, nav_state, target_point_id, nav_cancelled

    try:
        body = await request.json()
    except Exception:
        return web.json_response(
            {"code": 1002, "msg": "PARSE_ERROR", "data": {"detail": "invalid JSON"}},
            status=400,
        )

    x = body.get("x")
    y = body.get("y")
    theta = body.get("theta", 0.0)

    if x is None or y is None:
        return web.json_response(
            {"code": 1002, "msg": "MISSING_PARAMS", "data": {"detail": "x and y are required"}},
            status=400,
        )

    with state_lock:
        if robot_state == "navigating":
            return web.json_response(
                {"code": 2006, "msg": "NAV_IN_PROGRESS", "data": {"current_target": target_point_id}},
                status=409,
            )
        if robot_state == "manual":
            cancel_watchdog()
            stop_robot()

        robot_state = "navigating"
        nav_state = "navigating"
        target_point_id = "custom"
        nav_cancelled = False

    point = {"x": float(x), "y": float(y), "theta": float(theta)}
    executor.submit(do_navigate, "custom", point)

    return web.json_response({
        "code": 0,
        "msg": "accepted",
        "data": {"nav_state": "navigating"},
    })

# ---------------------------------------------------------------------------
# 状态查询处理器
# ---------------------------------------------------------------------------

async def handle_status(request):
    with state_lock:
        r_state = robot_state
        n_state = nav_state
        t_point = target_point_id

    nav_info = {"state": n_state, "target_point_id": t_point}

    return web.json_response({
        "code": 0,
        "msg": "ok",
        "data": {
            "robot_state": r_state,
            "pose": dict(current_pose),
            "nav": nav_info,
            "battery": {"level": 100, "is_charging": False},
            "localization": {"valid": True, "map_id": "default"},
            "errors": [],
        },
    })


# ---------------------------------------------------------------------------
# 点位查询处理器
# ---------------------------------------------------------------------------

async def handle_points(request):
    points = []
    for pid, p in poi_map.items():
        points.append({
            "point_id": pid,
            "name": p["name"],
        })

    return web.json_response({
        "code": 0,
        "msg": "ok",
        "data": {
            "points": points,
        },
    })


# ---------------------------------------------------------------------------
# 相机订阅节点
# ---------------------------------------------------------------------------

class CameraNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("camera_subscriber")
        self.create_subscription(Image, "/camera/rgb",   self._rgb_cb,   10)
        self.create_subscription(Image, "/camera/depth", self._depth_cb, 10)

    def _rgb_cb(self, msg):
        global latest_rgb
        with camera_lock:
            latest_rgb = msg

    def _depth_cb(self, msg):
        global latest_depth
        with camera_lock:
            latest_depth = msg


# ---------------------------------------------------------------------------
# 图片清理
# ---------------------------------------------------------------------------

def _delete_image(image_id: str):
    meta = image_store.pop(image_id, None)
    if meta is None:
        return
    for key in ("rgb_path", "depth_path"):
        path = meta.get(key)
        if path and os.path.exists(path):
            try:
                os.remove(path)
            except OSError:
                pass


def _cleanup_by_count():
    if len(image_store) > MAX_CAPTURES:
        oldest = sorted(image_store.keys(), key=lambda k: image_store[k]["created_at"])
        for iid in oldest[:len(image_store) - MAX_CAPTURES]:
            _delete_image(iid)


def _cleanup_loop():
    while True:
        time.sleep(60)
        now = datetime.now()
        with image_store_lock:
            expired = [
                iid for iid, meta in image_store.items()
                if (now - datetime.fromisoformat(meta["created_at"])).total_seconds() > CAPTURE_TTL
            ]
            for iid in expired:
                _delete_image(iid)
                print(f"[CAM] 过期清理: {iid}")


# ---------------------------------------------------------------------------
# 相机处理器
# ---------------------------------------------------------------------------

async def handle_capture(request):
    try:
        body = await request.json() if request.content_length else {}
    except Exception:
        body = {}

    include_depth = body.get("include_depth", False)

    with camera_lock:
        rgb_msg   = latest_rgb
        depth_msg = latest_depth if include_depth else None

    if rgb_msg is None:
        return web.json_response(
            {"code": 3001, "msg": "INTERNAL_FAILED", "data": {"detail": "no rgb frame available"}},
            status=500,
        )
    if include_depth and depth_msg is None:
        return web.json_response(
            {"code": 3001, "msg": "INTERNAL_FAILED", "data": {"detail": "no depth frame available"}},
            status=500,
        )

    now = datetime.now()
    image_id = f"img_{now.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:4]}"
    rgb_path = os.path.join(CAPTURE_DIR, f"{image_id}_rgb.jpg")

    try:
        cv_rgb = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        cv2.imwrite(rgb_path, cv_rgb)
    except Exception as e:
        return web.json_response(
            {"code": 3001, "msg": "INTERNAL_FAILED", "data": {"detail": str(e)}},
            status=500,
        )

    depth_path = None
    if include_depth:
        depth_path = os.path.join(CAPTURE_DIR, f"{image_id}_depth.png")
        try:
            cv_depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            cv2.imwrite(depth_path, cv_depth)
        except Exception as e:
            os.remove(rgb_path)
            return web.json_response(
                {"code": 3001, "msg": "INTERNAL_FAILED", "data": {"detail": str(e)}},
                status=500,
            )

    created_at  = now.isoformat()
    expires_at  = (now + timedelta(seconds=CAPTURE_TTL)).isoformat()

    with image_store_lock:
        image_store[image_id] = {
            "created_at": created_at,
            "rgb_path":   rgb_path,
            "has_depth":  include_depth,
            "depth_path": depth_path,
        }
        _cleanup_by_count()

    print(f"[CAM] 抓拍: {image_id}")
    return web.json_response({
        "code": 0,
        "msg": "ok",
        "data": {
            "image_id":    image_id,
            "created_at":  created_at,
            "expires_at":  expires_at,
            "has_depth":   include_depth,
            "rgb": {
                "content_type": "image/jpeg",
                "download_url": f"/api/v1/camera/images/{image_id}/rgb",
            },
            "depth": {
                "content_type": "image/png",
                "download_url": f"/api/v1/camera/images/{image_id}/depth",
            } if include_depth else None,
        },
    })


async def handle_image_meta(request):
    image_id = request.match_info["image_id"]
    with image_store_lock:
        meta = image_store.get(image_id)

    if meta is None:
        return web.json_response(
            {"code": 4004, "msg": "IMAGE_NOT_FOUND", "data": {"image_id": image_id}},
            status=404,
        )

    created_at = meta["created_at"]
    expires_at = (datetime.fromisoformat(created_at) + timedelta(seconds=CAPTURE_TTL)).isoformat()

    return web.json_response({
        "code": 0,
        "msg": "ok",
        "data": {
            "image_id":   image_id,
            "created_at": created_at,
            "expires_at": expires_at,
            "has_depth":  meta["has_depth"],
            "rgb":   {"download_url": f"/api/v1/camera/images/{image_id}/rgb"},
            "depth": {"download_url": f"/api/v1/camera/images/{image_id}/depth"} if meta["has_depth"] else None,
        },
    })


async def handle_image_rgb(request):
    image_id = request.match_info["image_id"]
    with image_store_lock:
        meta = image_store.get(image_id)

    if meta is None or not os.path.exists(meta["rgb_path"]):
        return web.json_response(
            {"code": 4004, "msg": "IMAGE_NOT_FOUND", "data": {"image_id": image_id}},
            status=404,
        )
    return web.FileResponse(meta["rgb_path"], headers={"Content-Type": "image/jpeg"})


async def handle_image_depth(request):
    image_id = request.match_info["image_id"]
    with image_store_lock:
        meta = image_store.get(image_id)

    if meta is None or not meta["has_depth"] or not os.path.exists(meta["depth_path"]):
        return web.json_response(
            {"code": 4004, "msg": "IMAGE_NOT_FOUND", "data": {"image_id": image_id}},
            status=404,
        )
    return web.FileResponse(meta["depth_path"], headers={"Content-Type": "image/png"})


async def handle_image_delete(request):
    image_id = request.match_info["image_id"]
    with image_store_lock:
        exists = image_id in image_store
        if exists:
            _delete_image(image_id)

    print(f"[CAM] 删除: {image_id}")
    return web.json_response({"code": 0, "msg": "deleted", "data": {"image_id": image_id}})


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
    app.router.add_post("/api/v1/move/forward",  handle_move_forward)
    app.router.add_post("/api/v1/move/backward", handle_move_backward)
    app.router.add_post("/api/v1/move/left",     handle_move_left)
    app.router.add_post("/api/v1/move/right",    handle_move_right)
    app.router.add_post("/api/v1/stop",          handle_stop)
    app.router.add_post("/api/v1/navigate",      handle_navigate)
    app.router.add_post("/api/v1/navigate/pose",  handle_navigate_pose)
    app.router.add_get("/api/v1/status",         handle_status)
    app.router.add_get("/api/v1/points",                            handle_points)
    app.router.add_post("/api/v1/camera/capture",                  handle_capture)
    app.router.add_get("/api/v1/camera/images/{image_id}",         handle_image_meta)
    app.router.add_get("/api/v1/camera/images/{image_id}/rgb",     handle_image_rgb)
    app.router.add_get("/api/v1/camera/images/{image_id}/depth",   handle_image_depth)
    app.router.add_delete("/api/v1/camera/images/{image_id}",      handle_image_delete)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", 8080)
    await site.start()
    print("[HTTP] 监听 :8080  (move / stop / navigate / status / points / camera)")

    async with websockets.serve(ws_handler, "0.0.0.0", 8081):
        print("[WS]   监听 :8081")
        await asyncio.Future()


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def main():
    global navigator, cmd_vel_pub, poi_map, camera_node

    os.makedirs(CAPTURE_DIR, exist_ok=True)
    print(f"[CAM] 图片目录: {CAPTURE_DIR}")


    poi_map = load_poi_map()
    print(f"[POI] 加载点位: {list(poi_map.keys())}")

    rclpy.init()
    navigator = BasicNavigator()


    cmd_vel_pub = navigator.create_publisher(Twist, "/cmd_vel", 10)

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    initial_pose_pub = navigator.create_publisher(
        PoseWithCovarianceStamped, "/initialpose", qos
    )
    home = poi_map["home"]
    init_pose = create_pose(home["x"], home["y"], home["theta"])
    init_msg = PoseWithCovarianceStamped()
    init_msg.header.frame_id = "map"
    init_msg.header.stamp = Time().to_msg()  # time=0 让 AMCL 用最新可用变换，避免时间戳超前报错
    init_msg.pose.pose = init_pose.pose

    time.sleep(1.0)  # 等 TF 树稳定
    for _ in range(5):
        init_msg.header.stamp = Time().to_msg()  # 每次发布刷新时间戳
        initial_pose_pub.publish(init_msg)
        time.sleep(0.3)
    print("[INIT] 已发布初始位姿")

    navigator.waitUntilNav2Active()
    print("[INIT] Nav2 已激活，等待 AMCL 定位...")
    camera_node = CameraNode()
    cam_executor = SingleThreadedExecutor()
    cam_executor.add_node(camera_node)
    threading.Thread(target=cam_executor.spin, daemon=True).start()
    print("[CAM] 相机订阅节点启动")
    threading.Thread(target=_cleanup_loop, daemon=True).start()
    time.sleep(2)
    print("[INIT] 初始化完成，启动服务器")

    asyncio.run(main_async())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
