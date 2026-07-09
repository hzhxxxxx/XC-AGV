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
import rclpy.node
from rclpy.executors import SingleThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from aiohttp import web
import websockets

# ---------------------------------------------------------------------------
# 速度档位配置
# ---------------------------------------------------------------------------

SPEED_LINEAR = {"slow": 0.1, "normal": 0.2, "fast": 0.3}

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

# robot_state: idle | manual | navigating
robot_state = "idle"

cmd_vel_pub = None

# amount 定时器
amount_timer = None
amount_timer_lock = threading.Lock()

# 开环旋转控制（梯形加减速曲线 + 误差重试）
ROTATE_MAX_SPEED = 0.4        # 最大转速，rad/s
ROTATE_ACCEL = 1.0            # 角加速度，rad/s^2
ROTATE_CALIBRATION = 1.0        # 第一次转动的标定系数
ROTATE_RETRY_CALIBRATION = 0.6  # 第二次及以后修正转动的标定系数（小角度更容易过冲，调保守）
ROTATE_PERIOD = 0.05          # 速度曲线更新周期，秒
ROTATE_TOLERANCE = math.radians(2)  # 到位容差，超出则重试补齐
ROTATE_MAX_RETRIES = 3        # 最多重试次数
ROTATE_SETTLE_TIME = 0.3      # 转完等待位姿稳定的时间，秒
_rotate_stop_event = threading.Event()

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
# Amount 控制
# ---------------------------------------------------------------------------

def _amount_timeout():
    global robot_state
    print("[AMOUNT] 运动量到位，自动停车")
    stop_robot()
    with state_lock:
        if robot_state == "manual":
            robot_state = "idle"


def cancel_amount_timer():
    global amount_timer
    _rotate_stop_event.set()
    with amount_timer_lock:
        if amount_timer is not None:
            amount_timer.cancel()
            amount_timer = None


def _run_rotate_profile(delta: float, calibration: float):
    """执行一次开环梯形加减速旋转，转过给定的 delta（弧度，带符号）"""
    distance = abs(delta) * calibration
    direction = 1.0 if delta > 0 else -1.0

    if distance < 1e-4:
        return

    v_max = ROTATE_MAX_SPEED
    a = ROTATE_ACCEL
    d_ramp = v_max * v_max / a  # 加速+减速两段共需要走过的角度

    if distance >= d_ramp:
        t_acc = v_max / a
        t_cruise = (distance - d_ramp) / v_max
    else:
        v_max = math.sqrt(distance * a)  # 距离不够加速到最大速度，退化为三角形曲线
        t_acc = v_max / a
        t_cruise = 0.0

    t_total = 2 * t_acc + t_cruise
    print(f"[ROTATE]   执行 delta={math.degrees(delta):.1f}° v_max={v_max:.2f}rad/s t_total={t_total:.2f}s")

    start = time.monotonic()
    while not _rotate_stop_event.is_set():
        t = time.monotonic() - start
        if t >= t_total:
            break

        if t < t_acc:
            v = a * t
        elif t < t_acc + t_cruise:
            v = v_max
        else:
            v = v_max - a * (t - t_acc - t_cruise)

        publish_twist(0.0, direction * v)
        time.sleep(ROTATE_PERIOD)

    stop_robot()


def _do_rotate(target_yaw: float):
    """开环旋转 + 误差重试：每次转完读角度校验，不够再补一次"""
    global robot_state
    _rotate_stop_event.clear()
    try:
        for attempt in range(1, ROTATE_MAX_RETRIES + 1):
            if _rotate_stop_event.is_set():
                break

            with state_lock:
                current_yaw = current_pose["theta"]

            delta = (target_yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi

            if abs(delta) < ROTATE_TOLERANCE:
                print(f"[ROTATE] 到位 target={target_yaw:.2f} current={current_yaw:.2f} attempt={attempt}")
                break

            calibration = ROTATE_CALIBRATION if attempt == 1 else ROTATE_RETRY_CALIBRATION
            print(f"[ROTATE] 第{attempt}次 target={target_yaw:.2f} current={current_yaw:.2f} delta={math.degrees(delta):.1f}° calib={calibration}")
            _run_rotate_profile(delta, calibration)
            time.sleep(ROTATE_SETTLE_TIME)
        else:
            print(f"[ROTATE] 达到最大重试次数（{ROTATE_MAX_RETRIES}），未完全到位")
    finally:
        stop_robot()
        with state_lock:
            if robot_state == "manual":
                robot_state = "idle"
        print("[ROTATE] 旋转结束")


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
    global nav_state, nav_id, target_point_id, failed_point_id, nav_cancelled, robot_state

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
            robot_state = "idle"
            return

    result = navigator.getResult()

    with state_lock:
        nav_id = None
        target_point_id = None
        if result == TaskResult.SUCCEEDED:
            nav_state = "arrived"
            failed_point_id = None
            robot_state = "idle"
        else:
            nav_state = "failed"
            failed_point_id = point_id
            robot_state = "failed"

    new_state = "arrived" if result == TaskResult.SUCCEEDED else "failed"
    print(f"[NAV] {point_id} -> {new_state}")
    push_nav_status(new_state, nid, point_id)

    # arrived 是瞬间事件，推完立刻回到 idle
    if new_state == "arrived":
        with state_lock:
            nav_state = "idle"


# ---------------------------------------------------------------------------
# 手动控制处理器
# ---------------------------------------------------------------------------

async def handle_move(request, direction: str):
    global robot_state

    try:
        body = await request.json()
    except Exception:
        return web.json_response(
            {"code": 1002, "msg": "PARSE_ERROR", "data": {"detail": "invalid JSON"}},
            status=400,
        )

    amount = body.get("amount")
    if amount is None:
        return web.json_response(
            {"code": 1002, "msg": "MISSING_AMOUNT", "data": {"detail": "amount is required"}},
            status=400,
        )

    with state_lock:
        if robot_state == "navigating":
            return web.json_response(
                {"code": 2006, "msg": "NAV_IN_PROGRESS", "data": {"current_target": target_point_id}},
                status=409,
            )
        robot_state = "manual"

    amount = float(amount)
    cancel_amount_timer()

    if direction in ("forward", "backward"):
        speed = SPEED_LINEAR["normal"]
        duration = amount / speed
        linear_x = speed if direction == "forward" else -speed
        print(f"[MOVE] {direction} amount={amount} duration={duration:.2f}s")

        publish_twist(linear_x, 0.0)

        t = threading.Timer(duration, _amount_timeout)
        with amount_timer_lock:
            amount_timer = t
        t.start()
    else:
        # amount 为绝对目标角度（度），开环旋转（梯形加减速曲线）
        target_yaw = math.radians(amount)

        threading.Thread(target=_do_rotate, args=(target_yaw,), daemon=True).start()

    msg_map = {
        "forward": "moving_forward",
        "backward": "moving_backward",
        "rotate": "rotating",
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


async def handle_move_rotate(request):
    return await handle_move(request, "rotate")


# ---------------------------------------------------------------------------
# HTTP 处理器
# ---------------------------------------------------------------------------

async def handle_navigate(request):
    global nav_state, nav_id, target_point_id, nav_cancelled, robot_state

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
            cancel_amount_timer()
            stop_robot()

        nid = f"NAV-{uuid.uuid4().hex[:6].upper()}"
        nav_state = "navigating"
        robot_state = "navigating"
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
    global nav_state, nav_id, target_point_id, nav_cancelled, robot_state

    cancel_amount_timer()
    stop_robot()

    with state_lock:
        if nav_state == "navigating":
            nid = nav_id
            point_id = target_point_id
            nav_state = "idle"
            nav_id = None
            target_point_id = None
            nav_cancelled = True
            cancel_event.set()
            push_nav_status("idle", nid, point_id)
            print(f"[STOP] 导航已取消: {point_id}")

        robot_state = "idle"

    return web.json_response({"code": 0, "msg": "stopped"})


async def handle_status(request):
    with state_lock:
        state = nav_state
        r_state = robot_state
        t_point = target_point_id
        f_point = failed_point_id
        pose = dict(current_pose)

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
            "robot_state": r_state,
            "pose": pose,
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
    app.router.add_post("/api/v1/move/forward",  handle_move_forward)
    app.router.add_post("/api/v1/move/backward", handle_move_backward)
    app.router.add_post("/api/v1/move/rotate",   handle_move_rotate)
    app.router.add_post("/api/v1/stop",          handle_stop)
    app.router.add_post("/api/v1/navigate",      handle_navigate)
    app.router.add_get("/api/v1/status",         handle_status)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", 8080)
    await site.start()
    print("[HTTP] 监听 :8080  (forward / backward / rotate / stop / navigate / status)")

    async with websockets.serve(ws_handler, "0.0.0.0", 8081):
        print("[WS]   监听 :8081")
        await asyncio.Future()


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def _amcl_pose_cb(msg):
    global current_pose
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    siny = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    with state_lock:
        current_pose["x"] = round(p.x, 3)
        current_pose["y"] = round(p.y, 3)
        current_pose["theta"] = round(math.atan2(siny, cosy), 4)


def _odom_cb(msg):
    """高频里程计回调，用于精确旋转控制"""
    global current_pose
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    siny = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    with state_lock:
        # 仅更新 theta（odom 的 xy 有漂移，不用）
        current_pose["theta"] = round(math.atan2(siny, cosy), 4)


def main():
    global navigator, cmd_vel_pub, poi_map

    poi_map = load_poi_map()
    print(f"[POI] 加载点位: {list(poi_map.keys())}")

    rclpy.init()
    navigator = BasicNavigator()
    cmd_vel_pub = navigator.create_publisher(Twist, "/cmd_vel", 10)

    amcl_node = rclpy.node.Node("amcl_pose_listener")
    amcl_node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", _amcl_pose_cb, 10)
    odom_node = rclpy.node.Node("odom_listener")
    odom_node.create_subscription(Odometry, "/odom", _odom_cb, 10)
    pose_executor = SingleThreadedExecutor()
    pose_executor.add_node(amcl_node)
    pose_executor.add_node(odom_node)
    threading.Thread(target=pose_executor.spin, daemon=True).start()
    print("[INIT] AMCL + Odom 位姿订阅已启动")

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    initial_pose_pub = navigator.create_publisher(
        PoseWithCovarianceStamped, "/initialpose", qos
    )
    home = poi_map["home"]
    init_pose = create_pose(home["x"], home["y"], home["theta"])
    init_msg = PoseWithCovarianceStamped()
    init_msg.header.frame_id = "map"
    init_msg.header.stamp = Time().to_msg()
    init_msg.pose.pose = init_pose.pose

    time.sleep(1.0)
    for _ in range(5):
        init_msg.header.stamp = Time().to_msg()
        initial_pose_pub.publish(init_msg)
        time.sleep(0.3)
    print("[INIT] 已发布初始位姿")

    navigator.waitUntilNav2Active()
    print("[INIT] Nav2 已激活，等待 AMCL 定位...")
    time.sleep(2)
    print("[INIT] 初始化完成，启动服务器")

    asyncio.run(main_async())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
