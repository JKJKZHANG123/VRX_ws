#!/usr/bin/env python3
"""
数传智能通信节点 - Jetson Orin Nano 端
======================================
功能:
  1. 接收 PC 串口助手发来的命令文本, 解析后执行对应 ROS2 / 系统操作
  2. 主动推送系统状态 (位姿、节点状态、地图信息) 到 PC

使用:
  python3 serial_agent.py [--port /dev/ttyUSB1] [--baud 115200]

支持的命令 (PC端发送):
  start1    - 启动场景1: Point-LIO 无 GPS 建图
  start2    - 启动场景2: RGB 彩色建图 (无回环)
  start3    - 启动场景3: 合并展示 (白色+彩色)
  stop      - 停止所有建图节点
  status    - 查询当前运行状态 (哪些节点活着)
  pose      - 查询当前位姿 (x,y,z,yaw)
  mapsize   - 查询彩色地图体素数
  savepcd   - 触发 Point-LIO 保存 PCD 文件 (需建图中)
  help      - 显示命令列表

注意:
  - 数传带宽约 11 KB/s, 足够传状态文本, 无法实时传点云
  - 串口默认编码 gbk (与 Windows 串口助手兼容), 可用 --encoding utf-8 切换
  - 建图命令会在后台启动 ros2 launch, 不会阻塞串口通信
"""

import serial
import threading
import subprocess
import argparse
import sys
import time
import os
from datetime import datetime

# ── ROS2 可选导入 (用于查询位姿/话题) ──────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from scipy.spatial.transform import Rotation
    import numpy as np
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# ─── 配置 ──────────────────────────────────────────────────────────────────
DEFAULT_PORT     = "/dev/ttyUSB1"    # 数传模块端口 (雷达占 USB0, 数传通常 USB1)
DEFAULT_BAUD     = 115200
WORKSPACE        = os.path.expanduser("~/Desktop/lidar_project")
STATUS_INTERVAL  = 5.0               # 主动推送状态的间隔 (秒), 0 关闭

# 命令 → (描述, 动作)
COMMAND_HELP = {
    "start1":  "启动场景1: Point-LIO 无 GPS 建图",
    "start2":  "启动场景2: RGB 彩色建图 (无回环)",
    "start3":  "启动场景3: 合并展示 (白色+彩色地图)",
    "stop":    "停止所有建图节点 (ros2 node kill)",
    "status":  "查询当前运行节点列表",
    "pose":    "查询当前位姿 x,y,z,yaw (需建图中)",
    "mapsize": "查询彩色地图体素数",
    "savepcd": "停止建图并保存 PCD 地图 (会结束建图)",
    "help":    "显示本命令列表",
}

# ─── 全局状态 ──────────────────────────────────────────────────────────────
_launch_proc: subprocess.Popen | None = None   # 当前建图子进程
_latest_pose: dict | None = None               # 最新位姿缓存
_map_voxels: int = 0                           # 彩色地图体素数 (从 /loop_status 或估算)
_ros_node: object | None = None                # ROS2 节点 (可选)
_stop_event = threading.Event()
_ser: serial.Serial | None = None
_encoding = "gbk"


def ts() -> str:
    return datetime.now().strftime("%H:%M:%S")


def tx(msg: str) -> None:
    """向 PC 发送一行文本。"""
    global _ser, _encoding
    if _ser and _ser.is_open:
        try:
            line = (msg.rstrip("\r\n") + "\r\n").encode(_encoding, errors="replace")
            _ser.write(line)
        except Exception as e:
            print(f"[TX错误] {e}")
    print(f"[{ts()}] → {msg}")


# ─── 命令处理 ──────────────────────────────────────────────────────────────

def cmd_start(scene: int) -> None:
    """后台启动建图 launch 文件。"""
    global _launch_proc
    # 先停掉旧的
    cmd_stop(silent=True)

    scripts = {
        1: "mapping_no_gps.launch.py",
        2: "with_camera.launch.py loop_closure:=false",
        3: "with_camera.launch.py loop_closure:=false rviz_config:=demo_combined.rviz",
    }
    if scene not in scripts:
        tx(f"[错误] 未知场景: {scene}")
        return

    args = scripts[scene].split()
    launch_file = args[0]
    extra_args = args[1:]

    cmd = [
        "bash", "-c",
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {WORKSPACE}/install/setup.bash && "
        f"ros2 launch my_mapping_launcher {launch_file} "
        + " ".join(extra_args)
        + " 2>&1"
    ]

    try:
        _launch_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,     # 新进程组, stop 时可整组杀掉
        )
        tx(f"[OK] 场景{scene} 已在后台启动 (PID={_launch_proc.pid})")
        tx(f"     {COMMAND_HELP[f'start{scene}']}")
    except Exception as e:
        tx(f"[错误] 启动失败: {e}")


def cmd_stop(silent=False) -> None:
    """停止所有建图节点。用 SIGINT 优雅退出, 触发 Point-LIO 保存 PCD。"""
    global _launch_proc
    killed = False

    if _launch_proc is not None and _launch_proc.poll() is None:
        try:
            import signal
            # SIGINT (等同 Ctrl+C): ros2 launch 会转发给子节点,
            # laserMapping 收到后走正常退出流程, 若 pcd_save_en=true 则保存 PCD
            os.killpg(os.getpgid(_launch_proc.pid), signal.SIGINT)
            # PCD 写盘可能耗时, 给足时间
            _launch_proc.wait(timeout=15)
            killed = True
        except subprocess.TimeoutExpired:
            # 超时则强杀 (PCD 可能未保存)
            try:
                import signal
                os.killpg(os.getpgid(_launch_proc.pid), signal.SIGKILL)
            except Exception:
                pass
            killed = True
        except Exception:
            try:
                _launch_proc.kill()
            except Exception:
                pass
        _launch_proc = None

    if not silent:
        tx("[OK] 已发送停止信号" + (" (进程已终止)" if killed else " (无活动进程)"))


def cmd_status() -> None:
    """查询当前运行的 ROS2 节点。"""
    try:
        result = subprocess.run(
            ["bash", "-c",
             f"source /opt/ros/jazzy/setup.bash && "
             f"source {WORKSPACE}/install/setup.bash && "
             "ros2 node list 2>/dev/null"],
            capture_output=True, text=True, timeout=5,
        )
        nodes = [n.strip() for n in result.stdout.strip().splitlines() if n.strip()]
        if nodes:
            tx(f"[状态] 运行中节点 ({len(nodes)}):")
            for n in nodes:
                tx(f"  {n}")
        else:
            tx("[状态] 当前无 ROS2 节点运行")
    except subprocess.TimeoutExpired:
        tx("[状态] 查询超时 (ROS2 环境可能未初始化)")
    except Exception as e:
        tx(f"[错误] 状态查询失败: {e}")


def cmd_pose() -> None:
    """从 /aft_mapped_to_init 话题抓取一次位姿。"""
    try:
        result = subprocess.run(
            ["bash", "-c",
             f"source /opt/ros/jazzy/setup.bash && "
             f"source {WORKSPACE}/install/setup.bash && "
             "timeout 3 ros2 topic echo /aft_mapped_to_init --once 2>/dev/null "
             "| grep -A 12 'pose:' | head -13"],
            capture_output=True, text=True, timeout=6,
        )
        out = result.stdout.strip()
        if not out:
            tx("[位姿] 无数据 (Point-LIO 是否在运行?)")
            return

        # 从输出中解析 position.x/y/z 和 orientation.z/w
        vals = {}
        for line in out.splitlines():
            line = line.strip()
            for k in ("x:", "y:", "z:", "w:"):
                if line.startswith(k):
                    try:
                        vals.setdefault(k[0], float(line.split(":")[1].strip()))
                    except ValueError:
                        pass

        x = vals.get("x", 0)
        y = vals.get("y", 0)
        z = vals.get("z", 0)
        tx(f"[位姿] x={x:.3f}m  y={y:.3f}m  z={z:.3f}m")
    except Exception as e:
        tx(f"[错误] 位姿查询失败: {e}")


def cmd_mapsize() -> None:
    """估算彩色地图当前体素数 (通过话题消息宽度字段)。"""
    try:
        result = subprocess.run(
            ["bash", "-c",
             f"source /opt/ros/jazzy/setup.bash && "
             f"source {WORKSPACE}/install/setup.bash && "
             "timeout 4 ros2 topic echo /colored_map --once --field width 2>/dev/null | head -1"],
            capture_output=True, text=True, timeout=6,
        )
        width = result.stdout.strip()
        if width.isdigit():
            tx(f"[地图] 当前彩色地图: {int(width):,} 个体素点")
        else:
            tx("[地图] 无数据 (着色节点是否在运行?)")
    except Exception as e:
        tx(f"[错误] 地图查询失败: {e}")


def cmd_savepcd() -> None:
    """保存 PCD 地图。

    重要: Point-LIO 只在节点【正常退出(SIGINT)】时写盘 (见 laserMapping.cpp:1312),
    运行时无法在不停止建图的情况下保存。因此本命令会【停止建图并保存】。
    前提: unilidar_l1.yaml 里 pcd_save_en=true (已确认默认为 true)。
    """
    if _launch_proc is None or _launch_proc.poll() is not None:
        tx("[PCD] 当前没有由本节点启动的建图进程, 无法保存")
        tx("     (若建图是手动启动的, 请在那个终端按 Ctrl+C 保存)")
        return

    tx("[PCD] 正在停止建图并保存地图 (这会结束当前建图)...")
    pcd_path = f"{WORKSPACE}/src/point_lio_ros2/PCD/scans.pcd"

    # 记录保存前的文件修改时间, 用于确认是否真的写入了新文件
    old_mtime = os.path.getmtime(pcd_path) if os.path.exists(pcd_path) else 0

    # SIGINT 优雅停止, laserMapping 退出时写 PCD
    cmd_stop(silent=True)

    # 校验文件是否被更新
    time.sleep(1.0)
    if os.path.exists(pcd_path):
        new_mtime = os.path.getmtime(pcd_path)
        size_mb = os.path.getsize(pcd_path) / 1e6
        if new_mtime > old_mtime:
            tx(f"[PCD] ✓ 已保存: scans.pcd ({size_mb:.1f} MB)")
        else:
            tx(f"[PCD] ⚠ 文件存在但未更新 ({size_mb:.1f} MB), 可能点云为空或未累积")
    else:
        tx("[PCD] ✗ 未生成文件。检查 unilidar_l1.yaml 的 pcd_save_en 是否为 true")


def cmd_help() -> None:
    tx("=" * 36)
    tx("可用命令:")
    for k, v in COMMAND_HELP.items():
        tx(f"  {k:<10} {v}")
    tx("=" * 36)


# ─── 命令分发 ──────────────────────────────────────────────────────────────

def dispatch(raw: str) -> None:
    """解析并执行收到的命令。"""
    cmd = raw.strip().lower()
    tx(f"[执行] 收到命令: '{cmd}'")

    if cmd == "start1":
        cmd_start(1)
    elif cmd == "start2":
        cmd_start(2)
    elif cmd == "start3":
        cmd_start(3)
    elif cmd == "stop":
        cmd_stop()
    elif cmd == "status":
        cmd_status()
    elif cmd == "pose":
        cmd_pose()
    elif cmd == "mapsize":
        cmd_mapsize()
    elif cmd == "savepcd":
        cmd_savepcd()
    elif cmd == "help":
        cmd_help()
    else:
        tx(f"[未知命令] '{cmd}' — 发送 'help' 查看命令列表")


# ─── 接收线程 ──────────────────────────────────────────────────────────────

def receive_loop(ser: serial.Serial) -> None:
    print("[接收线程] 已启动, 等待 PC 命令...")
    while not _stop_event.is_set():
        try:
            line = ser.readline()
            if line:
                try:
                    text = line.decode(_encoding).rstrip("\r\n").strip()
                except UnicodeDecodeError:
                    text = line.decode("latin-1").rstrip("\r\n").strip()

                if text:
                    print(f"[{ts()}] ← PC: {text}")
                    dispatch(text)
        except serial.SerialException as e:
            print(f"[错误] 串口断开: {e}")
            _stop_event.set()
            break
        except Exception as e:
            if not _stop_event.is_set():
                print(f"[错误] 接收异常: {e}")


# ─── 主动推送状态线程 ──────────────────────────────────────────────────────

def status_push_loop() -> None:
    """定时向 PC 推送一行摘要状态。"""
    if STATUS_INTERVAL <= 0:
        return
    time.sleep(STATUS_INTERVAL)    # 等系统稳定后再开始推
    while not _stop_event.is_set():
        try:
            # 检查关键节点是否活着
            result = subprocess.run(
                ["bash", "-c",
                 f"source /opt/ros/jazzy/setup.bash && "
                 f"source {WORKSPACE}/install/setup.bash && "
                 "ros2 node list 2>/dev/null"],
                capture_output=True, text=True, timeout=3,
            )
            nodes = result.stdout.strip().splitlines()
            has_lio = any("laserMapping" in n for n in nodes)
            has_cam = any("camera" in n for n in nodes)
            has_color = any("pointcloud_colorizer" in n for n in nodes)

            parts = []
            parts.append("LIO:ON" if has_lio else "LIO:OFF")
            parts.append("CAM:ON" if has_cam else "CAM:OFF")
            parts.append("COLOR:ON" if has_color else "COLOR:OFF")

            tx("[心跳] " + "  ".join(parts))
        except Exception:
            pass

        _stop_event.wait(timeout=STATUS_INTERVAL)


# ─── 主程序 ───────────────────────────────────────────────────────────────

def main() -> None:
    global _ser, _encoding

    parser = argparse.ArgumentParser(description="Jetson 数传智能通信节点")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"数传串口 (默认: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"波特率 (默认: {DEFAULT_BAUD})")
    parser.add_argument("--encoding", default="gbk",
                        help="编码 gbk/utf-8 (默认: gbk)")
    parser.add_argument("--status-interval", type=float, default=STATUS_INTERVAL,
                        help=f"主动推送状态间隔秒 (0=关闭, 默认: {STATUS_INTERVAL})")
    parser.add_argument("--list-ports", action="store_true",
                        help="列出可用串口后退出")
    args = parser.parse_args()

    _encoding = args.encoding

    if args.list_ports:
        import glob
        ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        print("可用串口:", ports if ports else "未找到")
        sys.exit(0)

    print(f"[配置] 端口={args.port}  波特率={args.baud}  编码={args.encoding}  "
          f"心跳={args.status_interval}s")
    print("=" * 50)

    try:
        _ser = serial.Serial(
            port=args.port, baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
        )
    except serial.SerialException as e:
        print(f"[错误] 无法打开串口 {args.port}: {e}")
        print("提示: 检查数传模块是否接到 USB1, 或用 --list-ports 查看可用串口")
        sys.exit(1)

    print(f"[成功] 串口已打开: {_ser.name}")

    # 启动后发一条欢迎消息到 PC
    time.sleep(0.5)
    tx("=" * 36)
    tx("Jetson 数传节点已就绪")
    tx("发送 'help' 查看可用命令")
    tx("=" * 36)

    # 接收线程
    rx_thread = threading.Thread(target=receive_loop, args=(_ser,),
                                 daemon=True, name="RX")
    rx_thread.start()

    # 心跳推送线程
    if args.status_interval > 0:
        push_thread = threading.Thread(
            target=status_push_loop, daemon=True, name="PUSH")
        push_thread.start()

    # 主线程等待中断
    try:
        print("[主线程] 运行中, Ctrl+C 退出...")
        while not _stop_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[提示] 用户中断")

    _stop_event.set()
    cmd_stop(silent=True)
    _ser.close()
    print("[结束] 串口已关闭。")


if __name__ == "__main__":
    main()
