#!/usr/bin/env python3
"""
数传双向通信脚本 - Jetson Orin Nano 端
用途: 通过串口与PC端串口助手实现双向数据通信
使用: python3 serial_comm.py [--port /dev/ttyUSB0] [--baud 115200] [--encoding gbk]

编码说明:
  Windows串口助手默认用 GBK，Jetson终端用 UTF-8。
  脚本默认编码 gbk，与PC端保持一致；若PC端配置了UTF-8则用 --encoding utf-8。
"""

import serial
import threading
import argparse
import sys
import time
from datetime import datetime


# ─── 配置 ────────────────────────────────────────────────────────────────────

DEFAULT_PORT = "/dev/ttyUSB0"   # 数传模块串口，可按实际修改
DEFAULT_BAUD = 115200           # 波特率，需与PC端串口助手一致
TIMEOUT      = 1.0              # 串口读超时（秒）


# ─── 接收线程 ─────────────────────────────────────────────────────────────────

def receive_loop(ser: serial.Serial, stop_event: threading.Event, encoding: str) -> None:
    """持续读取串口数据并打印，直到 stop_event 被置位。"""
    print("[接收线程] 启动，等待数据...\n")
    while not stop_event.is_set():
        try:
            # 按行读取；超时后返回空bytes，继续循环
            line = ser.readline()
            if line:
                ts  = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                # 先用指定编码解码，失败则回退到latin-1（不会抛异常）
                try:
                    msg = line.decode(encoding).rstrip("\r\n")
                except UnicodeDecodeError:
                    msg = line.decode("latin-1").rstrip("\r\n") + "  [解码失败，原始hex: " + line.hex() + "]"
                # 打印时在行首清除当前输入提示，避免显示混乱
                print(f"\r[{ts}] ← 收到: {msg}          ")
                print(">>> ", end="", flush=True)  # 重绘输入提示符
        except serial.SerialException as e:
            print(f"\r[错误] 串口异常: {e}")
            stop_event.set()
            break
        except Exception as e:
            print(f"\r[错误] 读取异常: {e}")


# ─── 发送循环（主线程）────────────────────────────────────────────────────────

def send_loop(ser: serial.Serial, stop_event: threading.Event, encoding: str) -> None:
    """从终端读取用户输入并发送到串口。输入 'quit' 退出。"""
    print("输入要发送的内容，按 Enter 发送。输入 'quit' 退出。\n")
    while not stop_event.is_set():
        try:
            text = input(">>> ")
        except (EOFError, KeyboardInterrupt):
            print("\n[提示] 用户中断，退出...")
            stop_event.set()
            break

        if text.strip().lower() == "quit":
            print("[提示] 退出通信。")
            stop_event.set()
            break

        if not text:
            continue

        try:
            # Jetson终端是UTF-8，先encode到指定编码再发送
            # GBK模式: utf-8字符串 → gbk字节 → 串口 → PC串口助手GBK解码 ✓
            ser.write(text.encode(encoding) + b"\r\n")
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"[{ts}] → 已发送: {text}")
        except (serial.SerialException, UnicodeEncodeError) as e:
            print(f"[错误] 发送失败: {e}")
            if isinstance(e, serial.SerialException):
                stop_event.set()
                break


# ─── 主程序 ───────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Jetson 数传双向通信脚本")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"串口设备路径 (默认: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"波特率 (默认: {DEFAULT_BAUD})")
    parser.add_argument("--encoding", default="gbk",
                        help="串口字符编码 (默认: gbk，PC串口助手通常用gbk；若PC端用UTF-8则填utf-8)")
    parser.add_argument("--list-ports", action="store_true",
                        help="列出可用串口后退出")
    args = parser.parse_args()

    # 列出可用串口
    if args.list_ports:
        import glob
        ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") + \
                glob.glob("/dev/ttyTHS*")
        print("可用串口:", ports if ports else "未找到")
        sys.exit(0)

    print(f"[配置] 端口={args.port}  波特率={args.baud}  编码={args.encoding}")
    print("-" * 45)

    # 打开串口
    try:
        ser = serial.Serial(
            port     = args.port,
            baudrate = args.baud,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout  = TIMEOUT,
        )
    except serial.SerialException as e:
        print(f"[错误] 无法打开串口 {args.port}: {e}")
        print("提示: 检查设备是否连接，或尝试 --list-ports 查看可用串口")
        sys.exit(1)

    print(f"[成功] 串口已打开: {ser.name}\n")

    stop_event = threading.Event()

    # 启动接收线程（daemon=True 保证主线程退出时自动结束）
    rx_thread = threading.Thread(target=receive_loop,
                                 args=(ser, stop_event, args.encoding),
                                 daemon=True,
                                 name="RX-Thread")
    rx_thread.start()

    # 主线程负责发送
    send_loop(ser, stop_event, args.encoding)

    # 清理
    stop_event.set()
    rx_thread.join(timeout=2.0)
    ser.close()
    print("[结束] 串口已关闭。")


if __name__ == "__main__":
    main()
