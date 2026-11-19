#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
X11 窗口管理与查找辅助工具
支持通过标题关键字查找窗口、获取偶数对齐的几何度量 (X, Y, W, H)，并使用 libX11 强制将窗口提升置顶并设置焦点。
"""

import re
import time
import ctypes
import subprocess

def get_x11_windows():
    """解析 xwininfo -root -tree 输出的窗口列表."""
    try:
        res = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True, check=True)
    except Exception as e:
        print(f"[WindowManager] xwininfo 运行失败: {e}")
        return []

    lines = res.stdout.splitlines()
    windows = []
    # 格式示例: 0x600004 "Clash Party": ("mihomo-party" "mihomo-party")  1600x1200+1740+54  +1740+54
    # 或含负坐标: 0x80000a "gnome-shell": ...  1x1+-200+-200
    pattern = re.compile(r'^\s*(0x[0-9a-fA-F]+)\s+"([^"]*)":.*?(\d+)x(\d+)([+-]\d+)([+-]\d+)')
    
    for line in lines:
        m = pattern.search(line)
        if m:
            wid_hex, title, w, h, x, y = m.groups()
            wid = int(wid_hex, 16)
            w = int(w)
            h = int(h)
            x = int(x)
            y = int(y)
            windows.append({
                'id_hex': wid_hex,
                'id': wid,
                'title': title,
                'w': w,
                'h': h,
                'x': x,
                'y': y
            })
    return windows

def find_window(keyword):
    """根据关键字匹配窗口 (忽略大小写)."""
    wins = get_x11_windows()
    # 优先精确或包含匹配
    for w in wins:
        if keyword.lower() in w['title'].lower() and w['w'] > 100 and w['h'] > 100:
            return w
    return None

def raise_and_focus_window(wid):
    """通过 ctypes 调用 libX11 将指定窗口提升至最前端并激活输入焦点."""
    try:
        x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        x11.XSetInputFocus.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
        x11.XFlush.argtypes = [ctypes.c_void_p]
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]

        d = x11.XOpenDisplay(None)
        if d:
            x11.XRaiseWindow(d, ctypes.c_ulong(wid))
            # 2 = RevertToParent, 0 = CurrentTime
            x11.XSetInputFocus(d, ctypes.c_ulong(wid), 2, 0)
            x11.XFlush(d)
            x11.XCloseDisplay(d)
            time.sleep(0.3)
            return True
    except Exception as e:
        print(f"[WindowManager] raise_and_focus_window 失败: {e}")
    return False

def get_capture_geometry(win):
    """返回偶数对齐的 (X, Y, W, H)，满足 ffmpeg 视频抓取要求."""
    w = win['w'] - (win['w'] % 2)
    h = win['h'] - (win['h'] % 2)
    x = win['x']
    y = win['y']
    # 边界保护
    if x < 0:
        w += x
        x = 0
    if y < 0:
        h += y
        y = 0
    w = max(w - (w % 2), 64)
    h = max(h - (h % 2), 64)
    return x, y, w, h

if __name__ == '__main__':
    wins = get_x11_windows()
    print(f"找到 {len(wins)} 个窗口:")
    for w in wins:
        if w['w'] > 50 and w['h'] > 50:
            print(f"  {w['id_hex']} [{w['w']}x{w['h']}+{w['x']}+{w['y']}] \"{w['title']}\"")
