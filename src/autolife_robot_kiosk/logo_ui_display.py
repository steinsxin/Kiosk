import os
import time
from collections import deque

import cv2
import numpy as np


MAX_GIF_FRAMES = 200
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 1.5
THICKNESS = 3
SCREEN_WIDTH = 2480
SCREEN_HEIGHT = 1860
HEARTBEAT_INTERVAL = 5  # seconds
ICON_SIZE = (80, 80)


def load_gif_frames(path, max_frames=MAX_GIF_FRAMES):
    frames = []
    cap = cv2.VideoCapture(path)

    while len(frames) < max_frames:
        ret, frame = cap.read()
        if not ret:
            break
        frames.append(frame)

    cap.release()
    return frames


def preload_battery_icons(assets_dir):
    def load_icon(filename):
        path = os.path.join(assets_dir, filename)
        icon = cv2.imread(path)
        if icon is not None:
            icon = cv2.resize(icon, ICON_SIZE)
            icon = cv2.rotate(icon, cv2.ROTATE_90_CLOCKWISE)
        return icon

    return {
        "charging": load_icon("Battery_charging.png"),
        1: load_icon("Battery_1.png"),
        2: load_icon("Battery_2.png"),
        3: load_icon("Battery_3.png"),
        4: load_icon("Battery_4.png"),
    }


def get_battery_icon(icons, percent, charging):
    if charging:
        return icons["charging"]
    elif percent <= 25:
        return icons[1]
    elif percent <= 50:
        return icons[2]
    elif percent <= 75:
        return icons[3]
    else:
        return icons[4]


def render_battery_text(text, bg_color=(0, 0, 0), font_color=(253, 123, 20)):
    (text_width, text_height), baseline = cv2.getTextSize(
        text, FONT, FONT_SCALE, THICKNESS
    )
    text_img = np.zeros(
        (text_height + baseline, text_width, 3), dtype=np.uint8
    )
    text_img[:] = bg_color

    cv2.putText(
        text_img, text, (0, text_height),
        FONT, FONT_SCALE, font_color, THICKNESS, cv2.LINE_AA
    )

    return cv2.rotate(text_img, cv2.ROTATE_90_CLOCKWISE)


def display_ui(queue, assets_path, scale_factor=2.5, bg_color=(0, 0, 0)):

    gif_path = os.path.join(assets_path, "Logo.gif")
    frames = load_gif_frames(gif_path)

    if not frames:
        print(f"[ERROR] Failed to load frames from {gif_path}")
        return

    icons = preload_battery_icons(assets_path)

    cv2.namedWindow("Battery UI", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty(
        "Battery UI", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
    )

    latest_data = deque(maxlen=1)
    last_percent = None
    last_charging = None
    cached_text_img = None

    last_heartbeat_time = time.time()
    frame_index = 0

    while True:
        loop_start = time.time()

        while not queue.empty():
            latest_data.append(queue.get())

        if latest_data:
            battery_data = latest_data[-1]
            percent = battery_data.get("capacity_percentage", 100)
            current = battery_data.get("current", 0.0)
        else:
            percent = 100
            current = 0.0

        charging = current > 0
        battery_icon = get_battery_icon(icons, percent, charging)

        if battery_icon is None:
            print("[WARN] Failed to load battery icon")
            continue

        if percent != last_percent or charging != last_charging:
            battery_text = f"{percent}%"
            cached_text_img = render_battery_text(battery_text, bg_color)
            last_percent = percent
            last_charging = charging
            print(f"[INFO] Battery updated: {battery_text} Charging: {charging}")

        frame = frames[frame_index % len(frames)]
        frame_index += 1

        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        new_size = (
            int(frame.shape[1] * scale_factor),
            int(frame.shape[0] * scale_factor)
        )
        frame = cv2.resize(frame, new_size)

        background = np.full(
            (SCREEN_HEIGHT, SCREEN_WIDTH, 3), bg_color, dtype=np.uint8
        )
        x_offset = (SCREEN_WIDTH - new_size[0]) // 2
        y_offset = (SCREEN_HEIGHT - new_size[1]) // 2 + 50

        background[
            y_offset:y_offset + frame.shape[0],
            x_offset:x_offset + frame.shape[1]
        ] = frame

        icon_h, icon_w = battery_icon.shape[:2]
        icon_x = SCREEN_HEIGHT - icon_h - 250
        icon_y = SCREEN_WIDTH - icon_w - 250

        background[
            icon_x:icon_x + icon_h,
            icon_y:icon_y + icon_w
        ] = battery_icon

        if cached_text_img is not None:
            text_h, text_w = cached_text_img.shape[:2]
            text_x = icon_x + icon_h + 30
            text_y = icon_y + (icon_w - text_w) // 2

            background[
                text_x:text_x + text_h,
                text_y:text_y + text_w
            ] = cached_text_img

        cv2.imshow("Battery UI", background)

        if cv2.waitKey(50) & 0xFF == 27:
            print("[INFO] ESC pressed, exiting display_ui")
            break

        now = time.time()
        if now - last_heartbeat_time > HEARTBEAT_INTERVAL:
            print(
                f"[HEARTBEAT] Frame {frame_index} "
                f"running. Battery: {percent}%, Charging: {charging}"
            )
            last_heartbeat_time = now

        # Optional debug:
        # elapsed = time.time() - loop_start
        # print(f"[DEBUG] Frame {frame_index} processed in {elapsed:.3f}s")

    cv2.destroyAllWindows()
