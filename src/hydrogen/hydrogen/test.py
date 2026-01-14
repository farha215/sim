#!/usr/bin/env python3

import pygame
import time
import json
import socket
import paho.mqtt.client as mqtt

# ================= CONFIGURATION =================
MQTT_BROKER = "localhost"
MQTT_TOPIC = "robot/control"
ROBOT_ID = 0

DEADZONE = 0.15
MAX_CMD = 255
PUBLISH_RATE = 50  # Hz (Increased from 10Hz for smoother control)
ROTATE_SPEED = 0.6 # 60% speed for rotation

# ================= HELPERS =================
def deadzone(v):
    return 0.0 if abs(v) < DEADZONE else v
    
def scale(v):
    return int(v * MAX_CMD)

def clamp(v):
    return max(-1.0, min(1.0, v))

def normalize_trigger(raw):
    # FIX: Handles controllers that rest at 0.0 OR -1.0
    if abs(raw) < 0.01: 
        return 0.0
    val = (raw + 1.0) / 2.0
    return max(0.0, min(1.0, val))

# ================= SETUP =================
client = mqtt.Client()
try:
    client.connect(MQTT_BROKER, 1883, 60)
    
    # LATENCY FIX: Disable Nagle's Algorithm
    if client.socket():
        client.socket().setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, True)
        
    client.loop_start()
    print("✅ MQTT Connected (TCP_NODELAY Enabled)")
except:
    print("⚠️ MQTT Failed (Running offline mode)")

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("❌ No Controller Found")
joy = pygame.joystick.Joystick(0)
joy.init()
print(f"🎮 Controller: {joy.get_name()}")

# ================= MAIN LOOP =================
last_time = time.time()

try:
    while True:
        pygame.event.pump()

        # 1. READ INPUTS
        # Axis 1 is usually Left Stick Y. We invert (-) so Up is Positive.
        ly = deadzone(-joy.get_axis(1)) 
        
        # Axis 2 = Left Trigger, Axis 5 = Right Trigger (Standard Xbox)
        lt = normalize_trigger(joy.get_axis(2))
        rt = normalize_trigger(joy.get_axis(5))

        left = 0.0
        right = 0.0
        mode = "STOP"

        # 2. CONTROL LOGIC (Priority Chain)
        
        # PRIORITY 1: LEFT TRIGGER -> CLOCKWISE
        if lt > 0.1:
            speed = lt * ROTATE_SPEED
            left = speed    # Left Forward
            right = -speed  # Right Back
            mode = "CLOCKWISE (LT)"

        # PRIORITY 2: RIGHT TRIGGER -> ANTI-CLOCKWISE
        elif rt > 0.1:
            speed = rt * ROTATE_SPEED
            left = -speed   # Left Back
            right = speed   # Right Forward
            mode = "ANTI-CLOCK (RT)"

        # PRIORITY 3: JOYSTICK -> FORWARD/BACKWARD ONLY
        elif abs(ly) > 0:
            left = ly
            right = ly
            mode = "LINEAR"

        # PRIORITY 4: STOP
        else:
            left = 0.0
            right = 0.0
            mode = "STOP"

        # 3. SEND COMMANDS
        now = time.time()
        if now - last_time >= 1.0 / PUBLISH_RATE:
            payload = {
                "id": ROBOT_ID,
                "m1": scale(left),
                "m2": scale(right),
                "m3": 0,
                "arm": 90, 
                "mag": 0
            }
            client.publish(MQTT_TOPIC, json.dumps(payload))
            last_time = now

            # Debug Print
            print(f"[{mode:^15}] LY:{ly:+.2f} LT:{lt:.2f} RT:{rt:.2f} -> L:{scale(left)} R:{scale(right)}")
        
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n🛑 Stopped")
finally:
    client.loop_stop()
    pygame.quit()