#!/usr/bin/env python3
import asyncio
import os
import threading
import time
import base64
import cv2
import numpy as np
import signal
import sys
import traceback
import re
from typing import Optional, Tuple
from contextlib import asynccontextmanager

# FastAPI imports
from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse, RedirectResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist            # <-- NEW
from cv_bridge import CvBridge

# Gemini imports
from google import genai
from google.genai import types

# MCP imports (for future MCP server integration)
import subprocess

import logging

# Configure logging with more detailed format
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# Request models
class CommandRequest(BaseModel):
    text: str

class CameraBuffer(Node):
    """ROS2 node that maintains the latest camera frame in memory"""
    def __init__(self):
        super().__init__('camera_buffer')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.frame_count = 0

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('Camera Buffer Node Started - Subscribing to /camera/image_raw')

    def image_callback(self, msg):
        """Callback to store the latest camera frame"""
        try:
            with self.frame_lock:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.latest_frame = cv_image
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    self.get_logger().info(f'Camera frames received: {self.frame_count}')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def get_latest_frame(self) -> Optional[np.ndarray]:
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def has_frame(self) -> bool:
        with self.frame_lock:
            return self.latest_frame is not None

class RobotController:
    """Main robot controller with Gemini integration"""
    def __init__(self):
        # Initialize Gemini client
        self.gemini_client = genai.Client(api_key=os.environ.get("GEMINI_API_KEY"))
        self.model_id = "gemini-robotics-er-1.5-preview"

        # Thread safety
        self.command_lock = threading.Lock()

        # Initialize ROS2
        rclpy.init()
        self.camera_buffer = CameraBuffer()

        # --- NEW: /cmd_vel publisher and safety limits
        self.cmd_vel_pub = self.camera_buffer.create_publisher(Twist, '/cmd_vel', 10)
        self.max_linear = 0.5     # m/s (match your motor node)
        self.max_angular = 2.0    # rad/s (match your motor node)

        # Start ROS2 spinning in background thread
        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_thread.start()

        # MCP server process
        self.mcp_server_process: Optional[subprocess.Popen] = None

        # System prompt for Gemini
        self.system_prompt = """
You are controlling a ROS2 differential drive robot car via MCP tools.

ROBOT SETUP:
- Two-wheel differential drive with motor controller
- Camera providing live view of robot's environment
- You receive BOTH the current camera image AND text command simultaneously

AVAILABLE MCP TOOLS:
1. publish_once(topic, msg_type, msg) - Send single ROS message
2. get_topics() - List available ROS topics  
3. call_service(service, type, args) - Call ROS services

ROBOT CONTROL:
- Motor control: publish_once('/cmd_vel', 'geometry_msgs/Twist', message)
- Message format: {linear: {x: speed_mps}, angular: {z: rotation_rps}}

MOVEMENT EXAMPLES:
- Forward: {linear: {x: 0.5}, angular: {z: 0.0}}
- Backward: {linear: {x: -0.5}, angular: {z: 0.0}}
- Turn left: {linear: {x: 0.0}, angular: {z: 0.5}}
- Turn right: {linear: {x: 0.0}, angular: {z: -0.5}}
- Turn left while moving: {linear: {x: 0.3}, angular: {z: 0.3}}
- Stop: {linear: {x: 0.0}, angular: {z: 0.0}}

COMMAND PROCESSING:
1. Analyze the provided camera image to understand the environment
2. Interpret the user's text command in context of what you see
3. Execute appropriate movement using publish_once('/cmd_vel', 'geometry_msgs/Twist', {})
4. Explain what you see in the image and what action you're taking

IMPORTANT:
- Each command is INDEPENDENT - no memory of previous commands
- Execute ONE immediate action per command
- Always explain your visual reasoning
- Consider safety - avoid obstacles you see in the image
"""

        # Start MCP server (optional - will log warning if not available)
        self.start_mcp_server()
        logger.info("RobotController initialized")

    def _spin_ros(self):
        try:
            rclpy.spin(self.camera_buffer)
        except Exception as e:
            logger.error(f"ROS spinning error: {e}")

    def start_mcp_server(self):
        try:
            env = os.environ.copy()
            env['ROSBRIDGE_IP'] = '127.0.0.1'
            env['ROSBRIDGE_PORT'] = '9090'
            try:
                self.mcp_server_process = subprocess.Popen([
                    sys.executable, "-c",
                    "import ros_mcp_server.server; ros_mcp_server.server.main()"
                ], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                logger.info("ROS-MCP Server started")
                time.sleep(2)
            except Exception as e:
                logger.warning(f"MCP server not available: {e}")
                logger.warning("Motor control via MCP tools will not work. Install ros-mcp-server or use direct ROS2 publishing.")
                self.mcp_server_process = None
        except Exception as e:
            logger.warning(f"Failed to start MCP server: {e}")
            self.mcp_server_process = None

    def _frame_to_base64(self, frame: np.ndarray) -> str:
        height, width = frame.shape[:2]
        if width > 640:
            new_width = 640
            new_height = int(height * (new_width / width))
            frame = cv2.resize(frame, (new_width, new_height))
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return base64.b64encode(buffer).decode('utf-8')

    # -------- NEW: helper to publish /cmd_vel
    def _publish_cmd_vel(self, linear_x: float, angular_z: float) -> Tuple[float, float]:
        # clamp to safe limits
        lx = max(-self.max_linear, min(self.max_linear, float(linear_x)))
        az = max(-self.max_angular, min(self.max_angular, float(angular_z)))
        msg = Twist()
        msg.linear.x = lx
        msg.angular.z = az
        self.cmd_vel_pub.publish(msg)
        logger.info(f"PUBLISHED /cmd_vel -> linear.x={lx:.3f} m/s, angular.z={az:.3f} rad/s")
        return lx, az

    # -------- NEW: parse Gemini text like ... {linear:{x:0.1}, angular:{z:0.0}} ...
    def _parse_and_publish_from_text(self, text: str) -> Optional[Tuple[float, float]]:
        """
        Extract linear x and angular z from Gemini's response and publish.
        Returns (lx, az) if published, else None.
        """
        # 1) Strip code fences for consistency
        cleaned = re.sub(r"```[\s\S]*?```", lambda m: m.group(0).replace("\n", " "), text)

        # 2) Tolerant regex for {linear:{x:0.1}, angular:{z:0.0}} with optional spaces/quotes
        pattern = re.compile(
            r"linear\s*:\s*{[^}]*x\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)"
            r"[^}]*}\s*,\s*angular\s*:\s*{[^}]*z\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)",
            re.IGNORECASE
        )
        m = pattern.search(cleaned)
        if not m:
            # fallback: try to find x: and z: anywhere (very permissive)
            mx = re.search(r"\bx\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)", cleaned)
            mz = re.search(r"\bz\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)", cleaned)
            if not (mx and mz):
                logger.warning("Could not find linear/angular values in Gemini response.")
                return None
            lx_val = float(mx.group(1))
            az_val = float(mz.group(1))
        else:
            lx_val = float(m.group(1))
            az_val = float(m.group(2))

        # Publish
        return self._publish_cmd_vel(lx_val, az_val)

    async def execute_command(self, command_text: str) -> dict:
        """Execute a robot command with the latest camera image"""
        with self.command_lock:
            try:
                latest_frame = self.camera_buffer.get_latest_frame()
                if latest_frame is None:
                    return {
                        "success": False,
                        "command": command_text,
                        "error": "No camera image available. Check camera connection.",
                        "timestamp": time.time()
                    }

                image_b64 = self._frame_to_base64(latest_frame)

                logger.info("=" * 60)
                logger.info("GEMINI API REQUEST")
                logger.info("=" * 60)
                logger.info(f"Command: {command_text}")
                logger.info(f"Model: {self.model_id}")
                logger.info(f"Image size: {latest_frame.shape[1]}x{latest_frame.shape[0]}")
                logger.info(f"Image data size: {len(image_b64)} bytes (base64)")

                user_command_text = f"{self.system_prompt}\n\nUSER COMMAND: {command_text}"
                parts = [
                    types.Part.from_text(text=user_command_text),
                    types.Part.from_bytes(data=base64.b64decode(image_b64), mime_type='image/jpeg')
                ]
                contents = [types.Content(role="user", parts=parts)]

                config = types.GenerateContentConfig(
                    temperature=0.1,
                    thinking_config=types.ThinkingConfig(thinking_budget=2000),
                    media_resolution="MEDIA_RESOLUTION_MEDIUM",
                    image_config=types.ImageConfig(image_size="1K"),
                )

                logger.info("Calling Gemini API...")
                logger.info("-" * 60)

                # Streaming call (await first, then iterate)
                response_text = ""
                chunk_count = 0
                stream = await self.gemini_client.aio.models.generate_content_stream(
                    model=self.model_id,
                    contents=contents,
                    config=config,
                )
                async for chunk in stream:
                    if chunk.text:
                        response_text += chunk.text
                        chunk_count += 1
                        if chunk_count <= 5:
                            logger.debug(f"Gemini chunk #{chunk_count}: {chunk.text[:100]}...")

                logger.info("-" * 60)
                logger.info("GEMINI API RESPONSE")
                logger.info("=" * 60)
                logger.info(f"Total chunks received: {chunk_count}")
                logger.info(f"Response length: {len(response_text)} characters")
                for i, line in enumerate(response_text.split('\n'), 1):
                    logger.info(f"{i:3d} | {line}")
                logger.info("=" * 60)

                cleaned_response = response_text.strip()
                logger.info(f"Command '{command_text}' completed successfully")
                logger.info(f"Response preview: {cleaned_response[:100]}..." if len(cleaned_response) > 100 else f"Response: {cleaned_response}")

                # ---- NEW: parse and publish to /cmd_vel
                published = self._parse_and_publish_from_text(cleaned_response)
                published_linear = None
                published_angular = None
                if published:
                    published_linear, published_angular = published

                return {
                    "success": True,
                    "command": command_text,
                    "response": cleaned_response,
                    "has_camera_image": True,
                    "image_size": f"{latest_frame.shape[1]}x{latest_frame.shape[0]}",
                    "response_length": len(cleaned_response),
                    "published_cmd_vel": bool(published),
                    "published_linear_x": published_linear,
                    "published_angular_z": published_angular,
                    "timestamp": time.time()
                }

            except Exception as e:
                logger.error("=" * 60)
                logger.error("GEMINI API ERROR")
                logger.error("=" * 60)
                logger.error(f"Command: {command_text}")
                logger.error(f"Error type: {type(e).__name__}")
                logger.error(f"Error message: {str(e)}")
                logger.error("Traceback:")
                logger.error(traceback.format_exc())
                logger.error("=" * 60)
                return {
                    "success": False,
                    "command": command_text,
                    "error": str(e),
                    "error_type": type(e).__name__,
                    "timestamp": time.time()
                }

    def get_latest_image_b64(self) -> Optional[str]:
        frame = self.camera_buffer.get_latest_frame()
        if frame is not None:
            return self._frame_to_base64(frame)
        return None

    def cleanup(self):
        logger.info("Starting cleanup...")
        if self.mcp_server_process:
            self.mcp_server_process.terminate()
            self.mcp_server_process.wait(timeout=5)
            logger.info("MCP server terminated")
        if hasattr(self, 'camera_buffer'):
            self.camera_buffer.destroy_node()
        rclpy.shutdown()
        logger.info("Cleanup completed")

# Global controller instance
controller = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    global controller
    logger.info("Starting Robot Control System...")
    controller = RobotController()

    startup_timeout = 10
    for i in range(startup_timeout):
        if controller.camera_buffer.has_frame():
            logger.info("Camera is ready!")
            break
        logger.info(f"Waiting for camera... ({i+1}/{startup_timeout})")
        await asyncio.sleep(1)

    yield

    if controller:
        controller.cleanup()

# FastAPI application
app = FastAPI(
    title="Optimized Robot Control API",
    description="ROS2 robot control with direct camera integration and Gemini AI",
    version="2.0.0",
    lifespan=lifespan
)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.post("/command")
async def process_command(request: CommandRequest):
    if not controller:
        raise HTTPException(status_code=503, detail="Robot controller not initialized")
    result = await controller.execute_command(request.text)
    return JSONResponse(content=result)

@app.get("/camera/latest")
async def get_latest_camera_image():
    if not controller:
        raise HTTPException(status_code=503, detail="Robot controller not initialized")
    image_b64 = controller.get_latest_image_b64()
    if image_b64:
        return {"success": True, "image": f"data:image/jpeg;base64,{image_b64}", "timestamp": time.time()}
    else:
        raise HTTPException(status_code=404, detail="No camera image available")

@app.get("/status")
async def get_robot_status():
    if not controller:
        return {"error": "Robot controller not initialized"}
    has_camera = controller.camera_buffer.has_frame()
    frame_count = controller.camera_buffer.frame_count
    return {
        "robot_online": True,
        "camera_active": has_camera,
        "total_frames_received": frame_count,
        "mcp_server_running": controller.mcp_server_process is not None if controller.mcp_server_process else False,
        "gemini_model": controller.model_id,
        "optimization": "direct_camera_integration",
        "endpoints": {
            "command": "POST /command - Send robot commands",
            "camera": "GET /camera/latest - Get latest image",
            "video": "GET /video - Live video stream",
            "stop": "GET /stop - Emergency stop"
        },
        "video_stream_url": "http://localhost:8080/stream?topic=/camera/image_raw"
    }

@app.get("/video")
async def video_stream_redirect():
    return RedirectResponse(url="http://localhost:8080/stream?topic=/camera/image_raw")

@app.get("/stop")
async def emergency_stop():
    if not controller:
        raise HTTPException(status_code=503, detail="Robot controller not initialized")
    # Prefer direct stop publish to be immediate & robust:
    controller._publish_cmd_vel(0.0, 0.0)
    # Also ask Gemini (optional, keeps logs consistent)
    result = await controller.execute_command("stop immediately and do not move")
    return JSONResponse(content=result)

@app.get("/")
async def root():
    return {
        "message": "Optimized Robot Control API with Direct Camera Integration",
        "version": "2.0.0",
        "description": "ROS2 robot controlled by Gemini AI with real-time camera vision",
        "key_features": [
            "Direct camera image integration (no MCP subscribe needed)",
            "Stateless command processing",
            "Real-time interruption capability",
            "Live video streaming",
            "RESTful API for remote control"
        ],
        "usage": {
            "send_command": "POST /command with JSON body: {\"text\": \"move forward\"}",
            "view_camera": "GET /camera/latest for single image or GET /video for stream",
            "check_status": "GET /status for system information",
            "emergency_stop": "GET /stop for immediate halt"
        }
    }

def main():
    if not os.environ.get("GEMINI_API_KEY"):
        logger.error("GEMINI_API_KEY environment variable not set!")
        logger.error("Please set it with: export GEMINI_API_KEY='your_api_key_here'")
        sys.exit(1)

    def signal_handler(signum, frame):
        logger.info(f"Received signal {signum}, shutting down...")
        if controller:
            controller.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logger.info("Starting Optimized Robot Control Server...")
    logger.info("Server will be available at: http://0.0.0.0:8000")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info", access_log=True)

if __name__ == "__main__":
    main()
