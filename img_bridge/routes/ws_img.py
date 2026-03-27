"""
Camera WebSocket router — dynamic multi-camera with quality control.
Auto-crops stereo cameras (ZED2i side-by-side) to left eye only.

Frontend sends:
  { "type": "get_cameras" }
  { "type": "config", "camera_id": 0, "quality": 40 }

Backend sends:
  { "type": "cameras", "data": [{"id": 0, "label": "Camera 0"}, ...] }
  { "type": "frame", "data": "<base64 jpeg>" }
"""

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import asyncio
import cv2
import base64
import json
import time

img_router = APIRouter()

# ---------------------------------------------------------------
# Camera detection & management
# ---------------------------------------------------------------

MAX_CAMERA_INDEX = 20

CAMERA_LABELS: dict[int, str] = {
    # 4: "ZED2i",
    # 6: "Front Cam",
}

# Aspect ratio threshold: if width/height > this, it's a stereo camera
# Normal 16:9 = 1.78, normal 4:3 = 1.33
# ZED side-by-side is ~3.56 (double 16:9) or ~2.67 (double 4:3)
STEREO_ASPECT_THRESHOLD = 2.5

cameras: dict[int, cv2.VideoCapture] = {}

_cached_camera_list: list[dict] = []
_cache_time: float = 0
CACHE_TTL = 5.0


def detect_cameras(force: bool = False) -> list[dict]:
    global _cached_camera_list, _cache_time

    now = time.time()
    if not force and (now - _cache_time) < CACHE_TTL and _cached_camera_list:
        return _cached_camera_list

    found = []

    # Add cameras already open (don't touch them)
    for cam_id, cap in list(cameras.items()):
        if cap.isOpened():
            label = CAMERA_LABELS.get(cam_id, f"Camera {cam_id}")
            found.append({"id": cam_id, "label": label})

    already_open = set(cameras.keys())

    # Probe indices NOT already open
    for i in range(MAX_CAMERA_INDEX):
        if i in already_open:
            continue
        try:
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
            if cap.isOpened():
                ret, _ = cap.read()
                cap.release()
                if ret:
                    label = CAMERA_LABELS.get(i, f"Camera {i}")
                    found.append({"id": i, "label": label})
        except Exception:
            pass

    found.sort(key=lambda c: c["id"])
    _cached_camera_list = found
    _cache_time = now
    print(f"📷 Detected {len(found)} cameras: {[c['id'] for c in found]}")
    return found


def get_camera(camera_id: int) -> cv2.VideoCapture | None:
    if camera_id in cameras:
        cap = cameras[camera_id]
        if cap.isOpened():
            return cap
        cap.release()
        del cameras[camera_id]

    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    if cap.isOpened():
        cameras[camera_id] = cap
        print(f"📷 Opened camera {camera_id}")
        return cap
    return None


def release_unused_cameras(active_ids: set[int]):
    to_release = [cid for cid in cameras if cid not in active_ids]
    for cid in to_release:
        cameras[cid].release()
        del cameras[cid]
        print(f"📷 Released camera {cid}")


def crop_stereo(frame):
    """
    If the frame is ultrawide (aspect ratio > threshold),
    crop to the left half (left eye of stereo camera).
    """
    h, w = frame.shape[:2]
    if w > 0 and h > 0 and (w / h) > STEREO_ASPECT_THRESHOLD:
        return frame[:, :w // 2]
    return frame


# Detect on startup
detect_cameras(force=True)


# ---------------------------------------------------------------
# Client state
# ---------------------------------------------------------------

class ClientState:
    def __init__(self, ws: WebSocket):
        self.ws = ws
        self.camera_id: int = _cached_camera_list[0]["id"] if _cached_camera_list else 0
        self.quality: int = 40


active_clients: dict[WebSocket, ClientState] = {}

# ---------------------------------------------------------------
# Streaming loop
# ---------------------------------------------------------------

TARGET_FPS = 30
streaming_task: asyncio.Task | None = None


async def stream_loop():
    while True:
        if not active_clients:
            await asyncio.sleep(0.1)
            continue

        camera_groups: dict[int, list[ClientState]] = {}
        for client in list(active_clients.values()):
            cid = client.camera_id
            if cid not in camera_groups:
                camera_groups[cid] = []
            camera_groups[cid].append(client)

        release_unused_cameras(set(camera_groups.keys()))

        for camera_id, clients in camera_groups.items():
            cap = get_camera(camera_id)
            if cap is None:
                continue

            ret, frame = cap.read()
            if not ret:
                continue

            # Auto-crop stereo cameras
            frame = crop_stereo(frame)

            quality_groups: dict[int, list[ClientState]] = {}
            for c in clients:
                if c.quality not in quality_groups:
                    quality_groups[c.quality] = []
                quality_groups[c.quality].append(c)

            for quality_val, qclients in quality_groups.items():
                encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), quality_val]
                _, buf = cv2.imencode(".jpg", frame, encode_params)
                b64 = base64.b64encode(buf).decode("utf-8")
                msg = json.dumps({"type": "frame", "data": b64})

                to_remove = []
                for client in qclients:
                    try:
                        await client.ws.send_text(msg)
                    except Exception:
                        to_remove.append(client.ws)
                for ws in to_remove:
                    active_clients.pop(ws, None)

        await asyncio.sleep(1 / TARGET_FPS)


def ensure_streaming():
    global streaming_task
    if streaming_task is None or streaming_task.done():
        streaming_task = asyncio.create_task(stream_loop())


# ---------------------------------------------------------------
# WebSocket endpoint
# ---------------------------------------------------------------

@img_router.websocket("/camera")
async def camera_websocket(websocket: WebSocket):
    await websocket.accept()
    client = ClientState(websocket)
    active_clients[websocket] = client
    ensure_streaming()
    print(f"📷 Client connected ({len(active_clients)} total)")

    cam_list = detect_cameras(force=False)
    await websocket.send_text(json.dumps({"type": "cameras", "data": cam_list}))

    if cam_list:
        client.camera_id = cam_list[0]["id"]

    try:
        while True:
            raw = await websocket.receive_text()
            try:
                msg = json.loads(raw)

                if msg.get("type") == "get_cameras":
                    cam_list = detect_cameras(force=True)
                    await websocket.send_text(json.dumps({"type": "cameras", "data": cam_list}))

                elif msg.get("type") == "config":
                    if "camera_id" in msg:
                        client.camera_id = int(msg["camera_id"])
                    if "quality" in msg:
                        client.quality = max(5, min(100, int(msg["quality"])))

            except (json.JSONDecodeError, ValueError):
                pass
    except WebSocketDisconnect:
        pass
    finally:
        active_clients.pop(websocket, None)
        active_ids = {c.camera_id for c in active_clients.values()}
        release_unused_cameras(active_ids)
        print(f"📷 Client disconnected ({len(active_clients)} remaining)")