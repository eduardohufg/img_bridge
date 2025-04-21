from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Set
import asyncio, cv2, base64

img_router = APIRouter()
active_connections: Set[WebSocket] = set()

# 1) Abre la cámara una vez al importar el módulo
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("No se pudo abrir /dev/video0")

async def send_image_to_websockets(frame_b64: str):
    to_remove = set()
    for ws in list(active_connections):
        try:
            await ws.send_text(frame_b64)
        except:
            to_remove.add(ws)
    active_connections.difference_update(to_remove)

@img_router.websocket("/img")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.add(websocket)
    print(f"Cliente conectado ({len(active_connections)})")

    try:
        while True:
            # 2) Reusa el mismo cap para todos
            ret, frame = cap.read()
            if not ret:
                break

            _, buf = cv2.imencode(".jpg", frame)
            b64 = base64.b64encode(buf).decode("utf-8")
            await send_image_to_websockets(b64)

            await asyncio.sleep(1/30)
    except WebSocketDisconnect:
        pass
    finally:
        active_connections.discard(websocket)
        print(f"Cliente desconectado ({len(active_connections)})")

# 3) En el shutdown del servidor, libera la cámara si quieres
#    (opcional, FastAPI no lo hace automáticamente)
