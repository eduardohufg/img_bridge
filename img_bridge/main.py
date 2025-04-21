# main.py
from fastapi import FastAPI, APIRouter
import uvicorn
import asyncio
from .app_globals import globals
from fastapi.middleware.cors import CORSMiddleware
from .ros2_bridge import start_ros2, stop_ros2
from .routes.ws_img import img_router
from contextlib import asynccontextmanager




@asynccontextmanager
async def lifespan(app: FastAPI):
    globals.loop = asyncio.get_running_loop()
    # 1) Arranca ROS2
    start_ros2()
    # 2) Registra los callbacks ROS2 que usan the_loop
    await asyncio.sleep(0.1)
  
    yield
    stop_ros2()

app = FastAPI(lifespan=lifespan)
api = APIRouter()


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
 #routes
api.include_router(img_router, prefix="/connection", tags=["connection"])


#includes
app.include_router(api, prefix= "/ws")


def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()