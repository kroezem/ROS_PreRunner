# Copyright 2026 matti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""FastAPI application and same-origin read-only WebSocket endpoint."""

from contextlib import asynccontextmanager
from pathlib import Path
from typing import AsyncIterator

from fastapi import FastAPI
from fastapi import WebSocket
from fastapi import WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from runner_paddock.client_stream import ClientHub
from runner_paddock.client_stream import StateStreamer
from runner_paddock.ros_runtime import RosRuntime
from runner_paddock.state_cache import StateCache
import uvicorn


STATIC_DIRECTORY = Path(__file__).resolve().parent / 'static'


def create_app(
    *, cache: StateCache | None = None, runtime: RosRuntime | None = None
) -> FastAPI:
    """Build one web application around an injectable ROS lifecycle."""
    state_cache = cache if cache is not None else StateCache()
    ros_runtime = runtime if runtime is not None else RosRuntime(state_cache)
    hub = ClientHub()
    streamer = StateStreamer(state_cache, hub)

    @asynccontextmanager
    async def lifespan(_app: FastAPI) -> AsyncIterator[None]:
        ros_runtime.start()
        try:
            streamer.start()
            yield
        finally:
            await streamer.stop()
            ros_runtime.stop()

    app = FastAPI(title='Runner Paddock', lifespan=lifespan)
    app.state.cache = state_cache
    app.state.hub = hub
    app.state.ros_runtime = ros_runtime
    app.mount('/static', StaticFiles(directory=STATIC_DIRECTORY), name='static')

    @app.get('/', include_in_schema=False)
    async def index() -> FileResponse:
        return FileResponse(STATIC_DIRECTORY / 'index.html')

    @app.websocket('/ws')
    async def websocket_state(websocket: WebSocket) -> None:
        await websocket.accept()
        client = hub.register()
        try:
            while True:
                await websocket.send_text(await client.next_frame())
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            hub.unregister(client)

    return app


def main() -> None:
    """Run Paddock with the supported single-worker process model."""
    uvicorn.run(
        'runner_paddock.web_app:create_app',
        factory=True,
        host='0.0.0.0',
        port=8000,
        workers=1,
    )
