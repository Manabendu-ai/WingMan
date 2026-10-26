"""FastAPI app. Every route needs the access token (header X-Token or ?token=)."""
import asyncio
import hmac
from pathlib import Path

from fastapi import Depends, FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.security import APIKeyHeader
from pydantic import BaseModel, Field

from .manager import BadAnswer, Busy, MissionManager

STATIC = Path(__file__).resolve().parents[2] / "static"


class MissionBody(BaseModel):
    request: str = Field(min_length=1, max_length=500)


class ApproveBody(BaseModel):
    approve: bool


class AnswerBody(BaseModel):
    text: str = Field(min_length=1, max_length=300)


def create_app(manager: MissionManager, camera, mapsrc, token: str, docs: bool = False) -> FastAPI:
    app = FastAPI(title="WINGMAN dashboard", redoc_url=None,
                  docs_url="/docs" if docs else None,
                  openapi_url="/openapi.json" if docs else None)
    key_header = APIKeyHeader(name="X-Token", auto_error=False)   # shows an Authorize button in /docs

    def ok_token(t) -> bool:
        return bool(t) and hmac.compare_digest(str(t), token)

    def auth(request: Request, _k=Depends(key_header)):
        if not ok_token(request.headers.get("x-token") or request.query_params.get("token")):
            raise HTTPException(401, "missing or wrong token")

    guard = [Depends(auth)]

    @app.get("/api/health")
    def health():
        return {"ok": True}

    @app.get("/api/state", dependencies=guard)
    def state():
        return manager.snapshot()

    @app.get("/api/map", dependencies=guard)
    def get_map():
        return mapsrc.snapshot()

    @app.post("/api/mission", dependencies=guard)
    def mission(body: MissionBody):
        try:
            manager.start(body.request)
        except Busy as e:
            raise HTTPException(409, str(e))
        except ValueError as e:
            raise HTTPException(422, str(e))
        return {"started": True}

    @app.post("/api/approve", dependencies=guard)
    def approve(body: ApproveBody):
        try:
            manager.answer("approval", body.approve)
        except BadAnswer as e:
            raise HTTPException(409, str(e))
        return {"ok": True}

    @app.post("/api/answer", dependencies=guard)
    def answer(body: AnswerBody):
        try:
            manager.answer("clarification", body.text)
        except BadAnswer as e:
            raise HTTPException(409, str(e))
        return {"ok": True}

    @app.post("/api/abort", dependencies=guard)
    def abort():
        manager.abort()
        return {"ok": True}

    @app.get("/camera.mjpg", dependencies=guard)
    async def camera_stream():
        async def gen():
            while True:
                jpg = await asyncio.to_thread(camera.jpeg)
                yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                       + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                await asyncio.sleep(0.2)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        if not ok_token(sock.query_params.get("token")):
            await sock.close(code=1008)
            return
        await sock.accept()
        last_v = -1
        try:
            while True:
                msg = manager.snapshot()
                if mapsrc.version != last_v:       # send the map only when it changed
                    msg["map"], last_v = mapsrc.snapshot(), mapsrc.version
                await sock.send_json(msg)
                await asyncio.sleep(0.2)
        except (WebSocketDisconnect, RuntimeError):
            pass

    @app.get("/")
    def index():
        page = STATIC / "index.html"
        if page.is_file():
            return FileResponse(page)
        return {"service": "wingman-dashboard", "frontend": "not installed yet"}

    return app
