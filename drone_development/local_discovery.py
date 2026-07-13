from fastapi import FastAPI, WebSocket
import uvicorn

app = FastAPI()

@app.post("/register")
async def register(payload: dict):
    print(f"[REGISTER] {payload}")
    return {"status": "ok", "id": payload.get("id")}

@app.post("/heartbeat")
async def heartbeat(payload: dict):
    return {"status": "ok"}

@app.websocket("/stream/{drone_id}")
async def stream(ws: WebSocket, drone_id: str):
    await ws.accept()
    print(f"[WS] {drone_id} connected")
    try:
        while True:
            msg = await ws.receive_text()
            print(f"[WS<{drone_id}>] {msg}")
    except Exception as e:
        print(f"[WS] {drone_id} disconnected: {e}")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
