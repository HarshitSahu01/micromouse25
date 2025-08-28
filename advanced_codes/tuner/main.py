"""
tuner_ws_cli.py
- Connects as WebSocket client to the ESP32 WS server (ws://esp32.local/ws)
- Presents a REPL prompt "_> "
- On `run`: sends next PID to ESP and `start` command (50 cm default)
- Waits for streaming 'error' messages and final 'result' and then computes cost
- Uses skopt.Optimizer (Bayesian) to ask/tell
"""

import asyncio
import json
import math
import time
from skopt import Optimizer
from skopt.space import Real
import numpy as np
import websockets
import sys
import threading

# ---------- CONFIG ----------
ESP_WS_URI = "ws://192.168.137.232/ws"   # <- change to ws://<IP>/ws if mDNS doesn't resolve
TARGET_DISTANCE_CM = 50.0
# initial_points provided by you
initial_points = [
    (1.25, 0.001, 0.255),
    (0.7, 0.01, 0.4)
]

space = [
    Real(0.1, 5.0, name="Kp"),
    Real(1e-5, 1.0, name="Ki", prior="log-uniform"),
    Real(0.0, 3.0, name="Kd")
]

# cost weights (tune if you want different priorities)
W_RMS = 1.0
W_OVR = 5.0
W_STL = 0.5
W_UNS = 1000.0

# ---------- helper cost ----------
def compute_cost(metrics):
    rms = metrics.get("rms_error_mm", 0.0)
    ovr = metrics.get("overshoot_frac", 0.0)
    stl = metrics.get("settling_time_s", 0.0)
    uns = metrics.get("unstable", False)
    ovr_excess = max(0.0, ovr - 0.01)  # allow 1% overshoot free
    J = W_RMS * (rms ** 2) + W_OVR * (ovr_excess ** 2) + W_STL * stl + (W_UNS if uns else 0.0)
    return float(J)

# ---------- main tuner class ----------
class WebTuner:
    def __init__(self, uri):
        self.uri = uri
        self.ws = None
        self.loop = asyncio.get_event_loop()
        self.optimizer = Optimizer(space, random_state=42, acq_func="EI", acq_optimizer="sampling")
        # queue to receive final run results
        self.result_q = asyncio.Queue()
        # hold last received error messages for logging
        self.last_errors = []
        # keep list of runs
        self.runs = []
        self.best = None
        # seed queue
        self.seed_iter = iter(initial_points)

    async def connect(self):
        print(f"Connecting to {self.uri} ...")
        self.ws = await websockets.connect(self.uri)
        print("Connected websocket")
        # spawn receiver
        asyncio.ensure_future(self.receiver())

    async def receiver(self):
        try:
            async for msg in self.ws:
                try:
                    data = json.loads(msg)
                except Exception as e:
                    print("Bad JSON from ESP:", msg)
                    continue

                # handle types
                if data.get("type") == "error":
                    # streaming error sample
                    t = data.get("time_ms", 0)
                    err_ticks = data.get("error_ticks", 0)
                    avg_ticks = data.get("avg_ticks", 0)
                    self.last_errors.append((t, err_ticks, avg_ticks))
                    print(f"[stream] t={t}ms err_ticks={err_ticks} avg_ticks={avg_ticks}")
                elif data.get("type") == "result":
                    print("[result] received final metrics")
                    # push metrics onto result_q for the awaiting run
                    await self.result_q.put(data)
                else:
                    # generic telemetry from notifyClients: print small summary
                    if "debug" in data and data["debug"]:
                        print("[esp debug] ", data["debug"])
        except websockets.ConnectionClosed:
            print("Websocket closed")
        except Exception as e:
            print("Receiver exception:", e)

    async def send(self, payload: dict):
        if self.ws is None:
            raise RuntimeError("Websocket not connected")
        msg = json.dumps(payload)
        await self.ws.send(msg)

    async def run_once(self, target_distance=TARGET_DISTANCE_CM):
        """
        Send next PID triple to ESP and start run, then wait for result.
        """
        # pick candidate: use seed first, then optimizer.ask()
        try:
            x0 = next(self.seed_iter)
            candidate = list(x0)
            print(f"[seed] using seed candidate {candidate}")
        except StopIteration:
            candidate = self.optimizer.ask().tolist()
            print(f"[ask] optimizer suggested {candidate}")

        Kp, Ki, Kd = candidate
        # send PID update first
        await self.send({"Kp": float(Kp), "Ki": float(Ki), "Kd": float(Kd)})
        # small delay to ensure esp updates params
        await asyncio.sleep(0.05)
        # send start command
        await self.send({"start": float(target_distance)})

        # clear last errors
        self.last_errors = []

        # now wait for result object (blocks here until result arrives)
        print("Waiting for trial result from ESP ...")
        result = await self.result_q.get()  # will be the JSON final metrics
        # compute cost
        J = compute_cost(result)
        print(f"Run finished: J={J:.5f} metrics={result}")
        # feed into optimizer
        # if candidate came from seed (x0), we still call tell
        self.optimizer.tell(candidate, J)
        # record run
        self.runs.append({"x": candidate, "J": J, "metrics": result})
        # update best
        if self.best is None or J < self.best["J"]:
            self.best = {"J": J, "x": candidate, "metrics": result}
        return result

    async def stop_run(self):
        await self.send({"stop": True})

    def print_best(self):
        print("BEST SO FAR:")
        if self.best:
            print(f"  J={self.best['J']:.6f}  x={self.best['x']} metrics={self.best['metrics']}")
        else:
            print("  No runs yet.")

# ------------------ CLI loop ------------------
def run_cli(uri):
    tuner = WebTuner(uri)

    async def main_async():
        await tuner.connect()
        # REPL implemented by running blocking input in executor
        loop = asyncio.get_event_loop()
        print("\nType 'run' to execute next PID candidate, 'stop' to abort run, 'status' for best, 'exit' to quit.")
        while True:
            # blocking input in threadpool so event loop continues
            cmd = await loop.run_in_executor(None, lambda: input("_> ").strip())
            if cmd == "run":
                # launch run_once and await
                try:
                    result = await tuner.run_once()
                    print("Trial done. You may 'run' again.")
                except Exception as e:
                    print("Run failed:", e)
            elif cmd == "stop":
                try:
                    await tuner.stop_run()
                    print("Stop command sent.")
                except Exception as e:
                    print("Stop failed:", e)
            elif cmd == "status":
                tuner.print_best()
            elif cmd in ("quit", "exit"):
                print("Exiting...")
                await tuner.ws.close()
                return
            elif cmd == "":
                continue
            else:
                print("Unknown command. Use run / stop / status / exit.")

    try:
        asyncio.get_event_loop().run_until_complete(main_async())
    except KeyboardInterrupt:
        print("Interrupted, exiting.")

if __name__ == "__main__":
    run_cli(ESP_WS_URI)
