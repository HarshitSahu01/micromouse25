# pip install scikit-optimize numpy

import time
import json
import numpy as np
from pathlib import Path
from skopt import gp_minimize
from skopt.space import Real
from skopt.utils import use_named_args
from skopt.callbacks import CheckpointSaver

# -----------------------------
# Config
# -----------------------------
RANDOM_SEED = 42
RESULTS_DIR = Path("./bo_pid_results")
RESULTS_DIR.mkdir(exist_ok=True)

# Search space (note Ki log-scale)
space = [
    Real(0.1, 5.0,       name="Kp"),                   # linear
    Real(1e-5, 1.0,      name="Ki", prior="log-uniform"),
    Real(0.0, 3.0,       name="Kd"),                   # linear
]

# Weights for objective
W_RMS = 1.0
W_OVR = 5.0         # penalize overshoot beyond threshold
W_STL = 0.5
W_UNS = 1000.0      # large penalty for unsafe/unstable

# Constraints/targets
OVERSHOOT_ALLOW = 0.01   # 1% overshoot allowed 'for free'
SETTLING_TOL_MM = 2.0    # +/- 2 mm band for "settled"

# Replicates per candidate to reduce noise
N_REP = 2

# Early stopping
MIN_IMPROVEMENT = 1e-3
PATIENCE = 6


# -----------------------------
# Hooks you will implement later
# -----------------------------
def run_robot_trial(kp, ki, kd):
    """
    Run *one* physical trial on the robot with PID gains (kp, ki, kd).
    Return a dict with performance metrics. This will be replaced later
    with your WebSocket round-trip. For now, this is a STUB or SIM.

    Return dict keys (all required):
        {
          "rms_error_mm": float,
          "overshoot_frac": float,  # overshoot as fraction of target distance
          "settling_time_s": float,
          "unstable": bool,         # True if oscillation/timeout
        }
    """
    # --- EXAMPLE: replace with real call ---
    # Example "sim": penalize too high Kp or Kd; Ki small boost.
    # This is just to make the code runnable; remove/replace later.
    target = 180.0  # mm, one cell
    np.random.seed()  # different noise per call
    base_error = abs(1.2 - kp) + 0.2*abs(0.05 - np.log10(ki+1e-9)) + 0.6*abs(0.8 - kd)
    rms_error_mm = max(0.5, 8.0 * base_error) + np.random.normal(0, 0.2)
    overshoot_frac = max(0.0, 0.2 * (kp - 1.5)) + np.random.normal(0, 0.01)
    settling_time_s = max(0.2, 1.0 + 0.6 * abs(kd - 0.8)) + np.random.normal(0, 0.05)
    unstable = (kp > 4.5 or kd > 2.8 or overshoot_frac > 0.25)

    return {
        "rms_error_mm": float(max(0.0, rms_error_mm)),
        "overshoot_frac": float(max(0.0, overshoot_frac)),
        "settling_time_s": float(max(0.0, settling_time_s)),
        "unstable": bool(unstable),
    }


# -----------------------------
# Cost function (lower is better)
# -----------------------------
def compute_cost(metrics):
    rms = metrics["rms_error_mm"]
    ovr = metrics["overshoot_frac"]
    stl = metrics["settling_time_s"]
    uns = metrics["unstable"]

    # Penalize overshoot only above allowed threshold
    ovr_excess = max(0.0, ovr - OVERSHOOT_ALLOW)

    J = (
        W_RMS * (rms ** 2) +
        W_OVR * (ovr_excess ** 2) +
        W_STL * stl +
        (W_UNS if uns else 0.0)
    )
    return float(J)


# -----------------------------
# Objective wrapper for skopt
# -----------------------------
best_so_far = {"J": float("inf"), "params": None, "metrics": None}
no_improve_count = 0

@use_named_args(space)
def objective(**params):
    global best_so_far, no_improve_count

    kp, ki, kd = params["Kp"], params["Ki"], params["Kd"]

    # Replicated runs for noise-robust estimate
    Js, Ms = [], []
    for _ in range(N_REP):
        metrics = run_robot_trial(kp, ki, kd)
        J = compute_cost(metrics)
        Js.append(J)
        Ms.append(metrics)

        # Safety: if unstable, break early (don’t do more reps)
        if metrics["unstable"]:
            break

    J_mean = float(np.mean(Js))
    # Track best
    if J_mean + MIN_IMPROVEMENT < best_so_far["J"]:
        best_so_far = {
            "J": J_mean,
            "params": dict(params),
            "metrics": Ms[int(np.argmin(Js))]
        }
        no_improve_count = 0
    else:
        no_improve_count += 1

    print(f"[BO] Kp={kp:.4f} Ki={ki:.6f} Kd={kd:.4f}  ->  J={J_mean:.5f}  "
          f"(best {best_so_far['J']:.5f})  unstable={Ms[-1]['unstable']}")
    return J_mean


# -----------------------------
# Seeds (good initial guesses)
# -----------------------------
# Add your Ziegler–Nichols/manual seeds here
initial_points = [
    (1.25, 0.001, 0.255),
    (0.7, 0.01, 0.4)
]


# -----------------------------
# Run BO
# -----------------------------
def run_bayesopt(
    n_calls=30,
    n_initial_points=6,
    acq_func="EI",     # EI or LCB or PI
    xi=0.01,           # EI exploration (bigger -> more explore)
    kappa=1.96,        # LCB param (ignored for EI)
):
    # Checkpointing
    checkpoint = CheckpointSaver(str(RESULTS_DIR / "checkpoint.pkl"), compress=9)

    # Combine explicit seeds with random initial points
    x0 = initial_points
    # y0 = [objective(Kp=kp, Ki=ki, Kd=kd) for (kp, ki, kd) in x0]
    y0 = [objective([kp, ki, kd]) for (kp, ki, kd) in x0]

    res = gp_minimize(
        func=objective,
        dimensions=space,
        n_calls=n_calls,
        n_initial_points=n_initial_points,
        x0=x0,
        y0=y0,
        acq_func=acq_func,
        acq_optimizer="sampling",     # robust on rough landscapes
        random_state=RANDOM_SEED,
        noise="gaussian",
        n_restarts_optimizer=5,
        kappa=kappa,
        xi=xi,
        callback=[checkpoint],
    )

    # Save best result
    out = {
        "best_J": best_so_far["J"],
        "best_params": best_so_far["params"],
        "best_metrics": best_so_far["metrics"],
        "all_res": {
            "x": res.x,
            "fun": res.fun
        }
    }
    (RESULTS_DIR / "best.json").write_text(json.dumps(out, indent=2))
    print("\n=== BEST ===")
    print(json.dumps(out, indent=2))

    return out


if __name__ == "__main__":
    t0 = time.time()
    result = run_bayesopt(n_calls=28, n_initial_points=6, acq_func="EI", xi=0.02)
    print(f"\nTime: {time.time()-t0:.1f}s")
