"""
Run one prediction.

    python src/predict.py                              # uses examples/sample_input.json
    python src/predict.py path/to/your_input.json
"""
import json, sys, warnings
from pathlib import Path

warnings.filterwarnings("ignore")
sys.path.insert(0, str(Path(__file__).resolve().parent))
from speed_controller import SpeedController  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "examples" / "sample_input.json"
    data = json.load(open(path))
    ctrl = SpeedController()
    out = ctrl.step(data["state"], v_plan=data.get("v_plan", 1.0))
    conflict = out["r_hat"] >= ctrl.threshold

    print(f"TTC Q50 (median time peer clears):     {out['t_clear_peer_q50']} s")
    print(f"TTC Q90 (cautious time peer clears):   {out['t_clear_peer_q90']} s")
    print(f"Conflict probability:                  {out['r_hat']}  "
          f"(threshold {ctrl.threshold:.2f}) -> {'CONFLICT' if conflict else 'no conflict'}")
    print(f"Speed command v_cmd:                   {out['v_cmd']} m/s  (alpha={out['alpha']})")
    print(f"Action:                                {out['action_state']}")
    print(f"Fallback used:                         {bool(out['fallback_flag'])}  {out['reasons']}")


if __name__ == "__main__":
    main()
