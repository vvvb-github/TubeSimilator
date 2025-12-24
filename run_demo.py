from __future__ import annotations

import argparse
import yaml
from typing import List

from conveyor_sim.sim import ConveyorSystem
from conveyor_sim.scheduler import SimpleScheduler, BufferingScheduler
from conveyor_sim.viz_pygame import run_visualization_pygame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=str, default="example_config.yaml")
    ap.add_argument(
        "--scheduler",
        type=str,
        default="simple",
        choices=["simple", "buffering"],
        help="Which scheduling strategy to use.",
    )
    ap.add_argument("--max-ticks", type=int, default=500)
    ap.add_argument(
        "--viz",
        type=str,
        default="pygame",
        choices=["pygame", "mpl"],
        help="Visualization backend: pygame (recommended) or mpl (matplotlib).",
    )
    ap.add_argument("--subframes", type=int, default=8)
    ap.add_argument("--interval-ms", type=int, default=60)
    ap.add_argument(
        "--tick-ms",
        type=int,
        default=None,
        help="(pygame only) Real-time ms per simulation tick. Default: interval_ms * subframes.",
    )
    ap.add_argument(
        "--fps",
        type=int,
        default=60,
        help="(pygame only) Target FPS for rendering.",
    )
    ap.add_argument("--no-viz", action="store_true")
    args = ap.parse_args()

    system = ConveyorSystem.from_yaml(args.config)

    if args.scheduler == "simple":
        sched = SimpleScheduler()
    else:
        with open(args.config, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        plan = raw.get("advanced_scheduler") or {}
        if not plan:
            raise SystemExit(
                "Config is missing 'advanced_scheduler' section required for --scheduler buffering."
            )
        sched = BufferingScheduler(system=system, plan=plan)

    if args.no_viz:
        for _ in range(args.max_ticks):
            snap = system.snapshot()
            acts = sched.decide(snap)
            frame = system.step(acts)
            if frame.alarms:
                print("\n".join(frame.alarms))
        print("\n=== DONE ===")
        print(f"tick={system.tick}")
        print(f"completed_output={len(system.completed_output)} tubes: {system.completed_output[:20]}")
    else:
        if args.viz == "mpl":
            # Import lazily so pygame users don't need matplotlib installed.
            from conveyor_sim.viz import run_visualization

            run_visualization(
                system,
                sched,
                max_ticks=args.max_ticks,
                subframes=args.subframes,
                interval_ms=args.interval_ms,
            )
        else:
            tick_ms = args.tick_ms if args.tick_ms is not None else args.interval_ms * args.subframes
            run_visualization_pygame(
                system,
                sched,
                max_ticks=args.max_ticks,
                tick_ms=tick_ms,
                fps=args.fps,
                window_title=f"Conveyor Simulator (pygame) | scheduler={args.scheduler}",
            )


if __name__ == "__main__":
    main()
