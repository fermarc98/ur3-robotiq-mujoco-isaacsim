"""Open the UR3 + Robotiq scene in Isaac Sim and start the simulation.

Run with Isaac Sim's python:
    ~/isaacsim/python.sh scripts/launch_isaacsim.py              # GUI, simulation playing
    ~/isaacsim/python.sh scripts/launch_isaacsim.py --paused     # GUI, press Play yourself
    ~/isaacsim/python.sh scripts/launch_isaacsim.py --headless --steps 300   # no GUI, run N frames, print robot state
(or simply: scripts/launch.sh isaacsim [args])
"""
import argparse
import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_SCENE = os.path.join(ROOT, "assets", "usd", "scenes", "isaacsim_scene_ur3.usd")

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--scene", default=DEFAULT_SCENE, help="USD scene to open (default: %(default)s)")
parser.add_argument("--headless", action="store_true", help="run without GUI")
parser.add_argument("--paused", action="store_true", help="open the scene but do not press Play")
parser.add_argument("--steps", type=int, default=0, help="run this many frames then exit (0 = until the window is closed)")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402  (must be created before any other isaacsim/omni import)

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage  # noqa: E402

if not os.path.isfile(args.scene):
    raise SystemExit(f"Scene not found: {args.scene}")

print(f"[launch_isaacsim] opening {args.scene}")
open_stage(usd_path=args.scene)
while is_stage_loading():
    simulation_app.update()

if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view

    # Same view as the reference screenshot: looking at the table from the window side.
    set_camera_view(eye=[-0.6, -0.9, 0.75], target=[-0.6, 0.5, 0.0])

robot = None
if not args.paused:
    app_utils.play()
    simulation_app.update()
    try:
        from isaacsim.core.experimental.prims import Articulation

        robot = Articulation("/World/ur3_robotiq2f_85")
        print(f"[launch_isaacsim] robot DOFs: {robot.dof_names}")
    except Exception as e:  # the scene still runs without the handle
        print(f"[launch_isaacsim] could not get robot articulation handle: {e}")

frame = 0
while simulation_app.is_running():
    simulation_app.update()
    frame += 1
    if args.steps and frame >= args.steps:
        break

if robot is not None and args.steps:
    print("[launch_isaacsim] robot joint positions after", frame, "frames:")
    for n, q in zip(robot.dof_names, robot.get_dof_positions().numpy().reshape(-1)):
        print(f"    {n:34s} {q: .5f}")

app_utils.stop()
simulation_app.close()
