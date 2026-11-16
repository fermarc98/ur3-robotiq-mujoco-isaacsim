"""Open the UR3 + Robotiq scene in MuJoCo.

Run with any python that has mujoco >= 3.2 (e.g. the `mujoco` conda env):
    python scripts/launch_mujoco.py                 # interactive viewer (Control panel = sliders for every actuator)
    python scripts/launch_mujoco.py --headless --steps 1500   # no GUI, simulate N steps, print robot state
    python scripts/launch_mujoco.py --model assets/mjcf/robots/ur3_robotiq2f_85/ur3_robotiq2f_85.xml   # any MJCF
(or simply: scripts/launch.sh mujoco [args])

Viewer tips: double-click a body to select it, Ctrl+right-drag to push it; keys 2/3 toggle visual/collision
geom groups; Space pauses; Backspace resets. Gripper: robot/gripper/fingers_actuator 0 = open, 255 = closed.
"""
import argparse
import os
import time

import mujoco
import mujoco.viewer
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_SCENE = os.path.join(ROOT, "assets", "mjcf", "scenes", "mujoco_scene_ur3.xml")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", default=DEFAULT_SCENE, help="MJCF file to open (default: %(default)s)")
    parser.add_argument("--headless", action="store_true", help="run without GUI")
    parser.add_argument("--steps", type=int, default=1500, help="headless only: number of physics steps")
    args = parser.parse_args()

    model_path = os.path.abspath(args.model)
    if not os.path.isfile(model_path):
        raise SystemExit(f"Model not found: {model_path}")
    print(f"[launch_mujoco] MuJoCo {mujoco.__version__}, opening {model_path}")
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    print(f"[launch_mujoco] {m.nbody - 1} bodies, {m.njnt} joints, {m.nu} actuators, dt = {m.opt.timestep} s")

    if args.headless:
        t0 = time.time()
        for _ in range(args.steps):
            mujoco.mj_step(m, d)
        wall = time.time() - t0
        print(f"[launch_mujoco] simulated {d.time:.2f} s in {wall:.2f} s ({d.time / max(wall, 1e-9):.0f}x real time), "
              f"finite state: {bool(np.isfinite(d.qpos).all())}")
        for j in range(m.njnt):
            if m.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                print(f"    {m.joint(j).name:40s} {d.qpos[m.jnt_qposadr[j]]: .5f}")
        return

    # Full "simulate" UI; blocks until the window is closed. Press Space to pause/resume.
    mujoco.viewer.launch(m, d)


if __name__ == "__main__":
    main()
