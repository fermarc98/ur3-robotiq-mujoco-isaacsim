"""Render the table camera (RealSense D435-style colour stream, 1280x720) in either simulator.

    python scripts/render_camera.py --sim mujoco                  # any python with mujoco + pillow (EGL offscreen)
    ~/isaacsim/python.sh scripts/render_camera.py --sim isaacsim  # Isaac Sim 6.1, headless RTX

Writes camera_output/<sim>_rgb.png and camera_output/<sim>_depth.png (16-bit PNG, depth to the image plane
in millimetres, 0 = no hit) and prints the pinhole intrinsics.
The camera is defined in the assets, not here:
  MuJoCo     table/table_camera            (assets/mjcf/objects/welding_table/welding_table.xml)
  Isaac Sim  /World/welding_table/camera_link/table_camera
             (assets/usd/objects/welding_table/configuration/welding_table_sensor.usd; tilt set in the scene)
Both scenes are opened read-only and physics is not stepped: the images show the initial state of the scene.
"""
import argparse
import math
import os

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OUT_DIR = os.path.join(ROOT, "camera_output")
MJCF_SCENE = os.path.join(ROOT, "assets", "mjcf", "scenes", "mujoco_scene_ur3.xml")
USD_SCENE = os.path.join(ROOT, "assets", "usd", "scenes", "isaacsim_scene_ur3.usd")
MJCF_CAMERA = "table/table_camera"
USD_CAMERA = "/World/welding_table/camera_link/table_camera"
WIDTH, HEIGHT = 1280, 720


def save(sim, rgb, depth_m):
    from PIL import Image

    os.makedirs(OUT_DIR, exist_ok=True)
    rgb_path = os.path.join(OUT_DIR, f"{sim}_rgb.png")
    depth_path = os.path.join(OUT_DIR, f"{sim}_depth.png")
    Image.fromarray(np.ascontiguousarray(rgb[:, :, :3]).astype(np.uint8)).save(rgb_path)
    d = np.nan_to_num(np.asarray(depth_m, dtype=np.float64), nan=0.0, posinf=0.0, neginf=0.0)
    Image.fromarray(np.clip(np.round(d * 1000.0), 0, 65535).astype(np.uint16)).save(depth_path)
    for p in (rgb_path, depth_path):
        print("wrote", os.path.relpath(p, ROOT))


def print_intrinsics(fovy_deg):
    fy = (HEIGHT / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    print(f"intrinsics {WIDTH}x{HEIGHT}: fx = fy = {fy:.2f} px, cx = {WIDTH / 2:.1f}, cy = {HEIGHT / 2:.1f}, "
          f"FOV {math.degrees(2 * math.atan(WIDTH / 2 / fy)):.1f} x {fovy_deg:.1f} deg")


def render_mujoco():
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco

    m = mujoco.MjModel.from_xml_path(MJCF_SCENE)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cam = m.camera(MJCF_CAMERA)
    print(f"camera {MJCF_CAMERA}: world pos {np.round(d.cam_xpos[cam.id], 4)}")
    with mujoco.Renderer(m, HEIGHT, WIDTH) as r:
        r.update_scene(d, camera=cam.id)
        rgb = r.render()
        r.enable_depth_rendering()
        r.update_scene(d, camera=cam.id)
        depth = r.render()
    far = m.vis.map.zfar * m.stat.extent
    depth[depth >= far * 0.999] = 0.0  # nothing hit
    save("mujoco", rgb, depth)
    print_intrinsics(float(cam.fovy[0]))


def render_isaacsim(settle_frames):
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "width": WIDTH, "height": HEIGHT, "renderer": "RayTracedLighting"})
    try:
        import carb
        import omni.replicator.core as rep
        import omni.usd
        from pxr import UsdGeom

        ctx = omni.usd.get_context()
        ctx.open_stage(USD_SCENE)
        while ctx.get_stage_loading_status()[2] > 0:
            app.update()
        stage = ctx.get_stage()
        prim = stage.GetPrimAtPath(USD_CAMERA)
        if not prim.IsValid():
            raise RuntimeError(f"camera {USD_CAMERA} not found (is the welding table Sensor variant enabled?)")
        cam = UsdGeom.Camera(prim)
        pos = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
        print(f"camera {USD_CAMERA}: world pos {np.round(np.array(pos), 4)}")

        settings = carb.settings.get_settings()
        settings.set("/rtx/post/aa/op", 3)  # DLAA
        rep.orchestrator.set_capture_on_play(False)

        rp = rep.create.render_product(USD_CAMERA, (WIDTH, HEIGHT))
        rgb_a = rep.AnnotatorRegistry.get_annotator("rgb")
        depth_a = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        rgb_a.attach([rp])
        depth_a.attach([rp])
        for _ in range(settle_frames):  # let textures stream in and the RTX accumulation converge
            app.update()
        rep.orchestrator.step(rt_subframes=32)
        save("isaacsim", np.asarray(rgb_a.get_data()), np.asarray(depth_a.get_data()))
        v_ap, f = cam.GetVerticalApertureAttr().Get(), cam.GetFocalLengthAttr().Get()
        print_intrinsics(math.degrees(2 * math.atan(v_ap / 2 / f)))
        rgb_a.detach()
        depth_a.detach()
        rp.destroy()
    finally:
        app.close()  # the stage is never saved


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", required=True, choices=["mujoco", "isaacsim"])
    parser.add_argument("--settle-frames", type=int, default=400, help="Isaac Sim only (default: %(default)s)")
    args = parser.parse_args()
    if args.sim == "mujoco":
        render_mujoco()
    else:
        render_isaacsim(args.settle_frames)


if __name__ == "__main__":
    main()
