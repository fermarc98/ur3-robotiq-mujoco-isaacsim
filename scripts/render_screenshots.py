"""Render the README screenshots of both scenes from the same camera pose (1920x1080 PNG).

    python scripts/render_screenshots.py --sim mujoco              # any python with mujoco + pillow (EGL offscreen)
    ~/isaacsim/python.sh scripts/render_screenshots.py --sim isaacsim   # Isaac Sim 6.1, headless RTX

Writes docs/media/mujoco_scene.png and docs/media/isaacsim_scene.png.
Both scenes are opened read-only: the MuJoCo render settings are applied in memory, and the Isaac Sim
camera is created on the session layer of a stage that is never saved. Physics is not stepped, so the
images show the initial state of the scene.
"""
import argparse
import math
import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OUT_DIR = os.path.join(ROOT, "docs", "media")
MJCF_SCENE = os.path.join(ROOT, "assets", "mjcf", "scenes", "mujoco_scene_ur3.xml")
USD_SCENE = os.path.join(ROOT, "assets", "usd", "scenes", "isaacsim_scene_ur3.usd")

WIDTH, HEIGHT = 1920, 1080
# Shared camera, in MuJoCo free-camera terms (degrees, metres, world frame).
LOOKAT = (-0.43, 0.68, 0.13)
DISTANCE = 1.2
AZIMUTH = 55.0
ELEVATION = -30.0
FOVY = 45.0  # MuJoCo default vertical field of view


def camera_eye():
    """World position of the camera for the shared view (same convention as mjv_ free camera)."""
    az, el = math.radians(AZIMUTH), math.radians(ELEVATION)
    fwd = (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
    return tuple(LOOKAT[i] - DISTANCE * fwd[i] for i in range(3))


def render_mujoco(out_path):
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    from PIL import Image

    spec = mujoco.MjSpec.from_file(MJCF_SCENE)
    # Render-only settings (in memory, the XML is not modified).
    spec.visual.global_.offwidth = WIDTH
    spec.visual.global_.offheight = HEIGHT
    spec.visual.global_.fovy = FOVY
    spec.visual.quality.shadowsize = 8192
    spec.visual.quality.offsamples = 8
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
    spec.visual.headlight.diffuse = [0.35, 0.35, 0.35]
    spec.visual.headlight.specular = [0.05, 0.05, 0.05]
    # Key light from the window side, casting the main shadows.
    spec.worldbody.add_light(pos=[-1.4, -0.9, 2.4], dir=[0.45, 0.6, -1.0], diffuse=[0.55, 0.55, 0.53],
                             specular=[0.15, 0.15, 0.15], castshadow=True, cutoff=60, exponent=2)
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    cam = mujoco.MjvCamera()
    cam.lookat[:] = LOOKAT
    cam.distance = DISTANCE
    cam.azimuth = AZIMUTH
    cam.elevation = ELEVATION
    with mujoco.Renderer(m, HEIGHT, WIDTH) as r:
        r.update_scene(d, cam)
        Image.fromarray(r.render()).save(out_path)


def render_isaacsim(out_path, settle_frames):
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "width": WIDTH, "height": HEIGHT, "renderer": "RayTracedLighting"})
    try:
        import carb
        import numpy as np
        import omni.replicator.core as rep
        import omni.usd
        from PIL import Image
        from pxr import Gf, Sdf, Usd, UsdGeom

        ctx = omni.usd.get_context()
        ctx.open_stage(USD_SCENE)
        while ctx.get_stage_loading_status()[2] > 0:
            app.update()
        stage = ctx.get_stage()

        settings = carb.settings.get_settings()
        settings.set("/rtx/post/aa/op", 3)  # DLAA
        settings.set("/rtx/pathtracing/maxSamplesPerLaunch", 1)
        rep.orchestrator.set_capture_on_play(False)

        # Camera on the session layer: the scene file itself is never edited (and never saved).
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            cam = UsdGeom.Camera.Define(stage, Sdf.Path("/RenderCam"))
            h_ap = 20.955
            v_ap = h_ap * HEIGHT / WIDTH
            cam.CreateHorizontalApertureAttr(h_ap)
            cam.CreateVerticalApertureAttr(v_ap)
            cam.CreateFocalLengthAttr((v_ap / 2.0) / math.tan(math.radians(FOVY) / 2.0))
            cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
            view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*camera_eye()), Gf.Vec3d(*LOOKAT), Gf.Vec3d(0, 0, 1))
            UsdGeom.Xformable(cam.GetPrim()).AddTransformOp().Set(view.GetInverse())

        rp = rep.create.render_product("/RenderCam", (WIDTH, HEIGHT))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([rp])
        for _ in range(settle_frames):  # let textures stream in and the RTX accumulation converge
            app.update()
        rep.orchestrator.step(rt_subframes=32)
        img = np.asarray(rgb.get_data())[:, :, :3]
        Image.fromarray(img).save(out_path)
        rgb.detach()
        rp.destroy()
    finally:
        app.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", required=True, choices=["mujoco", "isaacsim"])
    parser.add_argument("--settle-frames", type=int, default=400, help="Isaac Sim only (default: %(default)s)")
    args = parser.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, f"{args.sim}_scene.png")
    if args.sim == "mujoco":
        render_mujoco(out)
    else:
        render_isaacsim(out, args.settle_frames)
    print("wrote", os.path.relpath(out, ROOT))


if __name__ == "__main__":
    main()
