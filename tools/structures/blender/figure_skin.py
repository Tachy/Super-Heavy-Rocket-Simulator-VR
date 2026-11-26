"""The suit as the GAME skins it (fix 9vr-c), for the tests: bone transforms from per-bone rotations, the corrective-shape weights of
game/cockpit/figure_deform.gd (bend angle from the posed bones, swing gate, mid hat) and the turned-over triangle count per pose.

Poses: `chain_pose(cfg, rotations)` builds {bone: 4x4} (rest -> posed, figure frame) from global rotations per bone (each bone's head
follows its parent's transform), so a pose recorded on one skeleton applies to another (the shoulder pivot can move). `swing_pose`
swings one upper arm (with its children) forward-out (the raise to the console, figure_layout.bake_direction) or backward."""
import math

import numpy as np

import figure_body as fb
import figure_layout as fl


def chain_pose(cfg, rotations):
    """{bone: 4x4} from {bone: 3x3 global rotation (rest -> posed)}; bones without a rotation keep their parent's motion only."""
    b = fl.bones(cfg)
    out = {}
    pending = list(fl.BONE_NAMES)
    while pending:
        for n in list(pending):
            par = b[n]["parent"]
            if par and par not in out:
                continue
            head = np.array(b[n]["head"])
            r = np.asarray(rotations.get(n, np.eye(3)), dtype=float)
            t = np.eye(4)
            t[:3, :3] = r
            t[:3, 3] = (fb.apply(out[par], head) if par else head) - r @ head
            out[n] = t
            pending.remove(n)
    return out


def swing_pose(cfg, side, theta_deg, backward=False):
    """The upper arm of `side` (and its children) swung to theta from the torso's down axis: forward and a quarter out (the baked raise
    plane) or backward and a tenth out; the rest of the body at rest."""
    j = dict(fl.joint_table(cfg)[f"shoulder_{side}"])
    b = fl.bones(cfg)
    ref = fb._n(np.array(b[j["ref"]]["tail"]) - np.array(b[j["ref"]]["head"])) * j["ref_sign"]
    s = 1.0 if side == "l" else -1.0
    p = np.array([0.0, -1.0, 0.1 * s]) if backward else np.array([0.0, 1.0, fl.SHOULDER_BAKE_OUT * s])
    j["plane"] = list(fb._n(p - ref * (ref @ p)))
    return fb.joint_pose(cfg, j, theta_deg)


def game_skin(mesh, cfg, pose):
    """(points, transforms): LBS of the rest mesh plus the corrective shapes weighted as figure_deform.gd does for this pose."""
    names = mesh["names"]
    b = fl.bones(cfg)
    transforms = [pose.get(n, np.eye(4)) for n in names]

    def head(n):
        return fb.apply(pose.get(n, np.eye(4)), np.array(b[n]["head"]))

    offs = np.zeros_like(mesh["rest"])
    for j in fl.joint_table(cfg).values():
        ref = (head(j["ref_tail"]) - head(j["ref"])) * j["ref_sign"]
        child = head(j["child_tail"]) - head(j["child"])
        theta = math.degrees(math.acos(float(np.clip(fb._n(ref) @ fb._n(child), -1.0, 1.0))))
        flex, ext, _ = fl.bend_weights(j, theta)
        gate = 1.0
        if j["gated"]:
            r = fb._n(ref)
            swing = child - r * (r @ child)
            if np.linalg.norm(swing) > 1e-3 * np.linalg.norm(child):
                ref_rot = pose.get(j["ref"], np.eye(4))[:3, :3]
                gate = fl.swing_gate(j, fb._n(swing) @ fb._n(ref_rot @ np.array(j["plane"])))
        offs += gate * flex * mesh["shapes"][j["shape_flex"]] + ext * mesh["shapes"][j["shape_ext"]]
        if j["shape_mid"]:
            offs += gate * fl.mid_weight(j, theta) * mesh["shapes"][j["shape_mid"]]
    return fb.lbs(mesh["rest"] + offs, mesh["w"], transforms), transforms


def inverted_count(mesh, cfg, pose):
    """Turned-over suit triangles in a pose (figure_body.inverted_triangles of the game's skinning)."""
    points, transforms = game_skin(mesh, cfg, pose)
    return len(fb.inverted_triangles(mesh["hull"], mesh["rest"], points, mesh["w"], transforms))
