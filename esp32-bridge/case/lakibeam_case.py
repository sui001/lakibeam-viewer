#!/usr/bin/env python
"""Enclosure for the LakiBeam bridge perf board.

A 30 x 70 perf board carrying a W5500 Ethernet module at one end and an
ESP32-S3 SuperMini at the other. The board screws down onto four posts through
its corner mounting holes, the two ports come out the short ends, and a
snap-fit lid closes it.

Built in build123d rather than trimesh because this is a new part, and B-rep
does not have the watertightness failure class that mesh work does. The claims
still run against the exported STL, because the slicer only ever sees the
bytes.

Measurements are Sui's, taken with calipers on the soldered board:

    board          30 x 70 x 1.46
    RJ45           16.1 wide (30 - 8.1 - 5.8), sits 4.6 above the board
    USB-C           7.8 wide (30 - 10.65 - 11.55), sits 4.0 above the board

Port widths are derived from the two edge distances rather than measured
directly, which is why they carry a clearance rather than a tight fit.
"""

import argparse
import os
import sys

from build123d import (Align, Axis, Box, Cylinder, Pos, export_stl)
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import claims as K


# ── Board and connectors, as measured ────────────────────────────────────────
BOARD_W, BOARD_L, BOARD_T = 30.0, 70.0, 1.46

# The board's four plated corner mounting holes are what it hangs on, not the
# 0.1 inch perf grid. Their inset is a --hole-inset-x/y flag because it has not
# been measured yet.

RJ45_W, RJ45_H, RJ45_Z = 16.1, 13.5, 4.6     # height is the standard jack
RJ45_X = 8.1                                 # from the x=0 long edge
USB_W, USB_H, USB_Z = 7.8, 3.2, 4.0
USB_X = 10.65                                # from the x=0 long edge


def build(a):
    """Return (box, lid) as build123d solids, plus a dict of derived heights."""

    cav_w = BOARD_W + 2 * a.board_fit
    # Extra length at the RJ45 end only, for however far the jack's shell hangs
    # past the board edge. A jack that overhangs further than the board fit is
    # a hard collision with the end wall, not a tolerance problem.
    cav_l = BOARD_L + 2 * a.board_fit + a.rj45_overhang

    # Everything measured from the outside bottom of the box.
    floor_z = a.floor
    board_bot_z = floor_z + a.stand
    board_top_z = board_bot_z + BOARD_T

    rj_z0 = board_top_z + RJ45_Z
    rj_z1 = rj_z0 + RJ45_H
    cav_top_z = rj_z1 + a.headroom              # tallest thing sets the height

    out_w = cav_w + 2 * a.wall
    out_l = cav_l + 2 * a.wall

    # Board origin inside the box.
    bx0 = a.wall + a.board_fit
    by0 = a.wall + a.board_fit

    shell = Box(out_w, out_l, cav_top_z, align=(Align.MIN, Align.MIN, Align.MIN))
    cavity = Pos(a.wall, a.wall, floor_z) * Box(
        cav_w, cav_l, cav_top_z - floor_z + 1,
        align=(Align.MIN, Align.MIN, Align.MIN))
    box = shell - cavity

    # ── Screw posts ──────────────────────────────────────────────────────────
    # The board has four plated corner mounting holes, so it gets screwed down
    # rather than merely located. Post carries it at `stand`; the pilot hole is
    # undersize for an M2 to self-tap into.
    posts = []
    for hx in (a.hole_inset_x, BOARD_W - a.hole_inset_x):
        for hy in (a.hole_inset_y, BOARD_L - a.hole_inset_y):
            cx, cy = bx0 + hx, by0 + hy
            posts.append((cx, cy))
            box += Pos(cx, cy, floor_z) * Cylinder(
                a.post_d / 2, a.stand, align=(Align.CENTER, Align.CENTER, Align.MIN))
            # Blind, stopping short of the outside so the floor stays sealed.
            box -= Pos(cx, cy, floor_z + a.stand - a.pilot_depth) * Cylinder(
                a.pilot_d / 2, a.pilot_depth + 1,
                align=(Align.CENTER, Align.CENTER, Align.MIN))

    # ── Port apertures ───────────────────────────────────────────────────────
    # Cut oversize by `port_fit` on every side. Deliberately generous: the port
    # widths are derived from edge distances rather than measured, and a port
    # hole a millimetre large is invisible where one a millimetre small is a
    # reprint.
    def aperture(x0, w, z0, h, y_at_zero):
        c = a.port_fit
        y = -1 if y_at_zero else out_l - a.wall - 1
        return Pos(bx0 + x0 - c, y, z0 - c) * Box(
            w + 2 * c, a.wall + 2, h + 2 * c,
            align=(Align.MIN, Align.MIN, Align.MIN))

    rj_cut = aperture(RJ45_X, RJ45_W, rj_z0, RJ45_H, y_at_zero=False)

    # The USB-C receptacle sits about 3mm back from the board edge, so with the
    # wall and the board fit a plug has roughly 5.7mm of tunnel to cross before
    # it mates. A receptacle-sized hole means the plug's moulding lands on the
    # outside of the case and the contacts only just reach. Cut the hole to the
    # plug body instead so the moulding can come through, centred on the
    # receptacle.
    usb_z0 = board_top_z + USB_Z
    plug_x = USB_X + USB_W / 2 - a.usb_plug_w / 2
    plug_z = usb_z0 + USB_H / 2 - a.usb_plug_h / 2
    usb_cut = Pos(bx0 + plug_x, -1, plug_z) * Box(
        a.usb_plug_w, a.wall + 2, a.usb_plug_h,
        align=(Align.MIN, Align.MIN, Align.MIN))
    box = box - rj_cut - usb_cut

    # ── Snap groove ──────────────────────────────────────────────────────────
    # A slot in each long inner wall that the lid's bead drops into. Cut as a
    # horizontal slot in a vertical wall, so it bridges over `groove_d` and
    # prints without support.
    gz = cav_top_z - a.snap_depth
    for side in (0, 1):
        gx = a.wall - a.groove_d if side == 0 else a.wall + cav_w
        box -= Pos(gx, a.wall, gz) * Box(
            a.groove_d, cav_l, a.groove_h,
            align=(Align.MIN, Align.MIN, Align.MIN))

    # ── Lid ──────────────────────────────────────────────────────────────────
    lid = Box(out_w, out_l, a.lid_t, align=(Align.MIN, Align.MIN, Align.MIN))

    sk_w = cav_w - 2 * a.lid_fit
    sk_l = cav_l - 2 * a.lid_fit
    sk_x = a.wall + a.lid_fit        # same inset on both axes
    sk_y = a.wall + a.lid_fit
    skirt = Pos(sk_x, sk_y, -a.skirt_h) * Box(
        sk_w, sk_l, a.skirt_h, align=(Align.MIN, Align.MIN, Align.MIN))
    skirt -= Pos(sk_x + a.skirt_t, sk_y + a.skirt_t, -a.skirt_h - 1) * Box(
        sk_w - 2 * a.skirt_t, sk_l - 2 * a.skirt_t, a.skirt_h + 2,
        align=(Align.MIN, Align.MIN, Align.MIN))

    # Open the skirt at the RJ45 end. The jack reaches within a couple of mm of
    # the rim, and a skirt that runs the full perimeter drops straight onto it,
    # which is exactly what stopped the first print closing. Three sides is
    # still plenty stiff, and the beads live on the long sides anyway.
    skirt -= Pos(sk_x - 1, sk_y + sk_l - a.skirt_t - 0.01, -a.skirt_h - 1) * Box(
        sk_w + 2, a.skirt_t + 1, a.skirt_h + 2,
        align=(Align.MIN, Align.MIN, Align.MIN))
    lid += skirt

    # Beads on the long faces only. Two-sided snap needs less force than four
    # and is far easier to get off again without cracking a corner.
    bz = -a.snap_depth
    for side in (0, 1):
        bx = sk_x - a.bead_d if side == 0 else sk_x + sk_w
        lid += Pos(bx, a.wall + a.lid_fit + a.bead_inset, bz) * Box(
            a.bead_d, sk_l - 2 * a.bead_inset, a.bead_h,
            align=(Align.MIN, Align.MIN, Align.MIN))

    return box, lid, dict(
        cav_w=cav_w, cav_l=cav_l, out_w=out_w, out_l=out_l,
        cav_top_z=cav_top_z, board_bot_z=board_bot_z, board_top_z=board_top_z,
        rj_z0=rj_z0, rj_z1=rj_z1, bx0=bx0, by0=by0, floor_z=floor_z,
        posts=posts, usb_plug_x=plug_x, usb_plug_z=plug_z, usb_z0=usb_z0)


def as_mesh(solid, path):
    """Export, then read back what was written.

    Loaded with processing on, which merges the duplicated vertices an STL
    always carries. That is parsing, not repair: STL is a triangle soup with no
    shared vertices by definition, so without the merge every mesh reads as
    entirely open edges and no topology claim means anything.
    """
    export_stl(solid, path)
    return trimesh.load(path)


def make_parser():
    """The single definition of every dimension this part has.

    Shared with lakibeam_case_views so the drawings cannot depict a
    different part than the STL. A hand-copied second set of defaults is
    exactly the sort of quiet disagreement this project keeps being bitten
    by, and it would look correct right up until the print.
    """
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", default=os.path.dirname(os.path.abspath(__file__)))
    p.add_argument("--wall", type=float, default=2.0)
    p.add_argument("--floor", type=float, default=2.0)
    p.add_argument("--lid-t", type=float, default=2.0)
    p.add_argument("--stand", type=float, default=5.0,
                   help="clear air under the board")
    p.add_argument("--headroom", type=float, default=2.0,
                   help="above the tallest connector")
    p.add_argument("--board-fit", type=float, default=0.5,
                   help="clearance around the board, per side")
    p.add_argument("--port-fit", type=float, default=1.0,
                   help="clearance around each port aperture, per side")
    p.add_argument("--post-d", type=float, default=5.0)
    p.add_argument("--pilot-d", type=float, default=1.7,
                   help="undersize for an M2 to self-tap into")
    p.add_argument("--pilot-depth", type=float, default=4.0)
    p.add_argument("--hole-inset-x", type=float, default=2.2,
                   help="ESTIMATED off a photo. centre in from the long edge")
    p.add_argument("--hole-inset-y", type=float, default=2.5,
                   help="ESTIMATED off a photo. centre in from the short edge")
    p.add_argument("--rj45-overhang", type=float, default=0.0,
                   help="ASSUMED 0. how far the jack shell passes the board edge")
    p.add_argument("--usb-plug-w", type=float, default=13.0,
                   help="USB-C plug moulding, not the receptacle")
    p.add_argument("--usb-plug-h", type=float, default=7.0)
    p.add_argument("--rj45-depth", type=float, default=21.0,
                   help="how far the jack reaches back from the board end")
    p.add_argument("--usb-inset", type=float, default=3.2,
                   help="measured off a photo: receptacle face back from the edge")
    p.add_argument("--usb-depth", type=float, default=7.0)
    p.add_argument("--lid-fit", type=float, default=0.15,
                   help="clearance around the lid skirt, per side")
    p.add_argument("--skirt-h", type=float, default=5.0)
    p.add_argument("--skirt-t", type=float, default=1.5)
    p.add_argument("--snap-depth", type=float, default=3.5,
                   help="bead centre below the rim")
    p.add_argument("--bead-d", type=float, default=0.5)
    p.add_argument("--bead-h", type=float, default=1.2)
    p.add_argument("--bead-inset", type=float, default=6.0)
    p.add_argument("--groove-d", type=float, default=0.7)
    p.add_argument("--groove-h", type=float, default=1.6)
    p.add_argument("--plate", default="220x220x250", help="bed as WxDxH")
    p.add_argument("--max-interference", type=float, default=0.01,
                   help="cm3 allowed where two solids should not meet")
    return p


def main():
    a = make_parser().parse_args()

    os.makedirs(a.out, exist_ok=True)
    C = K.Claims("lakibeam-case")

    box, lid, d = build(a)

    print("  derived:")
    print("    outer          %.2f x %.2f x %.2f" % (d["out_w"], d["out_l"], d["cav_top_z"]))
    print("    cavity         %.2f x %.2f" % (d["cav_w"], d["cav_l"]))
    print("    board sits at  %.2f, top face %.2f" % (d["board_bot_z"], d["board_top_z"]))
    print("    RJ45 aperture  %.2f to %.2f" % (d["rj_z0"], d["rj_z1"]))

    box_stl = os.path.join(a.out, "case_body.stl")
    lid_stl = os.path.join(a.out, "case_lid.stl")
    bm = as_mesh(box, box_stl)
    lm = as_mesh(lid, lid_stl)

    ok = True
    for mesh, label in ((bm, "body"), (lm, "lid")):
        ok &= K.closed_volume(C, mesh, label)
        ok &= K.single_body(C, mesh, label)
        ok &= K.survives_the_file(C, mesh, label)
        ok &= K.fits_plate(C, mesh, a.plate, label)

    # The apertures must actually be holes. A probe the size of each opening,
    # spanning the wall, has to pass through without touching anything. This
    # is the claim that catches a cut that silently missed.
    def probe(x0, w, z0, h, y_at_zero):
        c = a.port_fit
        y = 0.0 if y_at_zero else d["out_l"] - a.wall
        return Pos(d["bx0"] + x0 - c, y, z0 - c) * Box(
            w + 2 * c, a.wall, h + 2 * c,
            align=(Align.MIN, Align.MIN, Align.MIN))

    rj_probe = as_mesh(probe(RJ45_X, RJ45_W, d["rj_z0"], RJ45_H, False),
                       os.path.join(a.out, "_probe_rj45.stl"))
    usb_probe = as_mesh(probe(USB_X, USB_W, d["board_top_z"] + USB_Z, USB_H, True),
                        os.path.join(a.out, "_probe_usb.stl"))
    ok &= K.no_interference(C, bm, rj_probe, "RJ45 aperture is open", a.max_interference)
    ok &= K.no_interference(C, bm, usb_probe, "USB-C aperture is open", a.max_interference)

    # The board's own envelope has to be empty air. Nothing reaches into it now
    # that the board is screwed down onto posts rather than located by pips, so
    # unlike the pip version this one really is zero.
    board_probe = as_mesh(
        Pos(d["bx0"], d["by0"], d["board_bot_z"]) * Box(
            BOARD_W, BOARD_L, BOARD_T, align=(Align.MIN, Align.MIN, Align.MIN)),
        os.path.join(a.out, "_probe_board.stl"))
    ok &= K.no_interference(C, bm, board_probe, "board envelope is clear",
                            a.max_interference)

    # The pilot holes have to be holes. A post with no bore looks identical
    # from outside and is only discovered with a screw in your hand.
    bores = None
    for cx, cy in d["posts"]:
        s = Pos(cx, cy, d["floor_z"] + a.stand - a.pilot_depth) * Cylinder(
            a.pilot_d / 2 - 0.05, a.pilot_depth,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
        bores = s if bores is None else bores + s
    bore_probe = as_mesh(bores, os.path.join(a.out, "_probe_bores.stl"))
    ok &= K.no_interference(C, bm, bore_probe, "screw pilots are bored",
                            a.max_interference)

    # The claim whose absence let the first print fail. The lid was only ever
    # checked as geometry on its own, never against what it has to close over,
    # so a skirt that dropped straight onto the RJ45 passed every check and was
    # found in the plastic.
    # Depth matters as much as height here. An earlier version of this claim
    # modelled each connector as running the whole 70mm of board, which is not
    # what they do, and the false overlap it produced would have been "fixed"
    # by raising the threshold. The jack is about 21mm deep at its end of the
    # board; the USB-C is about 7mm, set back from its edge.
    rj_y0 = d["by0"] + BOARD_L - a.rj45_depth
    usb_y0 = d["by0"] + a.usb_inset
    comps = None
    for x0, w, y0, dep, z0, h in (
            (RJ45_X, RJ45_W, rj_y0, a.rj45_depth, d["rj_z0"], RJ45_H),
            (USB_X, USB_W, usb_y0, a.usb_depth, d["usb_z0"], USB_H)):
        c = Pos(d["bx0"] + x0, y0, z0) * Box(
            w, dep, h, align=(Align.MIN, Align.MIN, Align.MIN))
        comps = c if comps is None else comps + c
    comp_mesh = as_mesh(comps, os.path.join(a.out, "_probe_comps.stl"))

    seated = Pos(0, 0, d["cav_top_z"]) * lid
    seated_mesh = as_mesh(seated, os.path.join(a.out, "_seated_lid.stl"))
    ok &= K.no_interference(C, seated_mesh, comp_mesh,
                            "seated lid clears the connectors", a.max_interference)
    ok &= K.no_interference(C, seated_mesh, board_probe,
                            "seated lid clears the board", a.max_interference)

    C.note("port apertures are oversize",
           "%.2f mm clearance per side, widths derived not measured" % a.port_fit,
           port_fit_mm=a.port_fit)
    C.note("corner holes are assumed",
           "inset %.2f x %.2f, overhang %.2f, none of these measured yet"
           % (a.hole_inset_x, a.hole_inset_y, a.rj45_overhang),
           hole_inset_x=a.hole_inset_x, hole_inset_y=a.hole_inset_y,
           rj45_overhang=a.rj45_overhang)

    for f in ("_probe_rj45.stl", "_probe_usb.stl", "_probe_board.stl",
              "_probe_bores.stl", "_probe_comps.stl", "_seated_lid.stl"):
        os.remove(os.path.join(a.out, f))

    if not (C.report(os.path.join(a.out, "claims.json")) and ok):
        raise SystemExit(1)
    print("  wrote %s and %s" % (box_stl, lid_stl))


if __name__ == "__main__":
    main()
