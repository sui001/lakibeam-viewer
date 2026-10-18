"""Claims a generated part has to satisfy before it is allowed off the bench.

The point of this module is back pressure. Every check in here existed as a
print statement somewhere first, and printing is not a check: the brake side
shipped with a 34.7 cm3 mount floating free inside a file that reported itself
watertight, and every line of output at the time said the build had worked.
Nothing failed, so nothing was noticed.

So the geometry is not asked to describe itself, it is asked to satisfy stated
claims, and a violated claim ends the run with a non-zero exit. Two kinds of
entry go in the ledger:

    require()   the part is wrong if this is false, and the build fails
    note()      recorded because it is worth knowing, never fails anything

Nothing in here repairs anything. Repair belongs upstream where the context is,
and a check that quietly fixes what it finds is back to printing.

The ledger writes to claims.json next to the STLs, so a later build can be
diffed against an earlier one instead of remembered.
"""

import io
import json
import os

import numpy as np
import trimesh


BODY_FLOOR = 2000.0      # mm3, below this a lump is swarf rather than a body


class Claims:
    """A ledger of claims made about one build, and whether they held."""

    def __init__(self, subject):
        self.subject = subject
        self.rows = []

    def require(self, name, passed, detail="", **facts):
        passed = bool(passed)
        self.rows.append(dict(claim=name, kind="require", passed=passed,
                              detail=detail, facts=_plain(facts)))
        print("    %-5s %-38s %s" % ("ok" if passed else "FAIL", name, detail))
        return passed

    def note(self, name, detail="", **facts):
        self.rows.append(dict(claim=name, kind="note", passed=None,
                              detail=detail, facts=_plain(facts)))
        print("    %-5s %-38s %s" % ("note", name, detail))

    @property
    def failures(self):
        return [r for r in self.rows
                if r["kind"] == "require" and not r["passed"]]

    def ok(self):
        return not self.failures

    def report(self, path=None):
        """Print the verdict, write the ledger, and say whether it held."""
        req = [r for r in self.rows if r["kind"] == "require"]
        bad = self.failures
        if path:
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            with open(path, "w") as fh:
                json.dump(dict(subject=self.subject, passed=not bad,
                               claims=self.rows), fh, indent=2)
        if bad:
            print("  %d of %d claims FAILED:" % (len(bad), len(req)))
            for r in bad:
                print("    %s: %s" % (r["claim"], r["detail"]))
        else:
            print("  all %d claims hold" % len(req))
        if path:
            print("  ledger: %s" % path)
        return not bad


def _plain(d):
    """JSON will not take numpy scalars, and a silent TypeError helps nobody."""
    out = {}
    for k, v in d.items():
        if isinstance(v, (np.floating, np.integer)):
            v = v.item()
        elif isinstance(v, np.ndarray):
            v = [float(x) for x in v.ravel()]
        elif isinstance(v, (list, tuple)):
            v = [x.item() if isinstance(x, (np.floating, np.integer)) else x
                 for x in v]
        out[k] = v
    return out


# ---------------------------------------------------------------- geometry


def bodies(mesh, floor=BODY_FLOOR):
    """Connected lumps big enough to count, largest first."""
    return sorted((b for b in mesh.split(only_watertight=False)
                   if abs(b.volume) > floor), key=lambda b: -abs(b.volume))


def slab_volume(mesh, lo, hi, axis=0):
    """How much material sits between two planes, in mm3.

    Done with a boolean rather than by summing tetrahedra, because the reason
    for asking is usually whether a region survived a union, and a boolean
    answers that in the same terms the union used.
    """
    big = float(max(mesh.extents)) * 3.0
    size = [big, big, big]
    size[axis] = hi - lo
    box = trimesh.creation.box(extents=size)
    ctr = list(mesh.centroid)
    ctr[axis] = 0.5 * (lo + hi)
    box.apply_translation(ctr)
    try:
        cut = trimesh.boolean.intersection([mesh, box], engine="manifold")
    except Exception:
        return float("nan")
    return float(abs(cut.volume))


def as_written(mesh):
    """The mesh as the slicer will actually receive it.

    Not a float32 cast: a real STL encode and decode, because that is the round
    trip that has bitten this project. Casting in place keeps the topology and
    only moves the numbers, while writing the file merges and splits vertices,
    and that is where the holes appear.
    """
    blob = mesh.export(file_type="stl")
    return trimesh.load(io.BytesIO(blob), file_type="stl", process=True)


def closed_volume(C, mesh, label):
    """One closed, correctly wound, positive volume, and no rubbish faces."""
    deg = int(len(mesh.faces) - int(mesh.nondegenerate_faces().sum()))
    dup = int(len(mesh.faces) - int(mesh.unique_faces().sum()))
    holes = len(trimesh.grouping.group_rows(mesh.edges_sorted,
                                            require_count=1))
    ok = C.require(
        "%s is a closed volume" % label,
        mesh.is_volume,
        "watertight=%s winding=%s vol=%+.1f cm3 open edges=%d"
        % (mesh.is_watertight, mesh.is_winding_consistent,
           mesh.volume / 1000, holes),
        watertight=bool(mesh.is_watertight),
        winding_consistent=bool(mesh.is_winding_consistent),
        volume_cm3=round(float(mesh.volume) / 1000, 3),
        open_edges=holes)
    if deg or dup:
        C.note("%s has junk faces" % label,
               "%d degenerate, %d duplicate" % (deg, dup),
               degenerate=deg, duplicate=dup)
    return ok


def single_body(C, mesh, label, floor=BODY_FLOOR):
    """One connected lump, with nothing loose rattling around inside it.

    Watertight does not mean connected. A union can hand back a mesh that
    passes every solidity test and is still two separate objects, which slices
    without complaint and prints as loose pieces in a pile.
    """
    real = bodies(mesh, floor)
    strays = len(mesh.split(only_watertight=False)) - len(real)
    vols = [round(abs(float(b.volume)) / 1000, 1) for b in real]
    ok = C.require(
        "%s is one connected body" % label,
        len(real) == 1,
        "%d bodies over %.1f cm3, volumes %s cm3"
        % (len(real), floor / 1000, vols),
        body_count=len(real), body_volumes_cm3=vols)
    if strays:
        C.note("%s carries stray fragments" % label,
               "%d lump(s) under %.1f cm3" % (strays, floor / 1000),
               fragments=strays)
    return ok


def survives_the_file(C, mesh, label):
    """Still a closed single body after a real STL write and read.

    This is the claim the whole module was built for. Everything upstream runs
    in float64 and says so; STL stores float32, and a shell this finely lofted
    has sliver triangles thinner than float32 can represent at 250 mm. Each one
    that collapses on the way out leaves a hole in the file, so the only mesh
    worth making claims about is the one that came back off the disk.
    """
    back = as_written(mesh)
    real = bodies(back)
    moved = abs(float(mesh.volume) - float(back.volume)) / 1000
    return C.require(
        "%s survives being written" % label,
        back.is_volume and len(real) == 1,
        "written and read back: watertight=%s bodies=%d volume moved %.3f cm3"
        % (back.is_watertight, len(real), moved),
        watertight=bool(back.is_watertight), body_count=len(real),
        volume_delta_cm3=round(moved, 4))


def mount_retained(C, out, src, mount_x, min_frac=0.97):
    """The original mount is still in there, and still attached.

    single_body() catches the mount coming adrift. It does not catch the mount
    being dropped: a stray-fragment cull that picks the wrong lump leaves one
    connected body which is the new shell on its own, with nothing to bolt to.
    Both failures have the same cause and only this one is invisible.
    """
    want = slab_volume(src, src.bounds[0][0] - 1.0, mount_x)
    got = slab_volume(out, out.bounds[0][0] - 1.0, mount_x)
    frac = got / want if want > 0 else 0.0
    return C.require(
        "mount survived the union",
        frac >= min_frac,
        "%.1f of %.1f cm3 kept below x=%.0f (%.1f%%)"
        % (got / 1000, want / 1000, mount_x, 100 * frac),
        kept_cm3=round(got / 1000, 2), source_cm3=round(want / 1000, 2),
        fraction=round(frac, 4), mount_x=round(float(mount_x), 2))


def no_interference(C, a, b, label, max_cm3=0.01):
    """Two parts that are meant to mate do not occupy the same space.

    A tolerance rather than zero, because the tongue is built to a stated fit
    and the boolean works in floats. Anything above it is a real collision and
    the joint will not close.
    """
    try:
        both = trimesh.boolean.intersection([a, b], engine="manifold")
        v = abs(float(both.volume)) / 1000
    except Exception as e:
        return C.require("%s does not interfere" % label, False,
                         "intersection failed: %s" % e)
    return C.require(
        "%s does not interfere" % label,
        v <= max_cm3,
        "overlap %.4f cm3, allowed %.4f" % (v, max_cm3),
        overlap_cm3=round(v, 5), allowed_cm3=max_cm3)


def clearance_at_least(C, worst, floor, label):
    """The new shell has not moved into the space the old one left a hand."""
    return C.require(
        "%s leaves the hand room" % label,
        worst >= floor,
        "worst station keeps %.1f%% of the original cup, floor %.1f%%"
        % (100 * worst, 100 * floor),
        worst_retained=round(float(worst), 4), floor=floor)


def fits_plate(C, mesh, plate, label):
    """It goes on the bed in some orientation about Z, or it does not."""
    w, d, h = [float(v) for v in plate.lower().split("x")]
    e = mesh.extents
    fits, how = False, "too tall"
    if e[2] <= h:
        how = "no orientation fits"
        for ang in np.radians(np.arange(0, 90, 1.0)):
            bw = e[0] * np.cos(ang) + e[1] * np.sin(ang)
            bd = e[0] * np.sin(ang) + e[1] * np.cos(ang)
            if bw <= w and bd <= d:
                fits = True
                how = ("flat" if ang == 0
                       else "rotated %.0f deg" % np.degrees(ang))
                break
    return C.require(
        "%s fits the plate" % label,
        fits,
        "%.0f x %.0f x %.0f in %s, %s" % (e[0], e[1], e[2], plate, how),
        extents=[round(float(v), 2) for v in e], plate=plate, orientation=how)


def pieces_reassemble(C, pieces, whole, tol_frac=0.01):
    """The split pieces add back up to the part they came from.

    A cut plane that misses, or a piece that comes back as the wrong side of
    the boolean, shows up here and nowhere else: both halves can be perfectly
    good closed solids and still not be the guard.
    """
    total = sum(abs(float(p.volume)) for _, p in pieces)
    want = abs(float(whole.volume))
    off = abs(total - want) / want if want else 1.0
    return C.require(
        "the pieces are the whole part",
        off <= tol_frac,
        "%.1f cm3 of pieces against %.1f cm3 whole (%.2f%% out)"
        % (total / 1000, want / 1000, 100 * off),
        pieces_cm3=round(total / 1000, 2), whole_cm3=round(want / 1000, 2),
        error=round(off, 5),
        parts={nm: round(abs(float(p.volume)) / 1000, 2) for nm, p in pieces})


def repair_was_cosmetic(C, before, after, label, max_frac=0.02):
    """Whatever the loader had to mend, it did not change the part.

    load() repairs a source mesh into a closed volume, which is reasonable and
    is also exactly the sort of helpfulness that hides damage. Repair that
    moves real volume means the input is wrong, not merely untidy.
    """
    off = abs(after - before) / before if before else 1.0
    return C.require(
        "%s needed only cosmetic repair" % label,
        off <= max_frac,
        "volume moved %.2f%% during repair (%.1f -> %.1f cm3)"
        % (100 * off, before / 1000, after / 1000),
        before_cm3=round(before / 1000, 2), after_cm3=round(after / 1000, 2),
        error=round(off, 5))
