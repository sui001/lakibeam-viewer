#!/usr/bin/env python
"""Line drawings of the LakiBeam case, for looking at before printing.

Hidden-line removal through OpenCascade, exported as SVG. No renderer and no
GPU, which matters because there is neither on this machine.

Takes the same flags as lakibeam_case.py, from the same parser, so a drawing
can never depict a part different from the one that gets exported.
"""

import os
import sys

from build123d import ExportSVG, LineType

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lakibeam_case as LC


VIEWS = [
    ("iso", (60.0, -80.0, 55.0)),
    ("rj45_end", (0.0, 100.0, 0.0)),
    ("usb_end", (0.0, -100.0, 0.0)),
    ("side", (100.0, 0.0, 0.0)),
]


def main():
    a = LC.make_parser().parse_args()
    box, lid, d = LC.build(a)
    os.makedirs(a.out, exist_ok=True)

    for label, eye in VIEWS:
        for part, pname in ((box, "body"), (lid, "lid")):
            visible, hidden = part.project_to_viewport(eye)
            ex = ExportSVG(scale=4)
            ex.add_layer("visible", line_weight=0.35)
            ex.add_layer("hidden", line_weight=0.15, line_type=LineType.DASHED)
            ex.add_shape(visible, layer="visible")
            ex.add_shape(hidden, layer="hidden")
            path = os.path.join(a.out, "view_%s_%s.svg" % (pname, label))
            ex.write(path)
            print("  wrote %s" % path)


if __name__ == "__main__":
    main()
