"""Build one species' review part studio: the L_track_final profile plus one feature per jig.

    python3 review.py rl      -> tab jig_dev_rl (1 call to create the tab, 2 for the profile, 1 per jig)
"""
import os
import sys

SPECIES = sys.argv[1] if len(sys.argv) > 1 else "rl"
os.environ["JIG_TAB"] = SPECIES
import jig_dev  # noqa: E402


def machining(track, split, x):
    common = {"trackType": f"TrackType.{track}", "stockModel": True, "partModel": True, "placeX": f"{x} mm"}
    return [("marbleOp1Jaws", f"op1 {track}", {**common, "op1Split": f"JawSplit.{split}", "placeY": "0 mm"}),
            ("marbleOp2Jaws", f"op2 {track}", {**common, "op2Split": f"JawSplit.{split}", "placeY": "-170 mm"})]


def glue(species):
    return [("marbleGlueUpJig", f"glue-up {species}", {"species": f"JigSpecies.{species}", "placeY": "220 mm"}),
            ("marbleOp0Plate", f"op0 {species}", {"species": f"JigSpecies.{species}", "placeY": "360 mm"})]


JIGS = {
    "rl": glue("RL") + machining("CURVE_RL", "BOX_Y", 0) + machining("CURVE_LR", "BOX_Y", 200),
}


def main():
    ns = jig_dev.namespace()
    fid = jig_dev.add("jigDevProfile", "L_track_final profile", ns=ns)
    script = ('function(context is Context, queries) { return transientQueriesToStrings(evaluateQuery(context, '
              f'qSketchRegion(makeId("{fid}") + "sketch"))); }}')
    region = [v["value"] for v in jig_dev.evaluate(script)["value"]]
    for ftype, name, params in JIGS[SPECIES]:
        jig_dev.add(ftype, name, {"profile": region, **params}, ns=ns)


if __name__ == "__main__":
    main()
