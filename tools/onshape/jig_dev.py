"""Deploy marble_track.fs + marble_jigs.fs into the Marble Track Jigs document and check a test part studio.

    python3 jig_dev.py push            compile the studio (2 calls), print compile notices
    python3 jig_dev.py state           feature status in the jig_dev part studio (1 call)
    python3 jig_dev.py view [matrix]   shaded view png of jig_dev (1 call)
    python3 jig_dev.py add FEATURE     add a custom feature (default parameters) to jig_dev (2 calls)
Element ids are cached in jig_dev.json next to this file (created on first use, 1 call each).
"""
import base64
import json
import sys
from pathlib import Path

from onshape_api import call

HERE = Path(__file__).parent
DID, WID = "e09f203a007ff8f241cd910d", "8a3ae30352bf6fcd93c1d36d"
VERSION = 3132
CACHE = HERE / "jig_dev.json"
OUT = Path("/tmp/claude-1000/-home-btingle-cam-misc-p2/5bbd788e-577f-420a-b1e2-fcd075bbc705/scratchpad/onshape")
SOURCES = ["marble_track.fs", "marble_jigs.fs", "connector_holes.fs", "marble_machining.fs", "jig_tests.fs"]


TAB = __import__("os").environ.get("JIG_TAB", "partstudio")  # JIG_TAB=review -> jig_dev_review tab


def ids():
    cache = json.loads(CACHE.read_text()) if CACHE.exists() else {}
    if TAB != "partstudio" and TAB not in cache:
        cache[TAB] = call("POST", f"/partstudios/d/{DID}/w/{WID}", {"name": f"jig_dev_{TAB}"})["id"]
    if "studio" not in cache:
        cache["studio"] = call("POST", f"/featurestudios/d/{DID}/w/{WID}", {"name": "marble_jigs"})["id"]
    if "partstudio" not in cache:
        cache["partstudio"] = call("POST", f"/partstudios/d/{DID}/w/{WID}", {"name": "jig_dev"})["id"]
    CACHE.write_text(json.dumps(cache, indent=1))
    return cache


def source():
    header = f'FeatureScript {VERSION};\nimport(path : "onshape/std/geometry.fs", version : "{VERSION}.0");\n\n'
    return header + "\n".join((HERE / s).read_text() for s in SOURCES if (HERE / s).exists())


def push():
    c = ids()
    path = f"/featurestudios/d/{DID}/w/{WID}/e/{c['studio']}"
    current = call("GET", path)
    src = source()
    r = call("POST", path, {"btType": current["btType"], "contents": src,
                            "serializationVersion": current["serializationVersion"],
                            "sourceMicroversion": current["sourceMicroversion"], "rejectMicroversionSkew": False})
    lines = src.splitlines()
    json.dump({k: v for k, v in r.items() if k != "contents"}, open(OUT / "push_response.json", "w"), indent=1)
    notices = r.get("notices") or []
    for n in notices:
        msg = n.get("message", {})
        text = msg.get("message", msg) if isinstance(msg, dict) else msg
        loc = (n.get("message", {}).get("location") or {}) if isinstance(n.get("message"), dict) else {}
        line = loc.get("line") or 0
        print(f"{n.get('level', '?')}: {text} (line {line})")
        if line:
            for k in range(max(line - 2, 1), min(line + 1, len(lines)) + 1):
                print(f"   {k:5}| {lines[k - 1]}")
    c["microversion"] = r.get("microversionId") or r.get("sourceMicroversion")
    CACHE.write_text(json.dumps(c, indent=1))
    print("pushed", len(lines), "lines;", len(notices), "notices; microversion", c["microversion"])


def state():
    c = ids()
    r = call("GET", f"/partstudios/d/{DID}/w/{WID}/e/{c[TAB]}/features")
    states = r.get("featureStates", {})
    for f in r.get("features", []):
        s = states.get(f["featureId"], {})
        print(f["name"], f.get("featureType"), s.get("featureStatus"), s.get("inactive", ""))
    json.dump(r, open(OUT / "jig_dev_features.json", "w"), indent=1)


def view(matrix="isometric"):
    c = ids()
    r = call("GET", f"/partstudios/d/{DID}/w/{WID}/e/{c[TAB]}/shadedviews", outputHeight=800,
             outputWidth=1200, viewMatrix=matrix, pixelSize=0, edges="show")
    p = OUT / f"jig_dev_{TAB}_{matrix}.png"
    p.write_bytes(base64.b64decode(r["images"][0]))
    print("saved", p)


def namespace():
    c = ids()
    el = call("GET", f"/documents/d/{DID}/w/{WID}/elements", elementId=c["studio"])[0]
    return f"e{c['studio']}::m{el['microversionId']}"


def param(pid, value, ns):
    """Feature parameter JSON from a python value: Query ids list, enum 'Enum.VALUE', bool, int, or 'N mm' string."""
    if isinstance(value, list):
        return {"btType": "BTMParameterQueryList-148", "parameterId": pid,
                "queries": [{"btType": "BTMIndividualQuery-138", "deterministicIds": value}]}
    if isinstance(value, bool):
        return {"btType": "BTMParameterBoolean-144", "parameterId": pid, "value": value}
    if isinstance(value, str) and "." in value and " " not in value:
        enum, v = value.split(".")
        return {"btType": "BTMParameterEnum-145", "parameterId": pid, "enumName": enum, "value": v, "namespace": ns}
    return {"btType": "BTMParameterQuantity-147", "parameterId": pid, "expression": str(value)}


def add(feature_type, name=None, params=None, ns=None):
    """Add a custom feature (1 call, +1 for the namespace unless given). Returns the new feature id."""
    c = ids()
    ns = ns or namespace()
    r = call("POST", f"/partstudios/d/{DID}/w/{WID}/e/{c[TAB]}/features", {"feature": {
        "btType": "BTMFeature-134", "featureType": feature_type, "name": name or feature_type, "namespace": ns,
        "parameters": [param(k, v, ns) for k, v in (params or {}).items()]}})
    st = r.get("featureState", {})
    print(name or feature_type, r.get("feature", {}).get("featureId"), st.get("featureStatus"))
    return r.get("feature", {}).get("featureId")


def evaluate(script):
    """Run a FeatureScript lambda in the current tab (1 call); returns the result value."""
    c = ids()
    r = call("POST", f"/partstudios/d/{DID}/w/{WID}/e/{c[TAB]}/featurescript", {"script": script, "queries": {}})
    for n in r.get("notices") or []:
        print(n.get("level"), n.get("message"))
    if r.get("console"):
        print(r["console"])
    return r.get("result")


def refresh():
    """Point every custom feature in the tab at the current studio version (2 calls + 1 for the list)."""
    c = ids()
    ns = namespace()
    base = f"/partstudios/d/{DID}/w/{WID}/e/{c[TAB]}/features"
    cur = call("GET", base)
    feats = [dict(f, namespace=ns) for f in cur["features"] if f.get("namespace", "").startswith(f"e{c['studio']}::")]
    r = call("POST", f"{base}/updates", {"features": feats, "serializationVersion": cur["serializationVersion"],
                                          "sourceMicroversion": cur["sourceMicroversion"]})
    states = r.get("featureStates", {})
    for f in r.get("features", feats):
        print(f["name"], states.get(f["featureId"], {}).get("featureStatus"))


def reset():
    """Delete every feature in jig_dev (1 + n calls)."""
    c = ids()
    base = f"/partstudios/d/{DID}/w/{WID}/e/{c[TAB]}/features"
    for f in call("GET", base).get("features", []):
        call("DELETE", f"{base}/featureid/{f['featureId']}")
        print("deleted", f["name"])


if __name__ == "__main__":
    {"push": push, "state": state, "view": view, "add": add, "refresh": refresh, "reset": reset}[sys.argv[1]](*sys.argv[2:])
