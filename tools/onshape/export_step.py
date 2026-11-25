"""Export Part Studio tabs of the Marble Track Jigs document as STEP files, one file per tab.

    python3 export_step.py OUT_DIR [NAME_PART ...]    tabs whose name contains any NAME_PART (default: all)

About 4 API calls per exported tab (start, poll, download) plus 1 for the tab list. Tabs whose microversion
matches the last export (OUT_DIR/exports.json) are skipped.
"""
import json
import sys
import time
from pathlib import Path

from jig_dev import DID, WID
from onshape_api import call, download


def export(element, out):
    t = call("POST", f"/partstudios/d/{DID}/w/{WID}/e/{element['id']}/translations",
             {"formatName": "STEP", "storeInDocument": False, "flattenAssemblies": False})
    for _ in range(30):
        if t.get("requestState") in ("DONE", "FAILED"):
            break
        time.sleep(3)
        t = call("GET", f"/translations/{t['id']}")
    if t.get("requestState") != "DONE":
        raise SystemExit(f"{element['name']}: translation {t.get('requestState')} {t.get('failureReason', '')}")
    out.write_bytes(download(f"/documents/d/{DID}/externaldata/{t['resultExternalDataIds'][0]}"))


def main(out_dir, *names):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    manifest_path = out / "exports.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    for e in call("GET", f"/documents/d/{DID}/w/{WID}/elements"):
        if e["elementType"] != "PARTSTUDIO" or (names and not any(n in e["name"] for n in names)):
            continue
        path = out / (e["name"].replace("/", "-") + ".step")
        if manifest.get(e["id"]) == e["microversionId"] and path.exists():
            print("unchanged", e["name"])
            continue
        export(e, path)
        manifest[e["id"]] = e["microversionId"]
        manifest_path.write_text(json.dumps(manifest, indent=1))
        print("exported", path)


if __name__ == "__main__":
    main(*sys.argv[1:])
