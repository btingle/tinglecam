"""Compile-check FeatureScript helpers through the Part Studio evaluator (1 API call).

Studio updates don't report compile errors, so: enums become string maps, top-level consts and functions
become local consts inside one lambda (dependency ordered), predicates and features are dropped.
Usage: fs_check.py [extra lambda body lines...]
"""
import re
import sys
from pathlib import Path

from check_track import block_end
from jig_dev import DID, WID, SOURCES, HERE, ids
from onshape_api import call


def transform(src):
    enums = {}
    for m in list(re.finditer(r"export enum (\w+)\s*", src))[::-1]:
        end = block_end(src, m.end())
        enums[m.group(1)] = re.findall(r"^\s*(\w+),?\s*$", src[m.end():end], re.M)
        src = src[:m.start()] + src[end:]
    # drop features (annotation + export const ... defineFeature(...) ... ;) and predicates
    bodies = []
    for m in list(re.finditer(r"^(annotation \{[^\n]*\}\n)?export const (\w+) = defineFeature\(function\((.*?)\)\s*precondition", src, re.M | re.S))[::-1]:
        pre = block_end(src, m.end())
        body = block_end(src, pre)
        end = src.index(";", body) + 1
        bodies.append(f"const {m.group(2)}Body = function({m.group(3)})" + src[pre:body] + ";")
        src = src[:m.start()] + src[end:]
    for m in list(re.finditer(r"^predicate \w+\(.*?\)\s*$", src, re.M))[::-1]:
        src = src[:m.start()] + src[block_end(src, m.end()):]
    consts = re.findall(r"^const .*?;\s*$", src, re.M)
    parts = {}
    for m in re.finditer(r"^function (\w+)\((.*?)\)( returns \w+)?\s*$", src, re.M):
        end = block_end(src, m.end())
        parts[m.group(1)] = (f"function({m.group(2)}){m.group(3) or ''}" + src[m.end():end], set())
    names = set(parts)
    for name, (code, deps) in parts.items():
        deps.update(n for n in names if n != name and re.search(rf"\b{n}\s*\(", code))
    ordered, done = [], set()
    while len(ordered) < len(parts):
        ready = [n for n, (_, d) in parts.items() if n not in done and d <= done]
        if not ready:
            raise ValueError("circular helper references: " + str(sorted(set(parts) - done)))
        for n in sorted(ready):
            ordered.append(n)
            done.add(n)
    defs = [f'const {e} = {{ {", ".join(f"{chr(34)}{v}{chr(34)} : {chr(34)}{v}{chr(34)}" for v in vals)} }};'
            for e, vals in enums.items()] + consts + [f"const {n} = {parts[n][0]};" for n in ordered] + bodies
    out = "\n".join(defs)
    for e in enums:
        out = out.replace(f" is {e}", "").replace(f" returns {e}", "")
    return out


def main(extra):
    src = "\n".join((HERE / s).read_text() for s in SOURCES)
    script = "function(context is Context, queries)\n{\n" + transform(src) + "\n" + "\n".join(extra) + "\n    return 0;\n}"
    out = Path("/tmp/claude-1000/-home-btingle-cam-misc-p2/5bbd788e-577f-420a-b1e2-fcd075bbc705/scratchpad/onshape/fs_check_script.fs")
    out.write_text(script)
    c = ids()
    r = call("POST", f"/partstudios/d/{DID}/w/{WID}/e/{c['partstudio']}/featurescript", {"script": script, "queries": {}})
    lines = script.splitlines()
    for n in r.get("notices") or []:
        loc = (n.get("stackTrace") or n.get("locationInfos") or [{}])[0]
        line = loc.get("line", 0)
        print(f"{n['level']} {n['type']}: {n['message']} (line {line})")
        for k in range(max(line - 2, 1), min(line + 1, len(lines)) + 1):
            print(f"   {k:5}| {lines[k - 1]}")
    print("console:", r.get("console", ""))
    print("result:", str(r.get("result"))[:300])


if __name__ == "__main__":
    main(sys.argv[1:])
