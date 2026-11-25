"""Run marble_track.fs through the Part Studio FeatureScript evaluator.

The evaluator only accepts one lambda, so top-level helpers become local consts,
the enum becomes a map of strings, and the feature body becomes a callable.
Usage: check_track.py [TRACK_TYPE ...]  (no types: parse/definition check only)
"""
import json
import re
import sys
from pathlib import Path

from onshape_api import call

PS = "/partstudios/d/9e8e51cb6505a5bed053f49d/w/e485afda1d5f6e92d9b310f4/e/765dfe7db867b5eb08eb5af1/featurescript"


def block_end(text, start):
    """Index just past the brace block opening at or after start."""
    i = text.index("{", start)
    depth = 0
    for j in range(i, len(text)):
        depth += {"{": 1, "}": -1}.get(text[j], 0)
        if depth == 0:
            return j + 1
    raise ValueError("unbalanced braces")


def transform(src):
    parts = {}
    enum = re.search(r"export enum (\w+)\s*", src)
    end = block_end(src, enum.end())
    values = re.findall(r"^\s*(\w+),?\s*$", src[enum.end():end], re.M)
    enum_name = enum.group(1)
    enum_def = f'const {enum_name} = {{ {", ".join(f"{chr(34)}{v}{chr(34)} : {chr(34)}{v}{chr(34)}" for v in values)} }};'
    src = src[:enum.start()] + src[end:]

    consts = re.findall(r"^const .*?;\s*$", src, re.M)

    feat = re.search(r"defineFeature\(function\((.*?)\)\s*precondition", src, re.S)
    pre_end = block_end(src, feat.end())
    body_end = block_end(src, pre_end)
    parts["marbleTrackBody"] = (f"function({feat.group(1)})" + src[pre_end:body_end], set())

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
            raise ValueError("circular helper references")
        for n in sorted(ready):
            ordered.append(n)
            done.add(n)
    defs = [enum_def] + consts + [f"const {n} = {parts[n][0]};" for n in ordered]
    return "\n".join(defs).replace(f" is {enum_name}", "")


def main(types):
    lib = transform(Path(__file__).with_name("marble_track.fs").read_text())
    runs = "\n".join(f'''
    {{
        println("--- {t}");
        const before = size(evaluateQuery(context, qEverything(EntityType.BODY)));
        try
        {{
            marbleTrackBody(context, makeId("chk{t}"), mergeMaps(defn, {{ "trackType" : "{t}" }}));
            const made = qCreatedBy(makeId("chk{t}"), EntityType.BODY);
            for (var b in evaluateQuery(context, made))
                println("  body " ~ toString(evaluateQuery(context, qBodyType(b, BodyType.SOLID)) != []) ~ " solid, volume " ~ toString(evVolume(context, {{ "entities" : qBodyType(b, BodyType.SOLID) }})));
            println("  bodies created: " ~ size(evaluateQuery(context, made)));
        }}
        catch (e)
        {{
            println("  ERROR: " ~ toString(e));
        }}
    }}''' for t in types)
    script = f'''function(context is Context, queries)
{{
{lib}
    const regions = evaluateQuery(context, qSketchFilter(qEverything(EntityType.FACE), SketchObject.YES));
    println("sketch regions: " ~ size(regions));
    const defn = {{
        "profile" : size(regions) > 0 ? regions[0] : qNothing(),
        "gridSize" : 40 * millimeter,
        "elevationGain" : 6 * millimeter,
        "profileSamples" : 9,
        "pathSamples" : 33,
        "surfacesOnly" : false
    }};
{runs}
    return 0;
}}'''
    Path("/tmp/claude-1000/-home-btingle-cam-misc-p2/0ab6e492-9bb5-4f5c-981e-f8ad630d7d39/scratchpad/check_script.fs").write_text(script)
    r = call("POST", PS, {"script": script, "queries": {}})
    lines = script.splitlines()
    for n in r.get("notices") or []:
        loc = (n.get("stackTrace") or n.get("locationInfos") or [{}])[0]
        line = loc.get("line", 0)
        print(f"{n['level']} {n['type']}: {n['message']} (line {line})")
        for k in range(max(line - 2, 1), min(line + 1, len(lines)) + 1):
            print(f"   {k:4}| {lines[k - 1]}")
    print(r.get("console", ""))


if __name__ == "__main__":
    main(sys.argv[1:])
