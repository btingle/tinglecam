"""Push marble_track.fs into the Feature Studio and report compile results."""
import json
import sys
from pathlib import Path

from onshape_api import call

DOC = "/d/9e8e51cb6505a5bed053f49d/w/e485afda1d5f6e92d9b310f4/e/42e7d8b38047c22ac135878c"
HEADER = 'FeatureScript 3112;\nimport(path : "onshape/std/geometry.fs", version : "3112.0");\n\n'

current = call("GET", "/featurestudios" + DOC)
source = HEADER + Path(__file__).with_name("marble_track.fs").read_text()
result = call("POST", "/featurestudios" + DOC, {
    "btType": current["btType"],
    "contents": source,
    "serializationVersion": current["serializationVersion"],
    "sourceMicroversion": current["sourceMicroversion"],
    "rejectMicroversionSkew": False,
})
print(json.dumps({k: v for k, v in result.items() if k != "contents"}, indent=1)[:4000])
specs = call("GET", "/featurestudios" + DOC + "/featurespecs")
print(json.dumps(specs, indent=1)[:int(sys.argv[1]) if len(sys.argv) > 1 else 4000])
