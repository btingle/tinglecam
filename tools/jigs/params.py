"""Every tweakable number in the jig pipeline (mm). Override any field from a JSON file."""
import json
from dataclasses import dataclass, fields, replace
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class Params:
    # track
    profile: str = str(REPO / "models/2D/trackprofile.dxf")
    grid: float = 40.0
    elevation: float = 6.0
    track_tolerance: float = 0.01   # STEP fit of the real track
    pocket_tolerance: float = 0.02  # STEP fit of jig pockets/walls
    pocket_simplify: float = 0.02   # polyline deviation allowed in jig pocket/wall profiles

    # wood blanks (laser)
    sheet: float = 6.35
    laser_allowance: float = 1.5
    fixture_hole_d: float = 3.175   # M4 threads straight into it
    fixture_hole_t: tuple = (0.3, 0.7)  # along the centreline; keep symmetric so mirrored/reversed pieces share holes

    # glue-up (acrylic)
    acrylic: float = 6.0
    web: float = 8.0                # acrylic between neighbouring blank outlines
    guide_clearance: float = 0.15   # guide cutout over the blank outline
    glue_bolt_d: float = 5.5        # M5 pass-through
    glue_nut_af: float = 8.0        # M5 nut across flats (hex socket in bottom plate)
    glue_nut_clearance: float = 0.2
    glue_bolt_wall: float = 3.0     # acrylic between a bolt hole and a cutout
    glue_plate_margin: float = 7.0  # bolt hole edge to plate edge
    glue_plate_corner_r: float = 10.0

    # op0 plate
    op0_thickness: float = 12.0
    op0_margin: float = 10.0        # plate edge beyond the blank outlines
    op0_corner_r: float = 6.0
    op0_bolt_d: float = 4.5         # M4 clearance
    op0_csk_d: float = 8.6          # 90 degree countersink for M4 flat heads

    # vise
    vise_jaw_width: float = 150.0

    # op1 soft jaws (top side)
    op1_pocket_clearance: float = 0.1
    op1_top_clearance: float = 1.5  # jaw stays this far below the finished track top
    op1_wall: float = 4.0           # raised jaw wall either side of the track
    op1_floor: float = 5.0          # material under the lowest pocket point
    op1_piece_gap: float = 10.0     # between pockets along the vise jaw
    op1_margin: float = 12.0        # pocket to the clamped faces
    split_gap: float = 0.3          # removed between the two jaws

    # op2 face jaws (end faces)
    op2_pocket_clearance: float = 0.1
    op2_wall: float = 8.0           # jaw material around the piece
    op2_face_proud: float = 0.5     # end face stands this far above the jaw
    op2_bolt_d: float = 3.4         # M3 clearance through jaw B
    op2_bolt_head_d: float = 6.5
    op2_bolt_head_depth: float = 3.5
    op2_insert_d: float = 4.0       # M3 heat-set insert hole in jaw A
    op2_insert_depth: float = 6.0
    op2_insert_wall: float = 2.5    # insert/bolt hole to pocket

    def __post_init__(self):
        # tuple fields arrive as lists from JSON
        object.__setattr__(self, "fixture_hole_t", tuple(self.fixture_hole_t))

    @classmethod
    def load(cls, path=None, **overrides):
        values = json.loads(Path(path).read_text()) if path else {}
        values.update(overrides)
        unknown = set(values) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f"Unknown jig parameters: {sorted(unknown)}")
        return replace(cls(), **values)
