"""
Eurobot 2027 table layout, in the `table` frame.

Frame: origin at the front-left (yellow) corner of the play area, X along the
3000 mm side towards blue, Y away from the audience, Z up. All values in
metres / radians. Source: rules OFFICIAL 1.0 and the official playmat
(see docs/RULES_2027.md).

The table is mirror-symmetric around x = 1.5 m, so everything that belongs to a
team is defined once for yellow and mirrored for blue. Write strategy code with
own() / opponent() and it works for both colours:

    from eurobot_navigation import field
    x, y, yaw = field.start_pose('blue')
    for q in field.QUARRIES: ...
    zone = field.own('blue', field.YELLOW_THRONE_ROOM)
"""

import math

TEAMS = ('yellow', 'blue')

TABLE_LENGTH = 3.0   # along X
TABLE_WIDTH = 2.0    # along Y
BORDER_HEIGHT = 0.07
BORDER_THICKNESS = 0.022

STONE_SIZE = (0.32, 0.11, 0.11)   # length, width, height
STONE_ARUCO_ID = 13
STONE_ARUCO_SIZE = 0.08           # black square side, on a 100 mm white label

# Rectangles are (x_min, y_min, x_max, y_max), defined for the yellow team.
YELLOW_THRONE_ROOM = (0.0, 0.75, 0.5, 1.25)          # start and finish area
YELLOW_STABLES = ((0.0, 0.0, 0.2, 0.3),              # SIMA start areas
                  (0.0, 1.7, 0.2, 2.0))

# Stone quarries at the start of the match: (x, y, yaw of the stones' long axis).
# Each one holds 3 stones lying flat and stacked. Shared by both teams.
QUARRIES = (
    (0.75, 0.055, 0.0), (1.25, 0.055, 0.0), (1.75, 0.055, 0.0), (2.25, 0.055, 0.0),
    (0.75, 1.945, 0.0), (1.25, 1.945, 0.0), (1.75, 1.945, 0.0), (2.25, 1.945, 0.0),
    (1.50, 0.50, math.pi / 2), (1.50, 1.50, math.pi / 2),
)
STONES_PER_QUARRY = 3

# Fixed ArUco markers printed on the playmat (DICT_4X4_50), used to correct the
# robot's position. id: (x, y) of the marker centre. Each is a 100 mm square
# lying flat, with its top edge facing +Y (away from the audience).
TABLE_MARKERS = {
    20: (0.6, 1.4),
    21: (2.4, 1.4),
    22: (0.6, 0.6),
    23: (2.4, 0.6),
}
TABLE_MARKER_SIZE = 0.10


def table_marker_corners(marker_id):
    """Corners of a fixed marker in the table frame, in ArUco order (TL, TR, BR, BL)."""
    x, y = TABLE_MARKERS[marker_id]
    h = TABLE_MARKER_SIZE / 2
    return ((x - h, y + h), (x + h, y + h), (x + h, y - h), (x - h, y - h))


def check_team(team):
    if team not in TEAMS:
        raise ValueError(f"team must be one of {TEAMS}, got {team!r}")
    return team


def mirror_point(x, y):
    """Mirror a point onto the other team's side."""
    return TABLE_LENGTH - x, y


def mirror_pose(x, y, yaw):
    """Mirror a pose onto the other team's side (heading flips left/right)."""
    return TABLE_LENGTH - x, y, math.atan2(math.sin(math.pi - yaw), math.cos(math.pi - yaw))


def mirror_rect(rect):
    x_min, y_min, x_max, y_max = rect
    return TABLE_LENGTH - x_max, y_min, TABLE_LENGTH - x_min, y_max


def own(team, yellow_rect):
    """A yellow-side rectangle, moved to `team`'s side."""
    return yellow_rect if check_team(team) == 'yellow' else mirror_rect(yellow_rect)


def opponent(team, yellow_rect):
    """A yellow-side rectangle, moved to the opponent's side."""
    return own('blue' if check_team(team) == 'yellow' else 'yellow', yellow_rect)


def start_pose(team):
    """Robot start pose (x, y, yaw): centre of the throne room, facing the opponent."""
    x_min, y_min, x_max, y_max = own(team, YELLOW_THRONE_ROOM)
    yaw = 0.0 if team == 'yellow' else math.pi
    return (x_min + x_max) / 2, (y_min + y_max) / 2, yaw
