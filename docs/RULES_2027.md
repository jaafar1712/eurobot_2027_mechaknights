# Eurobot 2027 — "The Legend of Camelot" (rules summary)

Source: [Eurobot 2027 rules, OFFICIAL 1.0](https://www.eurobot.org/wp-content/uploads/2026/10/Eurobot2027_Rules_EN.pdf).
This is a working summary for the team; the official PDF always wins.

## Table

- Play area 3000 × 2000 mm, borders 70 mm high and 22 mm thick (RAL 7032).
- Simulation world frame (`arena_world.sdf`): origin at the front-left (yellow) corner, X along the 3000 mm side towards blue, Y away from the audience, table surface at z = 0.
- **Throne room** (start and finish area): 500 × 500 mm on each short side, in the team colour.
  Yellow is x 0–0.5 m, y 0.75–1.25 m; blue is x 2.5–3.0 m, y 0.75–1.25 m.
- **Stables** (SIMA start areas): 200 × 300 mm, in the corners.
- **Ramparts**: walls are 200 × 400 mm rectangles, towers are 200 mm diameter circles.
- **Castle courtyard**: the area enclosed by a team's ramparts, including the throne room.
- **Moats**: blue areas around the ramparts.
- Four fixed ArUco markers are printed on the playmat, two per castle.

## Game elements

| Element | Details |
| --- | --- |
| Stone | Cardboard box 320 × 110 × 110 mm, about 90 g. ArUco `DICT_4X4_50` ID **13** on each of the 4 long faces. 30 stones in total |
| Stone quarry | 10 quarries, each with 3 stones lying flat and stacked |
| Grail | Built by the team, larger than a 60 mm cube. Placed in the throne room before the match |
| Cannonball | 10 foam balls per team, 45 mm in diameter |
| Knight (SIMA) | Small independent robot that starts in the stable and may only leave it from second 85 |

## Actions and points

| Action | Points |
| --- | --- |
| **Build Camelot**: stone placed in a construction zone | 1 per stone |
| Wall: 3 stones lying flat and stacked, inside a wall area | 5 |
| Tower: 1 stone standing upright, inside a tower circle | 5 |
| Gate: 2 upright stones topped by 1 flat stone, inside a wall area (one gate per castle) | 20 |
| **I have the real one**: Grail in the castle | 5 |
| Grail elevation (level 1 = on 1 flat stone, 2 = 2 flat, 3 = 3 flat or 1 upright, 4 = on the robot roof, >30 cm) | 5 per level |
| **Return of the king**: main robot partly in its throne room at the end | 5 |
| Main robot fully in its throne room at the end | 10 |
| **To the attack** (from second 85): SIMA in an opposing moat | 5 per moat |
| A SIMA attacks an opposing SIMA | 5 |
| At least 1 cannonball in the opposing courtyard | 5 |
| At least 3 cannonballs in the opposing courtyard | +15 |

### Constraints that matter for the robot

- A stone counts in a zone if part of its vertical projection is inside the zone, or if it sits on a stone that already counts there.
- Each zone holds only one building of its type. A stone cannot count for two buildings.
- Up to 3 stones may be stored in the castle courtyard during the match.
- Elements still held by the robot at the end are not counted.
- Stones may be taken from the opposing castle only once every quarry is empty. Unfair play costs at least 50 points.
- Buildings and the Grail may go above the height limit only inside construction zones and the castle. Actuators must stay within the height limit at all times.
- Moving the opposing team's Grail is forbidden.
