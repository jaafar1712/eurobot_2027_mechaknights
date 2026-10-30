# One stone seen by the camera (Eurobot 2027), from the ArUco marker (ID 13)
# on one of its long faces.

# Stone centre in the frame of the array's header (normally `table`)
geometry_msgs/Point position

# Height of the stone's top face above the table (m): where a gripper coming
# from above makes contact. Equal to position.z + half the stone height for a
# stone lying flat.
float64 top_z

# true if no stone was detected on top of this one in the same frame (it is the
# one a gripper can take from above)
bool is_top_of_stack

# Direction of the stone's long axis in that frame (radians). A stone is
# symmetric, so yaw and yaw + pi describe the same stone.
float64 yaw

# Position in the stack, counted from the table: 1 = on the table, 2 = on one
# stone, 3 = on two stones
int32 level

# true if the marker faces up (stone lying, seen from above), false if it is on
# a side face
bool seen_from_top

# Relative to the robot (base_link): distance in metres and bearing in degrees
# (0 = straight ahead, positive = to the left)
float64 distance
float64 bearing

# Image-space centre of the marker (pixels), for debugging
float64 pixel_x
float64 pixel_y
