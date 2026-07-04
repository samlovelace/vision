# One camera's captured frame(s) + calibration, published by iris_camera.
# Mirrors iris_core's internal StampedCameraOutput minus the nav-derived
# global pose (T_G_C) - that composition happens downstream, not here.

std_msgs/Header header              # left frame's true capture timestamp; frame_id = camera id

sensor_msgs/Image left
sensor_msgs/Image right             # default/empty if not stereo
sensor_msgs/Image depth             # default/empty if not a depth camera
bool is_stereo
bool is_depth

float32[9] k                        # intrinsic matrix, row-major: fx 0 cx / 0 fy cy / 0 0 1
float32 near_m
float32 far_m
float32 baseline_m
uint32 img_width                    # configured target image size (for post-processing resizes downstream)
uint32 img_height

geometry_msgs/Transform s2v         # static sensor-to-vehicle extrinsic (constant per camera)
