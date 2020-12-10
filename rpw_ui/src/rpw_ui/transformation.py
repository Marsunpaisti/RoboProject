
SCALE_FACTR = 100
SCENE_W = 1000
SCENE_H = 1000

def world_to_scene(x, y):
    """
    input: gazebo coords in meters, origin in the center, x-axis points to left and y-axis up
    output: dict{'x': int, 'y': int} scene coords in pixels, origin top left corner, x-axis
    """
    # Scale
    x = x * SCALE_FACTR     # example. 0.65 m to 65 pixel
    y = y * SCALE_FACTR

    # Translation
    scene_w_center = SCENE_W / 2
    scene_h_center = SCENE_H / 2
    x = int( scene_w_center + x )   # 500 + 65 = 565 pixels
    y = int( scene_h_center - y )   # y inverted

    coords = dict()
    coords["x"] = x
    coords["y"] = y
    return coords