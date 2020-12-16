
SCALE_FACTR = 100
SCENE_W = 1000
SCENE_H = 1000

def world_to_scene(x, y):
    """
    input: world coords in meters, origin in the center, x-axis points to left and y-axis up
    output: dict{'x': int, 'y': int} scene coords in pixels, origin top left corner, x-axis left, y-axis down
    """
    # Scale
    x = x * SCALE_FACTR     # example. 0.65 m to 65 px
    y = y * SCALE_FACTR

    # Translation
    scene_w_center = SCENE_W / 2    # 1000 px to 500 px
    scene_h_center = SCENE_H / 2
    x = int( scene_w_center + x )   # 500 + 65 = 565 px
    y = int( scene_h_center - y )   # y inverted

    coords = dict()
    coords["x"] = x
    coords["y"] = y
    return coords

def scene_to_world(x, y):
    """
    input: scene coords in pixels, origin top left corner, x-axis points to left and y-axis down
    output: dict{'x': int, 'y': int} world coords in meters, origin in the center, x-axis points to left and y-axis up
    """
    # Scale
    x = x / SCALE_FACTR     # example. 565 px to 5.65 m
    y = y / SCALE_FACTR

    # Translation
    scene_w_center = SCENE_W / 2 / SCALE_FACTR      # 1000 px / 2 / 100 = 5 m
    scene_h_center = SCENE_H / 2 / SCALE_FACTR
    x = float( x - scene_w_center )                 # 5.65 m - 5 m = 0.65 m
    y = float( -(y - scene_h_center) )                 # y inverted

    coords = dict()
    coords["x"] = x
    coords["y"] = y
    return coords