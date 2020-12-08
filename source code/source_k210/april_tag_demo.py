"""
  Quadruped Robot MaixPy IDE sketch.
  3/10/2020 by Alexandros Petkos
  Updates available at https://github.com/maestrakos/warp

  The quadruped robot kinematics simulation or warp_simulation is placed under CC-BY.

  [source] This is the main file that manages AprilTag detection

  Comment Description:

  # comment

  #> used to explain the function of a line
  #: used to summurize the function of multiple lines

  === used for headers
  ::: used for sketch parts

  # ## used to explain the measurement unit of a variable
  # !! used for warnings
"""

import sensor, lcd, image, time, math

body_size = [30, 6, 18] ## {cm, cm, cm}

# these values need calibration based on the size of the tag you are using
# the tag size I used in the demo is w 10cm x h 10cm
cm_c_x = 2.2
cm_c_y = 0
cm_c_z = 6.5

def init_lcd():
    lcd.init();

def init_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QQVGA)  # aprilTag detection works with low resolutions
    sensor.skip_frames(60)
    sensor.set_vflip(1) # flips the image
    sensor.run(1)

def c_turn(x0, y0, z0):
    b = body_size[0]/2 + z0;
    c = x0;
    C = math.degrees(math.atan(c/b))
    return C

init_lcd();
init_sensor();
clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()
    tags = img.find_apriltags()
    if len(tags) > 0:
        tag = tags[0]
        img.draw_rectangle(tag.rect(), color = (255, 0, 0), thickness = 2)
        img.draw_cross(tag.cx(), tag.cy(), color = (255, 0, 0), thickness = 1)
        x_translation = tag.x_translation() * cm_c_x
        z_translation = tag.z_translation() * cm_c_z
        f = 0
        r = 0
        turn = -c_turn(x_translation, 0, -z_translation)
        args = (f, r, turn)
        print("%f %f %f" % args)
    lcd.display(img.replace(hmirror = True, vflip = True))
