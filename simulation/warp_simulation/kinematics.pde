/* - ::: KINEMATICS :::
 
 This sketch is part of warp(Waste Allocation & Recognition Project).
 [kinematics] This file manages the basic kinematic functions.
 
 [BACK] [LEFT], LOWER JOINT (0, 0) : servo00,
 UPPER JOINT (0, 1) : servo02, SHLDR JOINT (0, 2) :servo04
 
 [FRONT][LEFT], LOWER JOINT (1, 0) : servo15,
 UPPER JOINT (1, 1) : servo13, SHLDR JOINT (1, 2) :servo05
 
 [FRONT][RIGHT], LOWER JOINT (2, 0) : servo14,
 UPPER JOINT (2, 1) : servo12, SHLDR JOINT (2, 2) :servo07
 
 [BACK] [RIGHT], LOWER JOINT (3, 0) : servo01,
 UPPER JOINT (3, 1) : servo03, SHLDR JOINT (3, 2) :servo06
 
 */

class Kinematics {
  Transform body_transform; // ## {mm, mm, mm}, {deg, deg, deg}, {mm, mm, mm}
  Vector[] p_joint_origin; // {mm, mm, mm} , {mm, mm, mm}, {mm, mm, mm}

  Vector2D step_extent; // ## {cm, cm}
  float frequency; // ## Hz

  float min_extent, bone_l; // ## mm

  float mm = 0.1;
  int sign(float num) {
    return int(num >= 0) - int(num < 0);
  }

  /*
   ==============================
   KINEMATICS PARAMETERS
   ==============================
   */

  //: this array stores the (relative to the body) direction of each leg.
  final float l_inv[][] = {
    {+1.f, -1.f}, // ## {dir, dir}
    {-1.f, -1.f}, // ## {dir, dir}
    {-1.f, +1.f}, // ## {dir, dir}
    {+1.f, +1.f}  // ## {dir, dir}
  };

  /*
   ::: SETUP :::
   */

  //: sets class variables :: parameters of _source(warp_simulation.pde)
  int init(Transform body_transform, Vector[] p_joint_origin, Vector2D step_extent, 
    float min_extent, float bone_l, float frequency) {
    this.body_transform = body_transform;
    this.p_joint_origin = p_joint_origin;
    this.step_extent = new Vector2D(step_extent.x()*mm, step_extent.y()*mm); //> unit conversion :: mm to cm
    this.min_extent = min_extent;
    this.bone_l = bone_l;
    this.frequency = frequency;
    return 0;
  }

  /*
   ::: HANDLE LOOP :::
   */

  float c[] = new float[4];
  boolean inv[] = new boolean[4];

  void handle(Vector2D direction, float turn, float period) {
    for (int l = 0; l < 4; l++) {
      float base = c_leg_base(90);
      base += c_bal_base(l, new Rotator(body_transform.rot().yaw(), 
        body_transform.rot().pitch() + serial.pitch, 
        body_transform.rot().roll() + serial.roll));

      Vector2D dir = new Vector2D(
        min_extent + direction.x() + turn  * l_inv[l][1], 
        min_extent + direction.y() + turn  * l_inv[l][0]);
      count_c(l, dir, period);

      Vector2D rDir = c_direction_ratio(dir);

      Step forward = c_step(rDir.x() * c[l], dir.x(), vrt_offset, base, boolean(l%2) ^ inv[l]);
      Step right   = c_step(rDir.y() * c[l], dir.y(), hrz_offset, base, boolean(l%2) ^ inv[l]);

      model.body_rRotation = new Rotator(model.body_rRotation.yaw() + turn * abs(c[l])/10, serial.pitch, -serial.roll);

      model.joint_rRotation[l] = c_triangle(l, forward.getAngle() + serial.pitch, right.getAngle() + serial.roll, c_base_magnitude(forward, right));
    }
  }

  void count_c(int inst, Vector2D dir, float period) {
    float w0 = sq(step_extent.x / (2 / max(abs(dir.x), abs(dir.y))));

    c[inst] += (sqrt(w0) * 2) / (frequency / period);

    if (c[inst]  > sqrt(w0)) {
      c[inst]  = -(sqrt(w0));
      inv[inst]  = !inv[inst];
    }
  }

  /*
   ::: [KINEMATICS] FUNCTIONS :::
   
   All the functions are explained with images 
   and graphs in the project's github page:
   
   https://github.com/maestrakos/warp fix exact link
   
   */

  //: base calculation function
  float c_leg_base(float angle1) {
    return sin(radians(angle1/2)) * bone_l*2;
  }

  //: balance function
  float c_bal_base(int leg, Rotator sRot) {
    float b0 = sin(radians(sRot.roll())) * body_transform.scl().z()/2 * l_inv[leg][1];
    float b1 = sin(radians(sRot.pitch())) * (body_transform.scl().x()/2 - abs(p_joint_origin[leg].x())) * l_inv[leg][0];
    return b0 + b1;
  }

  //: direction ratio calculation function
  Vector2D c_direction_ratio(Vector2D dir) {
    float dirX = dir.x()/max(abs(dir.x()), abs(dir.y()));
    float dirY = dir.y()/max(abs(dir.x()), abs(dir.y()));
    return new Vector2D(dirX, dirY);
  }

  //: step function
  Step c_step(float c0, float dir, float x, float y, boolean inv) {
    float w0 = sq(step_extent.x()/ (2/ dir));
    float h0 = w0/sq(step_extent.y());

    if (inv == false) c0 = -c0;
    float step_f = sqrt(abs((w0 - sq(c0))/h0))/mm;

    float base = sqrt(sq(x + c0/mm) + sq(y - step_f * int(inv)));
    float angle = degrees(asin((x + c0/mm)/base));
    return new Step(base, angle);
  }

  //: base magnitude calculation;
  float c_base_magnitude(Step forward, Step right) {
    return sqrt(sq(sin(radians(right.getAngle()))*right.getBase()) + sq(forward.getBase()));
  }

  //: final triangle calculation function
  Rotator c_triangle(int leg, float pitch_offset, float roll_offset, float l_base) {
    float angle1 = roll_offset * l_inv[leg][1];
    float angle3 = degrees(asin((l_base/2.0) / bone_l)) * 2;
    float angle2 = angle3/2 + pitch_offset;

    return new Rotator(angle1, angle2, angle3);
  }
}
