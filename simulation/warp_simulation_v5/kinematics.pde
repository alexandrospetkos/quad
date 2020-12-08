/* - ::: KINEMATICS ::: //<>// //<>// //<>//
 
 This sketch is part of warp_kinematics.
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

  Vector step_extent; // ## {cm, cm}
  float frequency; // ## Hz

  float precision, bone; // ## mm

  float mm = 0.1;
  int sign(float num) {
    return int(num >= 0) - int(num < 0);
  }

  /*
   ==============================
   KINEMATICS PARAMETERS
   ==============================
   */

  int crawl_pattern[] = {2, 0, 1, 3};
  int crawl_succession[] = {2, 1, 3, 0};

  //: this array stores the (relative to the body) direction that each leg faces.
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
  int init(Transform body_transform, Vector[] p_joint_origin, Vector step_extent, 
    float precision, float bone, float frequency) {
    this.body_transform = body_transform;
    this.p_joint_origin = p_joint_origin;
    this.step_extent = step_extent;
    this.precision = precision;
    this.bone = bone;
    this.frequency = frequency;
    return 0;
  }

  /*
   ::: HANDLE LOOP :::
   */

  float c[] = new float[4];
  float c_iter[] = new float[4];

  int c_inv[] = new int[4];

  void handle(Vector2D direction, float turn, float period) {
    for (int l = 0; l < 4; l++) {
      float base = c_leg_base(90); //> stores the base of each leg

      Vector2D dir = new Vector2D(
        precision + direction.x() + turn * l_inv[l][1], 
        precision + direction.y() + turn * l_inv[l][0]);
      count_c(l, dir, period);

      Vector2D rDir = c_direction_ratio(dir); //> calls the clock function  
      Vector vector = new Vector(0, 0, 0); //> default leg coordinates

      //: these functions run for each leg and return a 3 dimensional vector that stores the desired leg position in cartesian coordinates
      if (state == 1 && (abs(dir.x) > precision || abs(dir.y) > precision)) {
        vector = trot_gait_func(new Vector2D(rDir.x() * c[l], l_inv[l][1] * rDir.y() * c[l]), 
          dir, boolean(l%2) ^ boolean(c_inv[l]%2));
      } else if (state == 2 && (abs(dir.x) > precision || abs(dir.y) > precision)) {
        vector = crawl_gait_func(new Vector2D(rDir.x() * c[l], l_inv[l][1] * rDir.y() * c[l]), 
          new Vector2D(dir.x(), l_inv[l][1] * dir.y()), l == crawl_pattern[int(c_inv[l])%4], int(c_inv[l] + crawl_succession[l])%4);
      } else if (state == 3) {
        vector = yaw_axis(l, turn*10);
        model.body_rRotation = new Rotator(turn*10, 0, 0);
      } else if (state == 4) {
        vector = pitch_roll_axis(l, base, new Rotator(0, direction.x()*10, direction.y()*10));
        model.body_rRotation = new Rotator(0, direction.x()*10, direction.y()*10);
      }      

      /// the 3 dimensional vector is converted through the k_model function into the three joint angles of each leg,
      model.joint_rRotation[l] = k_model(vrt_offset, hrz_offset, base, 
        0, 0, vector);
    }
  }

  void count_c(int inst, Vector2D dir, float period) {
    float w0 = step_extent.x *mm / (2 / max(abs(dir.x), abs(dir.y)));
    float a0 = (2 * w0) * (c_iter[inst] / round(frequency / period)) - w0;

    c[inst] = a0;
    c_iter[inst] += 1.f;

    if (c_iter[inst] > round(frequency / period)) {
      c[inst] = -w0; 
      c_iter[inst] = 1.f;

      c_inv[inst] += 1.f;
      if (c_inv[inst] > 31) c_inv[inst] = 0;
    }
  }


  /*
   ::: [KINEMATICS] FUNCTIONS :::
   */

  /*
     ::: GAIT FUNCTIONS :::
   */

  //: trot function
  Vector trot_gait_func(Vector2D c0, Vector2D dir, boolean inv) {
    float w0 = step_extent.x()*mm / 2 * dir.x();
    float l0 = step_extent.y()*mm * 4 * dir.y();
    float h0 = step_extent.z()*mm;

    if (inv == false) 
      c0 = new Vector2D(-c0.x(), -c0.y());

    float h1 = sqrt(abs((1- sq(c0.x()/w0) - sq(c0.y()/l0)) * sq(h0)));
    return new Vector(c0.x()/mm, c0.y()/mm, h1/mm * int(inv));
  }

  //: crawl function
  Vector crawl_gait_func(Vector2D c0, Vector2D dir, boolean inv, int i0) {
    float w0 = step_extent.x()*mm / 2 * dir.x();
    float l0 = step_extent.y()*mm * 4 * dir.y();
    float h0 = step_extent.z()*mm;

    if (inv == false) c0 = new Vector2D(-(c0.x()+(2*i0-2)*w0)/3, -(c0.y()+(2*i0-2)*(l0/8))/3);

    float h1 = sqrt(abs((1- sq(c0.x()/w0) - sq(c0.y()/l0)) * sq(h0)));
    return new Vector(c0.x()/mm, c0.y()/mm, h1/mm * int(inv));
  }

  /*
   ::: TRIGONOMETRIC FUNCTIONS :::
   */

  //: base calculation function
  float c_leg_base(float angle1) {
    return sin(radians(angle1/2)) * bone*2;
  }

  //: pitch-roll axis function
  Vector pitch_roll_axis(int leg, float base, Rotator sRot) {
    float w0 = body_transform.scl().x()/2 * l_inv[leg][0] + p_joint_origin[leg].x() - vrt_offset;
    float l0 = body_transform.scl().z()/2 + hrz_offset;

    float C0 = radians(sRot.pitch());
    float C1 = radians(sRot.roll()) * l_inv[leg][1];

    float a0 = sin(C0) * w0;
    float a1 = sin(C1) * l0;

    float d0 = (1 - cos(C0)) * -w0;
    float d1 = (1 - cos(C1)) * l0;

    float var0 = sqrt(sq(base + a0) + sq(d0));
    C0 += asin(d0/var0);

    float b0 = cos(C0) * var0;
    float c0 = sin(C0) * var0;

    float var1 = sqrt(sq(b0 - a1) + sq(d1));
    C1 += asin(d1/var1);

    float b1 = cos(C1) * var1;
    float c1 = sin(C1) * var1;

    return new Vector(c0, c1, base-b1);
  }

  //: yaw axis function
  Vector yaw_axis(int leg, float yaw) {
    float x = body_transform.scl().x()/2 - abs(p_joint_origin[leg].x()) - vrt_offset * l_inv[leg][0];
    float y = body_transform.scl().z()/2 + hrz_offset;
    float radius = sqrt(sq(x) + sq(y));
    float angle = asin(y/radius) - radians(yaw) * l_inv[leg][0] * l_inv[leg][1];

    float rX = (x-cos(angle)*radius) * l_inv[leg][0];
    float rY = sin(angle)*radius-y;
    return new Vector(rX, rY, 0);
  }

  //: direction ratio calculation function
  Vector2D c_direction_ratio(Vector2D dir) {
    float dirX = dir.x()/max(abs(dir.x()), abs(dir.y()));
    float dirY = dir.y()/max(abs(dir.x()), abs(dir.y()));
    return new Vector2D(dirX, dirY);
  }

  //: inverse kinematic algorithm
  Rotator k_model(float x0, float y0, float z0, 
    float pitch_offset, float roll_offset, Vector vec) {
    float x = x0 + vec.x(), y = y0 + vec.y(), z = z0 - vec.z();

    float b0 = sqrt(sq(x) + sq(y));
    float h0 = sqrt(sq(b0) + sq(z));

    float a0 = degrees(atan(x/z));
    float a1 = degrees(atan(y/z));

    return c_triangle(a0 + pitch_offset, a1 + roll_offset, h0);
  }

  //: final triangle calculation function
  Rotator c_triangle(float a0, float a1, float b0) {
    float angle1 = a1;
    float angle3 = degrees(asin((b0/2.0) / bone)) * 2;
    float angle2 = angle3/2 + a0;

    return new Rotator(angle1, angle2, angle3);
  }
}
