/* - ::: MODEL :::
 
 This sketch is part of warp_kinematics.
 [model] This file manages the model's 3D representation.
 
 */

class Model {
  Vector origin; // ## {mm, mm, mm}
  Vector[] p_joint_origin;

  Transform body_tranform; // ## {mm, mm, mm}, {deg, deg, deg}, {mm, mm, mm}

  Rotator body_rRotation = 
    new Rotator(0, 0, 0);   // ## {deg, deg, deg}
  Rotator[] joint_rRotation = {
    new Rotator(0, 45, 90), // ## {deg, deg, deg}
    new Rotator(0, 45, 90), // ## {deg, deg, deg}
    new Rotator(0, 45, 90), // ## {deg, deg, deg}
    new Rotator(0, 45, 90)  // ## {deg, deg, deg}
  };

  float bone_l; // ## mm

  /*
   ==============================
   MODEL PARAMETERS
   ==============================
   */

  final float bone_size = 10.0; // ## mm

  final color plane_color = color(#212020); //## (blackish) 
  /// If the color is black the plane will not be drawn.
  final color body_color = color(50, 150, 220); // ## (bluish)
  final color bone_color = color(40, 200, 110); // ## (white)
  final color joint_color = color(255, 255, 255); // ## (greenish)

  /*
   ::: SETUP :::
   */

  //: sets class variables :: parameters of _source(warp_simulation.pde)
  int init(Vector origin, Transform body_tranform, Vector[] p_joint_origin, float bone_length) {
    this.origin = origin;
    this.body_tranform = body_tranform;
    this.p_joint_origin = p_joint_origin;
    this.bone_l = bone_length;
    return 0;
  }

  /*
   ::: DRAW LOOP :::
   */

  void draw() {
    background(0);

    pushMatrix(); //> groups bodies
    draw_plane(origin.x(), origin.y(), origin.z(), 5000, 5000); //> draws a flat plane

    draw_body(body_tranform, body_rRotation); //> draws body
    model.body_rRotation = new Rotator(0, 0, 0);

    //: draws legs

    float x_offset = body_tranform.scl().x()/2; //> calculates the distance from the center of the body
    float z_offset = body_tranform.scl().z()/2; //> calculates the distance from the center of the body


    /* ----------------------------------------------------------------------
     Each leg takes the following parameters:
     
     1.  The x parameter contains the positive or negative (depending on whether the leg is positioned in front[-] or back[+])
     X distance from the center of the body and afterwards adds or subtracts the defined X offset.
     
     2. The y parameter contains the defined Y offset.
     
     3.  The z parameter contains the positive or negative (depending on whether the leg is positioned on the left[-] or right[+])
     Z distance from the center of the body and afterwards adds or subtracts the defined Z offset.
     
     4. angle1 contains the positive or negative (depending on whether the leg is positioned on the left[-] or right[+]) angle of the [parent] joint.
     5. angle2 contains the angle of the [child1] joint.
     6. angle3 contains the angle of the [child2] joint.
     
     In this case yaw, pitch & roll do not represent 
     the type of rotation but a stored angle.  
     --------------------------------------------------------------------- */

    draw_leg( +x_offset + p_joint_origin[0].x(), p_joint_origin[0].y(), - z_offset + p_joint_origin[0].z(), 
      -joint_rRotation[0].yaw(), joint_rRotation[0].pitch(), joint_rRotation[0].roll());

    draw_leg( -x_offset + p_joint_origin[1].x(), p_joint_origin[1].y(), - z_offset + p_joint_origin[1].z(), 
      -joint_rRotation[1].yaw(), joint_rRotation[1].pitch(), joint_rRotation[1].roll());

    draw_leg( -x_offset + p_joint_origin[2].x(), p_joint_origin[2].y(), + z_offset + p_joint_origin[2].z(), 
      joint_rRotation[2].yaw(), joint_rRotation[2].pitch(), joint_rRotation[2].roll());

    draw_leg( +x_offset + p_joint_origin[3].x(), p_joint_origin[3].y(), + z_offset + p_joint_origin[3].z(), 
      joint_rRotation[3].yaw(), joint_rRotation[3].pitch(), joint_rRotation[3].roll());

    popMatrix(); //> ungroups bodies
  }
  
  /*
   ::: [DRAW] FUNCTIONS :::
   */

  void draw_plane(float x, float y, float z, float _width, float _length) {
    translate(x, y, z); //> sets displacement
    fill(plane_color); //> sets the plane color

    if (plane_color != #000000) //> checks whether the given color isn't black
      box(_width, 1, _length); //> draws a rectangle
  }

  void draw_body(Transform transfrom, Rotator rRot) {
    translate(transfrom.pos().x(), transfrom.pos().y(), transfrom.pos().z()); //> sets displacement

    //: sets yaw, pitch, roll
    rotateY(radians(-transfrom.rot().yaw() - rRot.yaw()));
    rotateZ(radians(-transfrom.rot().pitch() - rRot.pitch()));
    rotateX(radians(-transfrom.rot().roll() - rRot.roll()));

    //: specifies the body's appearance
    strokeWeight(10); 
    stroke(body_color);
    noFill();

    box(transfrom.scl().x(), transfrom.scl().y(), transfrom.scl().z()); //> draws a rectangle
    noStroke(); //> disables stroke property
  }

  void draw_leg(float x, float y, float z, float angle1, float angle2, float angle3) {
    pushMatrix(); //> groups bodies
    origin(x, y, z, angle1); //> corresponds to shoulder joint 

    joint(0, 0, 0, angle2);  //>  corresponds to upper joint 
    bone(bone_l/2, 0, 0); //>  corresponds to upper bone 

    joint(bone_l/2, 0, 0, 180 - angle3); //>  corresponds to lower joint 
    bone(bone_l/2, 0, 0); //>  corresponds to lower joint 

    joint(bone_l/2, 0, 0, 0); //>  corresponds to foot
    popMatrix(); //> ungroups bodies
  }

  /*
   ::: [DRAW] MODELS :::
   */

  void origin(float x, float y, float z, float a) {
    translate(x, y, z); //> sets displacement
    rotateX(radians(a)); //> sets roll
  }

  void joint(float x, float y, float z, float a) {
    translate(x, y, z); //> sets displacement
    rotateZ(radians(a)); //> sets pitch

    fill(joint_color); //> sets the joint color
    sphere(bone_size); //> draws a sphere
  }

  //: bone, lower/upper bone (relative type of rotation: pitch)
  void bone(float x, float y, float z) {
    translate(x, y, z); //> sets displacement

    fill(bone_color); //> sets the bone color
    box(bone_l, bone_size, bone_size); //> draws a rectangle
  }
}
