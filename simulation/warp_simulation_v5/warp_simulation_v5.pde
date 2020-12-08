/* //<>// //<>//
 Quadruped robot kinematics simulation/demonstration sketch.
 3/10/2020 by Alexandros Petkos
 Updates available at https://github.com/maestrakos/warp
 
 The quadruped robot kinematics simulation or warp_simulation 
 
 This sketch is part of warp_kinematics.
 
 [Base Class] This is the main file that manages [kinematics], [model] & [serial]. 
 All the important parameters are set in this class.
 
 Comment Description:
 
 /// comment
 
 //> used to explain the function of a line
 //: used to summurize the function of multiple lines
 
 === used for headers
 ::: used for sketch parts
 
 // ## used to explain the measurement unit of a variable
 // !! used for warnings
 
 User Input:
 
 !! Ensure that you are using an English keyboard, the library doesn't recognize other keyboards.
 
 ¬ Navigation controls:
 mouse, keyboard ['W'], ['A'], ['S'], ['D'], ['Q'], ['E'].
 
 ¬ Simulation controls:
 keyboard [KEY_UP], [KEY_DOWN], [KEY_LEFT], [KEY_RIGHT], ['['], [']'], ['1'], ['2'], ['3'], ['4'], ['5'].
 
 */

import queasycam.*; //> imports a library to handle camera control
QueasyCam cam; //> creates a QueasyCam object 

Model model; //> creates a Model object
Kinematics kinematics; //> creates a Kinematics object
SerialCom serial; //> creates a SerialCom object

/*
 ==============================
 SIMULATION PARAMETERS
 ==============================
 */

/// This Class
final Vector origin = new Vector(width/2, height/2, 0); // ## {mm, mm, mm}
final float camera_sensitivity = 0.5; /// stores the mouse sensitivity coefficient of the camera
final float frequency = 70; // ## Hz
final int input = 1; // !! Replace with the input you are using, (0) none, (1) keyboard, (2) serial

/// Model Class

//: stores the location, rotation and scale of the main [body]
final Transform body_transform = new Transform(
  new Vector(0, -150, 0), // ## {mm, mm, mm}
  new Rotator(0, 0, 0), // ## {deg, deg, deg}
  new Vector(300, 40, 180) // ## {mm, mm, mm}
  );

//: stores the parent joint location relative to the [body]
final Vector[] p_joint_origin = {
  new Vector(-50, 0, 0), // ## {mm, mm, mm}
  new Vector(+50, 0, 0), // ## {mm, mm, mm}
  new Vector(+50, 0, 0), // ## {mm, mm, mm}
  new Vector(-50, 0, 0)  // ## {mm, mm, mm}
};
final float bone_length = 105; // ## mm

/// Kinematics Class

//: high level parameters for the step function
final Vector step_extent = new Vector(40, 40, 20); // ## {mm, mm}
float vrt_offset = - 14.0; // ## mm
float hrz_offset = - 6.0; // ## mm

final float precision = 0.001; // ## mm

/// Serial Class
final String com_port = "COM9"; // !! Replace with the port you are using
final int baud_rate = 115200; // ## bps

/*
 ::: SETUP :::
 */

void setup() {
  size(1080, 720, P3D); //> sets window size and enables 3D Graphics

  lights(); //> enables lighting
  noCursor(); //> hides mouse cursor
  frameRate(frequency); //> sets draw() function frequency

  //: initialize classes
  init_camera();
  init_model();
  init_kinematics();
  init_serial();
}

/*
 ::: DRAW LOOP :::
 */

//: high level variables -> kinematics
Vector2D direction = new Vector2D(0, 0); //> indicates the direction of tranlation
float turn = 0.0; //> indicates the direction of rotation(yaw)
float state = 0; //> indicates the type of movement, (0) idle, (1) trot, (2) crawl, (3) yaw-rotation, (4) pitch-roll-rotation

float period = 3.2; //> indicates the iterations of the kinematics class every second

void draw() {
  kinematics.handle(direction, turn, period); //> calls handle -> kinematics class
  model.draw(); //> calls draw -> model class

  this.handle_serial_input(); //> handles serial input, if input = 2
}

//: callback function used to detect keyboard input
void keyPressed() {
  if (input == 1) {
    switch(keyCode) {
    case UP:
      direction = new Vector2D(direction.x()+0.25, direction.y()); //> increases the value of direction in the x axis
      break;
    case DOWN:
      direction = new Vector2D(direction.x()-0.25, direction.y()); //> decreases the value of direction in the x axis
      break;
    case LEFT:
      direction = new Vector2D(direction.x(), direction.y()+0.25); //> increases the value of direction in the y axis
      break;
    case RIGHT:
      direction = new Vector2D(direction.x(), direction.y()-0.25); //> decreases the value of direction in the y axis
      break;

    case '[':
      turn += 0.25; //> increases the value of turn
      break;
    case ']':
      turn -= 0.25; //> decreases the value of turn
      break;

    case '1':
      state = 0; //> sets state to idle
      break;
    case '2':
      state = 1; //> sets state to trot
      break;
    case '3':
      state = 2; //> sets state to crawl
      break;
    case '4':
      state = 3; //> sets state to yaw-rotation
      break;
    case '5':
      state = 4; //> sets state to pitch-roll-rotation
      break;

    default:
      return;
    }

    println("x:" + direction.x() + " y:" + direction.y() + " turn:" + turn + " state: " + state); //> prints the new values of direction, turn & state
  }
}

//: call function used to handle serial input
void handle_serial_input() {
  if (input == 2) {
    direction = new Vector2D(
      inter(direction.x(), (serial.data.charAt(1) - 53)/2.0, 0.03), //> linear interpolation, to the desired x-direction
      inter(direction.y(), (serial.data.charAt(0) - 53)/4.0, 0.03)); //> linear interpolation, to the desired y-direction
    turn = inter(turn, (serial.data.charAt(2) - 53)/2.0, 0.03); //> linear interpolation, to the desired turn
    state = serial.data.charAt(4) - 48; //> sets state, to the desired state
  }
}

//: linear interpolation function
float inter(float in, float en, float pl) {
  if (in < en) {
    return ((in*1000) + (pl*1000))/1000;
  } else if (in > en) {
    return ((in*1000) - (pl*1000))/1000;
  } else return in;
}

/*
     ::: INIT FUNCTIONS :::
 */

void init_camera() {
  cam = new QueasyCam(this); //> new QueasyCam object
  cam.sensitivity = camera_sensitivity; //> sets camera sensitivity
}

void init_model() {
  model = new Model(); //> new Model object
  int e = model.init(origin, body_transform, p_joint_origin, bone_length); //> transfers parameters to model class
  //: checks whether the object initialization was successful
  if (e == 0) print("[model] ::initialized::\n");
  else print("[model] ::failed::\n");
}

void init_kinematics() {
  kinematics = new Kinematics(); //> new Kinematics object
  int e = kinematics.init(body_transform, p_joint_origin, step_extent, precision, bone_length, frequency); //>  transfers parameters to kinematics class
  //: checks whether the object initialization was successful
  if (e == 0) print("[kinematics] ::initialized::\n");
  else print("[kinematics] ::failed::\n");
}

void init_serial() {
  serial = new SerialCom(); // :: new Serialcom object
  int e = serial.init(this, com_port, baud_rate); //> initializes serial class and sets the communication port & baud rate
  //: checks whether the object initialization was successful
  if (e == 0) print("[serial] ::initialized::::\n");
  else print("[serial] ::failed::\n");
}

/*
 ::: EVENT CALLBACKS :::
 */

//: When data is received call the data handling function -> serial class 
void serialEvent(Serial myPort) { 
  serial.serialEvent(myPort); //> calls data handling function -> serial class
}
