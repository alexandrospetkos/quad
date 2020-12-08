/* - ::: SERIAL :::
 
 This sketch is part of warp_kinematics.
 [serial] This file manages serial communication & data.
 
 */

import processing.serial.*; //> importing a library to handle serial communication

class SerialCom {
  Serial myPort; //> create a serial object

  /*
   ::: SETUP :::
   */

  int init(warp_simulation_v5 parent, String com_port, int baud_rate) {
    try {
      myPort = new Serial(parent, com_port, baud_rate); //> initializes Serial :: myPort Object
      myPort.bufferUntil('\n');
    }
    catch (Exception e) {
      print(e + "\n"); //> prints error message
      return 1; //> returns 0 :: failed
    }

    return 0; //> returns 1 :: success
  }

  /*
   ::: CLASS FUNCTIONS :::
   */

  String data = ""; //> used to handle incoming serial data

  void serialEvent(Serial myPort) { //> handles incoming data :: called by _source(warp_simulation.pde)
    data = myPort.readStringUntil('\n'); //> reads incoming data until a new line

    if (data != null) 
      data = trim(data); //> removes whitespaces from string
  }
}
