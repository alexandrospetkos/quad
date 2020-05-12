/* - ::: SERIAL :::
 
 This sketch is part of warp(Waste Allocation & Recognition Project).
 [serial] This file manages serial communication & data.
 
 */

import processing.serial.*; //> importing a library to handle serial communication

class SerialCom {
  Serial myPort; //> create a serial object

  /*
   ::: SETUP :::
   */

  int init(warp_simulation parent, String com_port, int baud_rate) {
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
  Vector2D dir = new Vector2D(0, 0);
  float roll, pitch, yaw; //> filtered output

  void serialEvent(Serial myPort) { //> handles incoming data :: called by _source(warp_simulation.pde)
    data = myPort.readStringUntil('\n'); //> reads incoming data until a new line

    if (data != null) {
      data = trim(data); //> removes whitespace characters from string

      //: splits string into multiple segments and sets :: yaw, pitch & roll
      String items[] = split(data, '/');
      if (items.length > 1) {
        yaw = float(items[0]);
        pitch = float(items[2]);
        roll = float(items[1]);
        dir = new Vector2D(float(items[3]), float(items[4]));
      }
    }
  }
}
