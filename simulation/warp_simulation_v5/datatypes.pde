/* - ::: DATATYPES :::
 
 This sketch is part of warp_kinematics.
 [datatypes] This file contains data types.
 
 */

/*
 ::: TRANFORM, DATA TYPE :::
 */

final class Transform {
  private final Vector pos;
  private final Rotator rot;
  private final Vector scl;

  public Transform(Vector pos, Rotator rot, Vector scl) {
    this.pos = pos;
    this.rot = rot;
    this.scl = scl;
  }

  public Vector pos() {
    return pos;
  }

  public Rotator rot() {
    return rot;
  }

  public Vector scl() {
    return scl;
  }
}

/*
 ::: VECTOR, DATA TYPE :::
 */

final class Vector {
  private final float x, y, z;

  public Vector(float x, float y, float z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  public float x() {
    return x;
  }

  public float y() {
    return y;
  }

  public float z() {
    return z;
  }
}

/*
 ::: 2D VECTOR, DATA TYPE :::
 */

final class Vector2D {
  private final float x, y;

  public Vector2D(float x, float y) {
    this.x = x;
    this.y = y;
  }

  public float x() {
    return x;
  }

  public float y() {
    return y;
  }
}

/*
 ::: ROTATOR, DATA TYPE :::
 */

final class Rotator {
  private final float yaw, pitch, roll;

  public Rotator(float yaw, float pitch, float roll) {
    this.yaw = yaw;
    this.pitch = pitch;
    this.roll = roll;
  }

  public float yaw() {
    return yaw;
  }

  public float pitch() {
    return pitch;
  }

  public float roll() {
    return roll;
  }
}
