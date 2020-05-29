#define mm 0.1 /// millimeter

int sign(float num) {
  return int(num >= 0) - int(num < 0);
}

class datatypes {
  public:
    /* ::: STEP, DATA TYPE ::: */
    struct Step {
      float base;
      float angle;
    };

    /* ::: VECTOR, DATA TYPE ::: */
    struct Vector {
      float x;
      float y;
      float z;
    };

    /* ::: 2D VECTOR, DATA TYPE ::: */
    struct Vector2D {
      float x;
      float y;
    };

    /* ::: ROTATOR, DATA TYPE ::: */
    struct Rotator {
      float yaw;
      float pitch;
      float roll;
    };

    /* ::: TRANFORM, DATA TYPE ::: */
    struct Transform {
      Vector pos;
      Rotator rot;
      Vector scl;
    };
};
