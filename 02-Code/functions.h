#ifndef FUNCTIONS_H
#define FUNCTIONS_H

namespace SaturnWheel
{
  // Macros

  // Linear Interpolation
  // It is assumed that x2 > x > x1
  #define INTERP(x1, x2, y1, y2, x) (((y2-y1)/(x2-x1))*(x-x1)+y1)

  // Max Value
  #define MAX(x, y) ((y>x)?y:x)

  // Min Value
  #define MIN(x, y) ((y<x)?y:x)

  // Limit Value
  // Assumes that x2 > x1
  #define LIMIT(x1, x2, y) ((y>x2)?x2:((y<x1)?x1:y))

  // Calculation of gravity force
  #define F_GRAVITY(m, a) ((m)*(a))

  // Calculation of acceleration due to drag
  //

  class Model
  {
    static const float c_drag[2] = [0.12, 0.34]; // Table of Drag constants at closed and open positions
    static float m;                       // Mass of spacecraft                                 [kg]
    static float d_alt;                   // Altitude of spacecraft from center of planet       [m]
    static float u_air;                   // Velocity of surrounding air                        [m/s]
    static float a_gravity;               // Acceleration due to gravity                        [m/s^2]
    static float u_sc_cur[3];             // Current velocity of spacecraft                     [m/s]
    static float u_sc_req[3];             // Requested velocity of spacecraft                   [m/s]
    static float a_sc_cur[3];             // Current acceleration of spacecraft                 [m/s^2]
    static float a_sc_req[3];             // Requested acceleration of spacecraft               [m/s^2]
    static float f_sc_cur[3];             // Current forces acting on spacecraft                [N]
    static float t_air;                   // Temperature of surrounding air                     [Celsius]
    static float rho_cur;                 // Current density of surrounding air                 [kg/m^3]
    static float rho_prev;                // Previous density of surrounding air                [kg/m^3]
    static const float area_ref = 2.356;  // Reference area for drag equation                   [m^2]
    static float d_shutter_act_cur[5];    // Current position of shutter actuator semi-halves   [%]
                                            // Indices:
                                            // 0 - top
                                            // 1 - quad 1
                                            // 2 - quad 2
                                            // 3 - quad 3
                                            // 4 - quad 4
    static float d_shutter_act_req[5];    // Requested position of shutter actuator semi-halves [%]
    static float r_shutter_act_req[5];    // Requested rate for shutter to open/close           [%/s]
    static float c_drag_cur;              // Current drag coefficient                           [-]
    static float c_drag_req;              // Requested drag coefficient                         [-]
    static float v_top;                   // Volume of top of spacecraft                        [m^3]

    float getDensity(void);
    float getGravity(void);
    float getForceBuoy(void);
    float getForceGrav(void);
    float getForceDrag(float u, float c_drag);
    float getCoeffDrag(float u, float a);

  public:
    void doUpdate(void);
    void getAtm(void);
    void calcCurrIter(void);
    void calcReq(void);
    void calcNextIter(void);
  }
}

#endif