#include functions.h

using namespace SaturnWheel;

Model::Model() : dt(0.5), m(1.0), totTime(0)
{
}

Model::~Model()
{
}

// Calculation of air density
float getDensity()
{
  // Values from source
}

// Calculation of top spacecraft
float getTopVolume()
{
  return INTERP(0.0, 1.0, v_top_min, v_top_max, d_shutter_act_cur[0])
}

// Calculation of acceleration due to gravity
// Using Newton's law of universal gravitation as an estimate
// F = G * ((m1 * m2) / r^2)
float getGravity()
{
  return (6.674*(10^-11)*5.683*(10^26))/(d_alt^2);
}

// Calculate the lift force due to changing air density
//
// F = (rho1 - rho2) * g * V
float getForceBuoy()
{
  float f = 0.0;

  // Check if buoyancy occurs
  if( rho_cur < rho_prev )
  {
    f = (rho_prev-rho_cur)*a_gravity*v_top;
  }

  return f;
}

// Calculate gravity force
float getForceGrav()
{
  return m*a_gravity;
}

// Calculation of drag force
//
// The drag force is determined using the equation
// F = 0.5 * rho * u^2 * area_ref * c_drag
float getForceDrag(float u, float c_drag)
{
  float f = 0.0;

  if( u > 0.0)
  {
    f = (0.5*rho_cur*(u^2)*area_ref*c_drag);
  }
  if( u < 0.0)
  {
    f = -1.0*(0.5*rho_cur*(u^2)*area_ref*c_drag);
  }
  
  return f;
}

// Calculation of drag coefficient
float getCoeffDrag(float u, float a)
{
  return (m*a)/(0.5*rho_cur*(u^2)*area_ref);
}

// Update surrounding atmospheric values and execute functionalities
void Model::doUpdate()
{
  // Update with new values for start of time step
  getAtm();
  calcCurrIter();

  // Calculate new target values, commanded internally or externally
  calcReq();

  // Calculate values for next time iteration
  calcNextIter();
}

// Get atmospheric values and update atmospheric variables for this time iteration
void Model::getAtm()
{
  u_air = getU();
  t_air = getTemp();
  d_alt = getAltitude();
  a_gravity = getGravity();
}

// Calculates velocity and acceleration of spacecraft
void Model::calcCurrIter()
{
  totTime += dt;
  m = getMass();
  rho_cur = getDensity();
  v_top = getTopVolume();

  // Frontal Forces (Fx)
  f_sc_cur[0] = getForceDrag(u_sc_cur[0], c_drag_cur[0]);

  // Altitudinal forces (Fz)
  f_sc_cur[2] = getForceBuoy()
                + getForceDrag(u_sc_cur[2], c_drag_cur[2])
                - getForceGrav();

  // Resultant accelerations
  for( int i=0; i<3; i++ )
  {
    a_sc_cur[i] = f_sc_cur[i]/m;
  }
}

// Calculations of requested variables
void Model::calcReq()
{
  static const float t_vel_change = 1.0; // Tune for acceleration extremeties

  for( int i=0; i<3; i++)
  {
    // If requested from user, use inputs from user rather than calculated inputs
    if( req_vel_cntr_prev[i] > req_vel_cntr[i] )
    {
      u_sc_req[i] = user_input_vel[i];
    }
    else
    {
      u_sc_req[i] = u_sc_cur[i];
    }

    a_sc_req[i] = (u_sc_req[i] - u_sc_cur[i]) / t_vel_change;

    // Drag forces
    c_drag_req[i] = LIMIT(c_drag[i][0], c_drag[i][1], getCoeffDrag(u_sc_req[i], a_sc_req[i]));
    d_shutter_act_req[i] = INTERP(c_drag[i][0], c_drag[i][1], 0.0, 1.0, c_drag_req[i]);
  }
}

// Calculations for next iteration
void Model::calcNextIter()
{
  static const float r_shutter_act = 0.05; // Rate of speed shutter opens/closes [%/s]

  for( int i=0; i<3; i++)
  {
    if( d_shutter_act_req[i] > d_shutter_act_cur[i])
    {
      d_shutter_act_cur[i] = MIN(d_shutter_act_req[i], d_shutter_act_cur[i]+r_shutter_act*dt);
    }
    else
    {
      d_shutter_act_cur[i] = MAX(d_shutter_act_req[i], d_shutter_act_cur[i]-r_shutter_act*dt);
    }

    c_drag_cur[i] = getCoeffDrag(u_sc_cur[i], a_sc_cur[i]);
  }

  rho_prev = rho_cur;
}