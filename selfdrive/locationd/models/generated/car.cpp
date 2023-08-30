#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1285813334012964983) {
   out_1285813334012964983[0] = delta_x[0] + nom_x[0];
   out_1285813334012964983[1] = delta_x[1] + nom_x[1];
   out_1285813334012964983[2] = delta_x[2] + nom_x[2];
   out_1285813334012964983[3] = delta_x[3] + nom_x[3];
   out_1285813334012964983[4] = delta_x[4] + nom_x[4];
   out_1285813334012964983[5] = delta_x[5] + nom_x[5];
   out_1285813334012964983[6] = delta_x[6] + nom_x[6];
   out_1285813334012964983[7] = delta_x[7] + nom_x[7];
   out_1285813334012964983[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4976523369423083376) {
   out_4976523369423083376[0] = -nom_x[0] + true_x[0];
   out_4976523369423083376[1] = -nom_x[1] + true_x[1];
   out_4976523369423083376[2] = -nom_x[2] + true_x[2];
   out_4976523369423083376[3] = -nom_x[3] + true_x[3];
   out_4976523369423083376[4] = -nom_x[4] + true_x[4];
   out_4976523369423083376[5] = -nom_x[5] + true_x[5];
   out_4976523369423083376[6] = -nom_x[6] + true_x[6];
   out_4976523369423083376[7] = -nom_x[7] + true_x[7];
   out_4976523369423083376[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1708338526219741257) {
   out_1708338526219741257[0] = 1.0;
   out_1708338526219741257[1] = 0;
   out_1708338526219741257[2] = 0;
   out_1708338526219741257[3] = 0;
   out_1708338526219741257[4] = 0;
   out_1708338526219741257[5] = 0;
   out_1708338526219741257[6] = 0;
   out_1708338526219741257[7] = 0;
   out_1708338526219741257[8] = 0;
   out_1708338526219741257[9] = 0;
   out_1708338526219741257[10] = 1.0;
   out_1708338526219741257[11] = 0;
   out_1708338526219741257[12] = 0;
   out_1708338526219741257[13] = 0;
   out_1708338526219741257[14] = 0;
   out_1708338526219741257[15] = 0;
   out_1708338526219741257[16] = 0;
   out_1708338526219741257[17] = 0;
   out_1708338526219741257[18] = 0;
   out_1708338526219741257[19] = 0;
   out_1708338526219741257[20] = 1.0;
   out_1708338526219741257[21] = 0;
   out_1708338526219741257[22] = 0;
   out_1708338526219741257[23] = 0;
   out_1708338526219741257[24] = 0;
   out_1708338526219741257[25] = 0;
   out_1708338526219741257[26] = 0;
   out_1708338526219741257[27] = 0;
   out_1708338526219741257[28] = 0;
   out_1708338526219741257[29] = 0;
   out_1708338526219741257[30] = 1.0;
   out_1708338526219741257[31] = 0;
   out_1708338526219741257[32] = 0;
   out_1708338526219741257[33] = 0;
   out_1708338526219741257[34] = 0;
   out_1708338526219741257[35] = 0;
   out_1708338526219741257[36] = 0;
   out_1708338526219741257[37] = 0;
   out_1708338526219741257[38] = 0;
   out_1708338526219741257[39] = 0;
   out_1708338526219741257[40] = 1.0;
   out_1708338526219741257[41] = 0;
   out_1708338526219741257[42] = 0;
   out_1708338526219741257[43] = 0;
   out_1708338526219741257[44] = 0;
   out_1708338526219741257[45] = 0;
   out_1708338526219741257[46] = 0;
   out_1708338526219741257[47] = 0;
   out_1708338526219741257[48] = 0;
   out_1708338526219741257[49] = 0;
   out_1708338526219741257[50] = 1.0;
   out_1708338526219741257[51] = 0;
   out_1708338526219741257[52] = 0;
   out_1708338526219741257[53] = 0;
   out_1708338526219741257[54] = 0;
   out_1708338526219741257[55] = 0;
   out_1708338526219741257[56] = 0;
   out_1708338526219741257[57] = 0;
   out_1708338526219741257[58] = 0;
   out_1708338526219741257[59] = 0;
   out_1708338526219741257[60] = 1.0;
   out_1708338526219741257[61] = 0;
   out_1708338526219741257[62] = 0;
   out_1708338526219741257[63] = 0;
   out_1708338526219741257[64] = 0;
   out_1708338526219741257[65] = 0;
   out_1708338526219741257[66] = 0;
   out_1708338526219741257[67] = 0;
   out_1708338526219741257[68] = 0;
   out_1708338526219741257[69] = 0;
   out_1708338526219741257[70] = 1.0;
   out_1708338526219741257[71] = 0;
   out_1708338526219741257[72] = 0;
   out_1708338526219741257[73] = 0;
   out_1708338526219741257[74] = 0;
   out_1708338526219741257[75] = 0;
   out_1708338526219741257[76] = 0;
   out_1708338526219741257[77] = 0;
   out_1708338526219741257[78] = 0;
   out_1708338526219741257[79] = 0;
   out_1708338526219741257[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_48359745169696250) {
   out_48359745169696250[0] = state[0];
   out_48359745169696250[1] = state[1];
   out_48359745169696250[2] = state[2];
   out_48359745169696250[3] = state[3];
   out_48359745169696250[4] = state[4];
   out_48359745169696250[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_48359745169696250[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_48359745169696250[7] = state[7];
   out_48359745169696250[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6580537581630503009) {
   out_6580537581630503009[0] = 1;
   out_6580537581630503009[1] = 0;
   out_6580537581630503009[2] = 0;
   out_6580537581630503009[3] = 0;
   out_6580537581630503009[4] = 0;
   out_6580537581630503009[5] = 0;
   out_6580537581630503009[6] = 0;
   out_6580537581630503009[7] = 0;
   out_6580537581630503009[8] = 0;
   out_6580537581630503009[9] = 0;
   out_6580537581630503009[10] = 1;
   out_6580537581630503009[11] = 0;
   out_6580537581630503009[12] = 0;
   out_6580537581630503009[13] = 0;
   out_6580537581630503009[14] = 0;
   out_6580537581630503009[15] = 0;
   out_6580537581630503009[16] = 0;
   out_6580537581630503009[17] = 0;
   out_6580537581630503009[18] = 0;
   out_6580537581630503009[19] = 0;
   out_6580537581630503009[20] = 1;
   out_6580537581630503009[21] = 0;
   out_6580537581630503009[22] = 0;
   out_6580537581630503009[23] = 0;
   out_6580537581630503009[24] = 0;
   out_6580537581630503009[25] = 0;
   out_6580537581630503009[26] = 0;
   out_6580537581630503009[27] = 0;
   out_6580537581630503009[28] = 0;
   out_6580537581630503009[29] = 0;
   out_6580537581630503009[30] = 1;
   out_6580537581630503009[31] = 0;
   out_6580537581630503009[32] = 0;
   out_6580537581630503009[33] = 0;
   out_6580537581630503009[34] = 0;
   out_6580537581630503009[35] = 0;
   out_6580537581630503009[36] = 0;
   out_6580537581630503009[37] = 0;
   out_6580537581630503009[38] = 0;
   out_6580537581630503009[39] = 0;
   out_6580537581630503009[40] = 1;
   out_6580537581630503009[41] = 0;
   out_6580537581630503009[42] = 0;
   out_6580537581630503009[43] = 0;
   out_6580537581630503009[44] = 0;
   out_6580537581630503009[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6580537581630503009[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6580537581630503009[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6580537581630503009[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6580537581630503009[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6580537581630503009[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6580537581630503009[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6580537581630503009[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6580537581630503009[53] = -9.8000000000000007*dt;
   out_6580537581630503009[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6580537581630503009[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6580537581630503009[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6580537581630503009[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6580537581630503009[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6580537581630503009[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6580537581630503009[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6580537581630503009[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6580537581630503009[62] = 0;
   out_6580537581630503009[63] = 0;
   out_6580537581630503009[64] = 0;
   out_6580537581630503009[65] = 0;
   out_6580537581630503009[66] = 0;
   out_6580537581630503009[67] = 0;
   out_6580537581630503009[68] = 0;
   out_6580537581630503009[69] = 0;
   out_6580537581630503009[70] = 1;
   out_6580537581630503009[71] = 0;
   out_6580537581630503009[72] = 0;
   out_6580537581630503009[73] = 0;
   out_6580537581630503009[74] = 0;
   out_6580537581630503009[75] = 0;
   out_6580537581630503009[76] = 0;
   out_6580537581630503009[77] = 0;
   out_6580537581630503009[78] = 0;
   out_6580537581630503009[79] = 0;
   out_6580537581630503009[80] = 1;
}
void h_25(double *state, double *unused, double *out_6097011765287479957) {
   out_6097011765287479957[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5605603257303256240) {
   out_5605603257303256240[0] = 0;
   out_5605603257303256240[1] = 0;
   out_5605603257303256240[2] = 0;
   out_5605603257303256240[3] = 0;
   out_5605603257303256240[4] = 0;
   out_5605603257303256240[5] = 0;
   out_5605603257303256240[6] = 1;
   out_5605603257303256240[7] = 0;
   out_5605603257303256240[8] = 0;
}
void h_24(double *state, double *unused, double *out_3578546623177939744) {
   out_3578546623177939744[0] = state[4];
   out_3578546623177939744[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3379895473324387678) {
   out_3379895473324387678[0] = 0;
   out_3379895473324387678[1] = 0;
   out_3379895473324387678[2] = 0;
   out_3379895473324387678[3] = 0;
   out_3379895473324387678[4] = 1;
   out_3379895473324387678[5] = 0;
   out_3379895473324387678[6] = 0;
   out_3379895473324387678[7] = 0;
   out_3379895473324387678[8] = 0;
   out_3379895473324387678[9] = 0;
   out_3379895473324387678[10] = 0;
   out_3379895473324387678[11] = 0;
   out_3379895473324387678[12] = 0;
   out_3379895473324387678[13] = 0;
   out_3379895473324387678[14] = 1;
   out_3379895473324387678[15] = 0;
   out_3379895473324387678[16] = 0;
   out_3379895473324387678[17] = 0;
}
void h_30(double *state, double *unused, double *out_490816161188815206) {
   out_490816161188815206[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1311087084188360515) {
   out_1311087084188360515[0] = 0;
   out_1311087084188360515[1] = 0;
   out_1311087084188360515[2] = 0;
   out_1311087084188360515[3] = 0;
   out_1311087084188360515[4] = 1;
   out_1311087084188360515[5] = 0;
   out_1311087084188360515[6] = 0;
   out_1311087084188360515[7] = 0;
   out_1311087084188360515[8] = 0;
}
void h_26(double *state, double *unused, double *out_1434326014090477641) {
   out_1434326014090477641[0] = state[7];
}
void H_26(double *state, double *unused, double *out_9099637497532239152) {
   out_9099637497532239152[0] = 0;
   out_9099637497532239152[1] = 0;
   out_9099637497532239152[2] = 0;
   out_9099637497532239152[3] = 0;
   out_9099637497532239152[4] = 0;
   out_9099637497532239152[5] = 0;
   out_9099637497532239152[6] = 0;
   out_9099637497532239152[7] = 1;
   out_9099637497532239152[8] = 0;
}
void h_27(double *state, double *unused, double *out_5950690791008222513) {
   out_5950690791008222513[0] = state[3];
}
void H_27(double *state, double *unused, double *out_863676227612064396) {
   out_863676227612064396[0] = 0;
   out_863676227612064396[1] = 0;
   out_863676227612064396[2] = 0;
   out_863676227612064396[3] = 1;
   out_863676227612064396[4] = 0;
   out_863676227612064396[5] = 0;
   out_863676227612064396[6] = 0;
   out_863676227612064396[7] = 0;
   out_863676227612064396[8] = 0;
}
void h_29(double *state, double *unused, double *out_2545969890255517479) {
   out_2545969890255517479[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1821318428502752699) {
   out_1821318428502752699[0] = 0;
   out_1821318428502752699[1] = 1;
   out_1821318428502752699[2] = 0;
   out_1821318428502752699[3] = 0;
   out_1821318428502752699[4] = 0;
   out_1821318428502752699[5] = 0;
   out_1821318428502752699[6] = 0;
   out_1821318428502752699[7] = 0;
   out_1821318428502752699[8] = 0;
}
void h_28(double *state, double *unused, double *out_8098101201419700091) {
   out_8098101201419700091[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3261080588566777875) {
   out_3261080588566777875[0] = 1;
   out_3261080588566777875[1] = 0;
   out_3261080588566777875[2] = 0;
   out_3261080588566777875[3] = 0;
   out_3261080588566777875[4] = 0;
   out_3261080588566777875[5] = 0;
   out_3261080588566777875[6] = 0;
   out_3261080588566777875[7] = 0;
   out_3261080588566777875[8] = 0;
}
void h_31(double *state, double *unused, double *out_1106204191698506013) {
   out_1106204191698506013[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5574957295426295812) {
   out_5574957295426295812[0] = 0;
   out_5574957295426295812[1] = 0;
   out_5574957295426295812[2] = 0;
   out_5574957295426295812[3] = 0;
   out_5574957295426295812[4] = 0;
   out_5574957295426295812[5] = 0;
   out_5574957295426295812[6] = 0;
   out_5574957295426295812[7] = 0;
   out_5574957295426295812[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_1285813334012964983) {
  err_fun(nom_x, delta_x, out_1285813334012964983);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4976523369423083376) {
  inv_err_fun(nom_x, true_x, out_4976523369423083376);
}
void car_H_mod_fun(double *state, double *out_1708338526219741257) {
  H_mod_fun(state, out_1708338526219741257);
}
void car_f_fun(double *state, double dt, double *out_48359745169696250) {
  f_fun(state,  dt, out_48359745169696250);
}
void car_F_fun(double *state, double dt, double *out_6580537581630503009) {
  F_fun(state,  dt, out_6580537581630503009);
}
void car_h_25(double *state, double *unused, double *out_6097011765287479957) {
  h_25(state, unused, out_6097011765287479957);
}
void car_H_25(double *state, double *unused, double *out_5605603257303256240) {
  H_25(state, unused, out_5605603257303256240);
}
void car_h_24(double *state, double *unused, double *out_3578546623177939744) {
  h_24(state, unused, out_3578546623177939744);
}
void car_H_24(double *state, double *unused, double *out_3379895473324387678) {
  H_24(state, unused, out_3379895473324387678);
}
void car_h_30(double *state, double *unused, double *out_490816161188815206) {
  h_30(state, unused, out_490816161188815206);
}
void car_H_30(double *state, double *unused, double *out_1311087084188360515) {
  H_30(state, unused, out_1311087084188360515);
}
void car_h_26(double *state, double *unused, double *out_1434326014090477641) {
  h_26(state, unused, out_1434326014090477641);
}
void car_H_26(double *state, double *unused, double *out_9099637497532239152) {
  H_26(state, unused, out_9099637497532239152);
}
void car_h_27(double *state, double *unused, double *out_5950690791008222513) {
  h_27(state, unused, out_5950690791008222513);
}
void car_H_27(double *state, double *unused, double *out_863676227612064396) {
  H_27(state, unused, out_863676227612064396);
}
void car_h_29(double *state, double *unused, double *out_2545969890255517479) {
  h_29(state, unused, out_2545969890255517479);
}
void car_H_29(double *state, double *unused, double *out_1821318428502752699) {
  H_29(state, unused, out_1821318428502752699);
}
void car_h_28(double *state, double *unused, double *out_8098101201419700091) {
  h_28(state, unused, out_8098101201419700091);
}
void car_H_28(double *state, double *unused, double *out_3261080588566777875) {
  H_28(state, unused, out_3261080588566777875);
}
void car_h_31(double *state, double *unused, double *out_1106204191698506013) {
  h_31(state, unused, out_1106204191698506013);
}
void car_H_31(double *state, double *unused, double *out_5574957295426295812) {
  H_31(state, unused, out_5574957295426295812);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
