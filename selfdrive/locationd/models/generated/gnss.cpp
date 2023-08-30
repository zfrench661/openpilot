#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2607042044397922302) {
   out_2607042044397922302[0] = delta_x[0] + nom_x[0];
   out_2607042044397922302[1] = delta_x[1] + nom_x[1];
   out_2607042044397922302[2] = delta_x[2] + nom_x[2];
   out_2607042044397922302[3] = delta_x[3] + nom_x[3];
   out_2607042044397922302[4] = delta_x[4] + nom_x[4];
   out_2607042044397922302[5] = delta_x[5] + nom_x[5];
   out_2607042044397922302[6] = delta_x[6] + nom_x[6];
   out_2607042044397922302[7] = delta_x[7] + nom_x[7];
   out_2607042044397922302[8] = delta_x[8] + nom_x[8];
   out_2607042044397922302[9] = delta_x[9] + nom_x[9];
   out_2607042044397922302[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6847906624395849551) {
   out_6847906624395849551[0] = -nom_x[0] + true_x[0];
   out_6847906624395849551[1] = -nom_x[1] + true_x[1];
   out_6847906624395849551[2] = -nom_x[2] + true_x[2];
   out_6847906624395849551[3] = -nom_x[3] + true_x[3];
   out_6847906624395849551[4] = -nom_x[4] + true_x[4];
   out_6847906624395849551[5] = -nom_x[5] + true_x[5];
   out_6847906624395849551[6] = -nom_x[6] + true_x[6];
   out_6847906624395849551[7] = -nom_x[7] + true_x[7];
   out_6847906624395849551[8] = -nom_x[8] + true_x[8];
   out_6847906624395849551[9] = -nom_x[9] + true_x[9];
   out_6847906624395849551[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_1241072667078802097) {
   out_1241072667078802097[0] = 1.0;
   out_1241072667078802097[1] = 0;
   out_1241072667078802097[2] = 0;
   out_1241072667078802097[3] = 0;
   out_1241072667078802097[4] = 0;
   out_1241072667078802097[5] = 0;
   out_1241072667078802097[6] = 0;
   out_1241072667078802097[7] = 0;
   out_1241072667078802097[8] = 0;
   out_1241072667078802097[9] = 0;
   out_1241072667078802097[10] = 0;
   out_1241072667078802097[11] = 0;
   out_1241072667078802097[12] = 1.0;
   out_1241072667078802097[13] = 0;
   out_1241072667078802097[14] = 0;
   out_1241072667078802097[15] = 0;
   out_1241072667078802097[16] = 0;
   out_1241072667078802097[17] = 0;
   out_1241072667078802097[18] = 0;
   out_1241072667078802097[19] = 0;
   out_1241072667078802097[20] = 0;
   out_1241072667078802097[21] = 0;
   out_1241072667078802097[22] = 0;
   out_1241072667078802097[23] = 0;
   out_1241072667078802097[24] = 1.0;
   out_1241072667078802097[25] = 0;
   out_1241072667078802097[26] = 0;
   out_1241072667078802097[27] = 0;
   out_1241072667078802097[28] = 0;
   out_1241072667078802097[29] = 0;
   out_1241072667078802097[30] = 0;
   out_1241072667078802097[31] = 0;
   out_1241072667078802097[32] = 0;
   out_1241072667078802097[33] = 0;
   out_1241072667078802097[34] = 0;
   out_1241072667078802097[35] = 0;
   out_1241072667078802097[36] = 1.0;
   out_1241072667078802097[37] = 0;
   out_1241072667078802097[38] = 0;
   out_1241072667078802097[39] = 0;
   out_1241072667078802097[40] = 0;
   out_1241072667078802097[41] = 0;
   out_1241072667078802097[42] = 0;
   out_1241072667078802097[43] = 0;
   out_1241072667078802097[44] = 0;
   out_1241072667078802097[45] = 0;
   out_1241072667078802097[46] = 0;
   out_1241072667078802097[47] = 0;
   out_1241072667078802097[48] = 1.0;
   out_1241072667078802097[49] = 0;
   out_1241072667078802097[50] = 0;
   out_1241072667078802097[51] = 0;
   out_1241072667078802097[52] = 0;
   out_1241072667078802097[53] = 0;
   out_1241072667078802097[54] = 0;
   out_1241072667078802097[55] = 0;
   out_1241072667078802097[56] = 0;
   out_1241072667078802097[57] = 0;
   out_1241072667078802097[58] = 0;
   out_1241072667078802097[59] = 0;
   out_1241072667078802097[60] = 1.0;
   out_1241072667078802097[61] = 0;
   out_1241072667078802097[62] = 0;
   out_1241072667078802097[63] = 0;
   out_1241072667078802097[64] = 0;
   out_1241072667078802097[65] = 0;
   out_1241072667078802097[66] = 0;
   out_1241072667078802097[67] = 0;
   out_1241072667078802097[68] = 0;
   out_1241072667078802097[69] = 0;
   out_1241072667078802097[70] = 0;
   out_1241072667078802097[71] = 0;
   out_1241072667078802097[72] = 1.0;
   out_1241072667078802097[73] = 0;
   out_1241072667078802097[74] = 0;
   out_1241072667078802097[75] = 0;
   out_1241072667078802097[76] = 0;
   out_1241072667078802097[77] = 0;
   out_1241072667078802097[78] = 0;
   out_1241072667078802097[79] = 0;
   out_1241072667078802097[80] = 0;
   out_1241072667078802097[81] = 0;
   out_1241072667078802097[82] = 0;
   out_1241072667078802097[83] = 0;
   out_1241072667078802097[84] = 1.0;
   out_1241072667078802097[85] = 0;
   out_1241072667078802097[86] = 0;
   out_1241072667078802097[87] = 0;
   out_1241072667078802097[88] = 0;
   out_1241072667078802097[89] = 0;
   out_1241072667078802097[90] = 0;
   out_1241072667078802097[91] = 0;
   out_1241072667078802097[92] = 0;
   out_1241072667078802097[93] = 0;
   out_1241072667078802097[94] = 0;
   out_1241072667078802097[95] = 0;
   out_1241072667078802097[96] = 1.0;
   out_1241072667078802097[97] = 0;
   out_1241072667078802097[98] = 0;
   out_1241072667078802097[99] = 0;
   out_1241072667078802097[100] = 0;
   out_1241072667078802097[101] = 0;
   out_1241072667078802097[102] = 0;
   out_1241072667078802097[103] = 0;
   out_1241072667078802097[104] = 0;
   out_1241072667078802097[105] = 0;
   out_1241072667078802097[106] = 0;
   out_1241072667078802097[107] = 0;
   out_1241072667078802097[108] = 1.0;
   out_1241072667078802097[109] = 0;
   out_1241072667078802097[110] = 0;
   out_1241072667078802097[111] = 0;
   out_1241072667078802097[112] = 0;
   out_1241072667078802097[113] = 0;
   out_1241072667078802097[114] = 0;
   out_1241072667078802097[115] = 0;
   out_1241072667078802097[116] = 0;
   out_1241072667078802097[117] = 0;
   out_1241072667078802097[118] = 0;
   out_1241072667078802097[119] = 0;
   out_1241072667078802097[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_7426352004039221893) {
   out_7426352004039221893[0] = dt*state[3] + state[0];
   out_7426352004039221893[1] = dt*state[4] + state[1];
   out_7426352004039221893[2] = dt*state[5] + state[2];
   out_7426352004039221893[3] = state[3];
   out_7426352004039221893[4] = state[4];
   out_7426352004039221893[5] = state[5];
   out_7426352004039221893[6] = dt*state[7] + state[6];
   out_7426352004039221893[7] = dt*state[8] + state[7];
   out_7426352004039221893[8] = state[8];
   out_7426352004039221893[9] = state[9];
   out_7426352004039221893[10] = state[10];
}
void F_fun(double *state, double dt, double *out_4166084344064825099) {
   out_4166084344064825099[0] = 1;
   out_4166084344064825099[1] = 0;
   out_4166084344064825099[2] = 0;
   out_4166084344064825099[3] = dt;
   out_4166084344064825099[4] = 0;
   out_4166084344064825099[5] = 0;
   out_4166084344064825099[6] = 0;
   out_4166084344064825099[7] = 0;
   out_4166084344064825099[8] = 0;
   out_4166084344064825099[9] = 0;
   out_4166084344064825099[10] = 0;
   out_4166084344064825099[11] = 0;
   out_4166084344064825099[12] = 1;
   out_4166084344064825099[13] = 0;
   out_4166084344064825099[14] = 0;
   out_4166084344064825099[15] = dt;
   out_4166084344064825099[16] = 0;
   out_4166084344064825099[17] = 0;
   out_4166084344064825099[18] = 0;
   out_4166084344064825099[19] = 0;
   out_4166084344064825099[20] = 0;
   out_4166084344064825099[21] = 0;
   out_4166084344064825099[22] = 0;
   out_4166084344064825099[23] = 0;
   out_4166084344064825099[24] = 1;
   out_4166084344064825099[25] = 0;
   out_4166084344064825099[26] = 0;
   out_4166084344064825099[27] = dt;
   out_4166084344064825099[28] = 0;
   out_4166084344064825099[29] = 0;
   out_4166084344064825099[30] = 0;
   out_4166084344064825099[31] = 0;
   out_4166084344064825099[32] = 0;
   out_4166084344064825099[33] = 0;
   out_4166084344064825099[34] = 0;
   out_4166084344064825099[35] = 0;
   out_4166084344064825099[36] = 1;
   out_4166084344064825099[37] = 0;
   out_4166084344064825099[38] = 0;
   out_4166084344064825099[39] = 0;
   out_4166084344064825099[40] = 0;
   out_4166084344064825099[41] = 0;
   out_4166084344064825099[42] = 0;
   out_4166084344064825099[43] = 0;
   out_4166084344064825099[44] = 0;
   out_4166084344064825099[45] = 0;
   out_4166084344064825099[46] = 0;
   out_4166084344064825099[47] = 0;
   out_4166084344064825099[48] = 1;
   out_4166084344064825099[49] = 0;
   out_4166084344064825099[50] = 0;
   out_4166084344064825099[51] = 0;
   out_4166084344064825099[52] = 0;
   out_4166084344064825099[53] = 0;
   out_4166084344064825099[54] = 0;
   out_4166084344064825099[55] = 0;
   out_4166084344064825099[56] = 0;
   out_4166084344064825099[57] = 0;
   out_4166084344064825099[58] = 0;
   out_4166084344064825099[59] = 0;
   out_4166084344064825099[60] = 1;
   out_4166084344064825099[61] = 0;
   out_4166084344064825099[62] = 0;
   out_4166084344064825099[63] = 0;
   out_4166084344064825099[64] = 0;
   out_4166084344064825099[65] = 0;
   out_4166084344064825099[66] = 0;
   out_4166084344064825099[67] = 0;
   out_4166084344064825099[68] = 0;
   out_4166084344064825099[69] = 0;
   out_4166084344064825099[70] = 0;
   out_4166084344064825099[71] = 0;
   out_4166084344064825099[72] = 1;
   out_4166084344064825099[73] = dt;
   out_4166084344064825099[74] = 0;
   out_4166084344064825099[75] = 0;
   out_4166084344064825099[76] = 0;
   out_4166084344064825099[77] = 0;
   out_4166084344064825099[78] = 0;
   out_4166084344064825099[79] = 0;
   out_4166084344064825099[80] = 0;
   out_4166084344064825099[81] = 0;
   out_4166084344064825099[82] = 0;
   out_4166084344064825099[83] = 0;
   out_4166084344064825099[84] = 1;
   out_4166084344064825099[85] = dt;
   out_4166084344064825099[86] = 0;
   out_4166084344064825099[87] = 0;
   out_4166084344064825099[88] = 0;
   out_4166084344064825099[89] = 0;
   out_4166084344064825099[90] = 0;
   out_4166084344064825099[91] = 0;
   out_4166084344064825099[92] = 0;
   out_4166084344064825099[93] = 0;
   out_4166084344064825099[94] = 0;
   out_4166084344064825099[95] = 0;
   out_4166084344064825099[96] = 1;
   out_4166084344064825099[97] = 0;
   out_4166084344064825099[98] = 0;
   out_4166084344064825099[99] = 0;
   out_4166084344064825099[100] = 0;
   out_4166084344064825099[101] = 0;
   out_4166084344064825099[102] = 0;
   out_4166084344064825099[103] = 0;
   out_4166084344064825099[104] = 0;
   out_4166084344064825099[105] = 0;
   out_4166084344064825099[106] = 0;
   out_4166084344064825099[107] = 0;
   out_4166084344064825099[108] = 1;
   out_4166084344064825099[109] = 0;
   out_4166084344064825099[110] = 0;
   out_4166084344064825099[111] = 0;
   out_4166084344064825099[112] = 0;
   out_4166084344064825099[113] = 0;
   out_4166084344064825099[114] = 0;
   out_4166084344064825099[115] = 0;
   out_4166084344064825099[116] = 0;
   out_4166084344064825099[117] = 0;
   out_4166084344064825099[118] = 0;
   out_4166084344064825099[119] = 0;
   out_4166084344064825099[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_3051204460865106966) {
   out_3051204460865106966[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_2469251146768242897) {
   out_2469251146768242897[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2469251146768242897[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2469251146768242897[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2469251146768242897[3] = 0;
   out_2469251146768242897[4] = 0;
   out_2469251146768242897[5] = 0;
   out_2469251146768242897[6] = 1;
   out_2469251146768242897[7] = 0;
   out_2469251146768242897[8] = 0;
   out_2469251146768242897[9] = 0;
   out_2469251146768242897[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_9035002349482779463) {
   out_9035002349482779463[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_4647669428315909809) {
   out_4647669428315909809[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4647669428315909809[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4647669428315909809[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4647669428315909809[3] = 0;
   out_4647669428315909809[4] = 0;
   out_4647669428315909809[5] = 0;
   out_4647669428315909809[6] = 1;
   out_4647669428315909809[7] = 0;
   out_4647669428315909809[8] = 0;
   out_4647669428315909809[9] = 1;
   out_4647669428315909809[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_9066111164245375750) {
   out_9066111164245375750[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_4198580798365358951) {
   out_4198580798365358951[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4198580798365358951[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4198580798365358951[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4198580798365358951[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4198580798365358951[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4198580798365358951[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4198580798365358951[6] = 0;
   out_4198580798365358951[7] = 1;
   out_4198580798365358951[8] = 0;
   out_4198580798365358951[9] = 0;
   out_4198580798365358951[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_9066111164245375750) {
   out_9066111164245375750[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_4198580798365358951) {
   out_4198580798365358951[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4198580798365358951[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4198580798365358951[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4198580798365358951[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4198580798365358951[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4198580798365358951[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4198580798365358951[6] = 0;
   out_4198580798365358951[7] = 1;
   out_4198580798365358951[8] = 0;
   out_4198580798365358951[9] = 0;
   out_4198580798365358951[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2607042044397922302) {
  err_fun(nom_x, delta_x, out_2607042044397922302);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6847906624395849551) {
  inv_err_fun(nom_x, true_x, out_6847906624395849551);
}
void gnss_H_mod_fun(double *state, double *out_1241072667078802097) {
  H_mod_fun(state, out_1241072667078802097);
}
void gnss_f_fun(double *state, double dt, double *out_7426352004039221893) {
  f_fun(state,  dt, out_7426352004039221893);
}
void gnss_F_fun(double *state, double dt, double *out_4166084344064825099) {
  F_fun(state,  dt, out_4166084344064825099);
}
void gnss_h_6(double *state, double *sat_pos, double *out_3051204460865106966) {
  h_6(state, sat_pos, out_3051204460865106966);
}
void gnss_H_6(double *state, double *sat_pos, double *out_2469251146768242897) {
  H_6(state, sat_pos, out_2469251146768242897);
}
void gnss_h_20(double *state, double *sat_pos, double *out_9035002349482779463) {
  h_20(state, sat_pos, out_9035002349482779463);
}
void gnss_H_20(double *state, double *sat_pos, double *out_4647669428315909809) {
  H_20(state, sat_pos, out_4647669428315909809);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_9066111164245375750) {
  h_7(state, sat_pos_vel, out_9066111164245375750);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4198580798365358951) {
  H_7(state, sat_pos_vel, out_4198580798365358951);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_9066111164245375750) {
  h_21(state, sat_pos_vel, out_9066111164245375750);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4198580798365358951) {
  H_21(state, sat_pos_vel, out_4198580798365358951);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
