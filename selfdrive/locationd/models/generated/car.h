#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_1285813334012964983);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4976523369423083376);
void car_H_mod_fun(double *state, double *out_1708338526219741257);
void car_f_fun(double *state, double dt, double *out_48359745169696250);
void car_F_fun(double *state, double dt, double *out_6580537581630503009);
void car_h_25(double *state, double *unused, double *out_6097011765287479957);
void car_H_25(double *state, double *unused, double *out_5605603257303256240);
void car_h_24(double *state, double *unused, double *out_3578546623177939744);
void car_H_24(double *state, double *unused, double *out_3379895473324387678);
void car_h_30(double *state, double *unused, double *out_490816161188815206);
void car_H_30(double *state, double *unused, double *out_1311087084188360515);
void car_h_26(double *state, double *unused, double *out_1434326014090477641);
void car_H_26(double *state, double *unused, double *out_9099637497532239152);
void car_h_27(double *state, double *unused, double *out_5950690791008222513);
void car_H_27(double *state, double *unused, double *out_863676227612064396);
void car_h_29(double *state, double *unused, double *out_2545969890255517479);
void car_H_29(double *state, double *unused, double *out_1821318428502752699);
void car_h_28(double *state, double *unused, double *out_8098101201419700091);
void car_H_28(double *state, double *unused, double *out_3261080588566777875);
void car_h_31(double *state, double *unused, double *out_1106204191698506013);
void car_H_31(double *state, double *unused, double *out_5574957295426295812);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}