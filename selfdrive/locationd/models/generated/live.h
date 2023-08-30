#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_6616903330243791477);
void live_err_fun(double *nom_x, double *delta_x, double *out_7776782165476422622);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1063905258470432307);
void live_H_mod_fun(double *state, double *out_1018911972123403788);
void live_f_fun(double *state, double dt, double *out_946918272088411036);
void live_F_fun(double *state, double dt, double *out_7622692614917921459);
void live_h_4(double *state, double *unused, double *out_4921921813527815973);
void live_H_4(double *state, double *unused, double *out_5733132674020333042);
void live_h_9(double *state, double *unused, double *out_5323875262485216062);
void live_H_9(double *state, double *unused, double *out_8556443663334441091);
void live_h_10(double *state, double *unused, double *out_3114740063238958036);
void live_H_10(double *state, double *unused, double *out_8539081862252184483);
void live_h_12(double *state, double *unused, double *out_6123549659445635911);
void live_H_12(double *state, double *unused, double *out_5112033648972739375);
void live_h_35(double *state, double *unused, double *out_1769556361636079911);
void live_H_35(double *state, double *unused, double *out_2366470616647725666);
void live_h_32(double *state, double *unused, double *out_560078100617833566);
void live_H_32(double *state, double *unused, double *out_1181373723339433063);
void live_h_13(double *state, double *unused, double *out_7746327227580583969);
void live_H_13(double *state, double *unused, double *out_8484307507704266798);
void live_h_14(double *state, double *unused, double *out_5323875262485216062);
void live_H_14(double *state, double *unused, double *out_8556443663334441091);
void live_h_33(double *state, double *unused, double *out_5155087305118834571);
void live_H_33(double *state, double *unused, double *out_784086387991131938);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}