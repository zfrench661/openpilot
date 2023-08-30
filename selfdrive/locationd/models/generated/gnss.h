#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2607042044397922302);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6847906624395849551);
void gnss_H_mod_fun(double *state, double *out_1241072667078802097);
void gnss_f_fun(double *state, double dt, double *out_7426352004039221893);
void gnss_F_fun(double *state, double dt, double *out_4166084344064825099);
void gnss_h_6(double *state, double *sat_pos, double *out_3051204460865106966);
void gnss_H_6(double *state, double *sat_pos, double *out_2469251146768242897);
void gnss_h_20(double *state, double *sat_pos, double *out_9035002349482779463);
void gnss_H_20(double *state, double *sat_pos, double *out_4647669428315909809);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_9066111164245375750);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4198580798365358951);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_9066111164245375750);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4198580798365358951);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}