#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6064873246655451156);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3037113823828593355);
void gnss_H_mod_fun(double *state, double *out_931065095912888979);
void gnss_f_fun(double *state, double dt, double *out_8245044661289878852);
void gnss_F_fun(double *state, double dt, double *out_884770970109736067);
void gnss_h_6(double *state, double *sat_pos, double *out_7129903965475405534);
void gnss_H_6(double *state, double *sat_pos, double *out_6076338186553436386);
void gnss_h_20(double *state, double *sat_pos, double *out_3680457990501382862);
void gnss_H_20(double *state, double *sat_pos, double *out_5241420826605998171);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3927344199620323249);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6401279497347571329);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3927344199620323249);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6401279497347571329);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}