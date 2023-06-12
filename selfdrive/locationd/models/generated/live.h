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
void live_H(double *in_vec, double *out_4988174483772121019);
void live_err_fun(double *nom_x, double *delta_x, double *out_981480701005785015);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7691555028347121894);
void live_H_mod_fun(double *state, double *out_3170047989686754658);
void live_f_fun(double *state, double dt, double *out_8204218452068835951);
void live_F_fun(double *state, double dt, double *out_5970024142781703028);
void live_h_4(double *state, double *unused, double *out_5068762038946320161);
void live_H_4(double *state, double *unused, double *out_9200525426774583301);
void live_h_9(double *state, double *unused, double *out_7087070483927245922);
void live_H_9(double *state, double *unused, double *out_1913306491510135831);
void live_h_10(double *state, double *unused, double *out_4000295638352727946);
void live_H_10(double *state, double *unused, double *out_9069560577065108470);
void live_h_12(double *state, double *unused, double *out_4084887904843798783);
void live_H_12(double *state, double *unused, double *out_2864960269892235319);
void live_h_35(double *state, double *unused, double *out_7997807198617791916);
void live_H_35(double *state, double *unused, double *out_5833863369401975925);
void live_h_32(double *state, double *unused, double *out_4122265439760215713);
void live_H_32(double *state, double *unused, double *out_8720344792250430421);
void live_h_13(double *state, double *unused, double *out_5889941460073324800);
void live_H_13(double *state, double *unused, double *out_4874733410392409137);
void live_h_14(double *state, double *unused, double *out_7087070483927245922);
void live_H_14(double *state, double *unused, double *out_1913306491510135831);
void live_h_33(double *state, double *unused, double *out_2652568407664332798);
void live_H_33(double *state, double *unused, double *out_2683306364763118321);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}