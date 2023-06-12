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
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6064873246655451156) {
   out_6064873246655451156[0] = delta_x[0] + nom_x[0];
   out_6064873246655451156[1] = delta_x[1] + nom_x[1];
   out_6064873246655451156[2] = delta_x[2] + nom_x[2];
   out_6064873246655451156[3] = delta_x[3] + nom_x[3];
   out_6064873246655451156[4] = delta_x[4] + nom_x[4];
   out_6064873246655451156[5] = delta_x[5] + nom_x[5];
   out_6064873246655451156[6] = delta_x[6] + nom_x[6];
   out_6064873246655451156[7] = delta_x[7] + nom_x[7];
   out_6064873246655451156[8] = delta_x[8] + nom_x[8];
   out_6064873246655451156[9] = delta_x[9] + nom_x[9];
   out_6064873246655451156[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3037113823828593355) {
   out_3037113823828593355[0] = -nom_x[0] + true_x[0];
   out_3037113823828593355[1] = -nom_x[1] + true_x[1];
   out_3037113823828593355[2] = -nom_x[2] + true_x[2];
   out_3037113823828593355[3] = -nom_x[3] + true_x[3];
   out_3037113823828593355[4] = -nom_x[4] + true_x[4];
   out_3037113823828593355[5] = -nom_x[5] + true_x[5];
   out_3037113823828593355[6] = -nom_x[6] + true_x[6];
   out_3037113823828593355[7] = -nom_x[7] + true_x[7];
   out_3037113823828593355[8] = -nom_x[8] + true_x[8];
   out_3037113823828593355[9] = -nom_x[9] + true_x[9];
   out_3037113823828593355[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_931065095912888979) {
   out_931065095912888979[0] = 1.0;
   out_931065095912888979[1] = 0;
   out_931065095912888979[2] = 0;
   out_931065095912888979[3] = 0;
   out_931065095912888979[4] = 0;
   out_931065095912888979[5] = 0;
   out_931065095912888979[6] = 0;
   out_931065095912888979[7] = 0;
   out_931065095912888979[8] = 0;
   out_931065095912888979[9] = 0;
   out_931065095912888979[10] = 0;
   out_931065095912888979[11] = 0;
   out_931065095912888979[12] = 1.0;
   out_931065095912888979[13] = 0;
   out_931065095912888979[14] = 0;
   out_931065095912888979[15] = 0;
   out_931065095912888979[16] = 0;
   out_931065095912888979[17] = 0;
   out_931065095912888979[18] = 0;
   out_931065095912888979[19] = 0;
   out_931065095912888979[20] = 0;
   out_931065095912888979[21] = 0;
   out_931065095912888979[22] = 0;
   out_931065095912888979[23] = 0;
   out_931065095912888979[24] = 1.0;
   out_931065095912888979[25] = 0;
   out_931065095912888979[26] = 0;
   out_931065095912888979[27] = 0;
   out_931065095912888979[28] = 0;
   out_931065095912888979[29] = 0;
   out_931065095912888979[30] = 0;
   out_931065095912888979[31] = 0;
   out_931065095912888979[32] = 0;
   out_931065095912888979[33] = 0;
   out_931065095912888979[34] = 0;
   out_931065095912888979[35] = 0;
   out_931065095912888979[36] = 1.0;
   out_931065095912888979[37] = 0;
   out_931065095912888979[38] = 0;
   out_931065095912888979[39] = 0;
   out_931065095912888979[40] = 0;
   out_931065095912888979[41] = 0;
   out_931065095912888979[42] = 0;
   out_931065095912888979[43] = 0;
   out_931065095912888979[44] = 0;
   out_931065095912888979[45] = 0;
   out_931065095912888979[46] = 0;
   out_931065095912888979[47] = 0;
   out_931065095912888979[48] = 1.0;
   out_931065095912888979[49] = 0;
   out_931065095912888979[50] = 0;
   out_931065095912888979[51] = 0;
   out_931065095912888979[52] = 0;
   out_931065095912888979[53] = 0;
   out_931065095912888979[54] = 0;
   out_931065095912888979[55] = 0;
   out_931065095912888979[56] = 0;
   out_931065095912888979[57] = 0;
   out_931065095912888979[58] = 0;
   out_931065095912888979[59] = 0;
   out_931065095912888979[60] = 1.0;
   out_931065095912888979[61] = 0;
   out_931065095912888979[62] = 0;
   out_931065095912888979[63] = 0;
   out_931065095912888979[64] = 0;
   out_931065095912888979[65] = 0;
   out_931065095912888979[66] = 0;
   out_931065095912888979[67] = 0;
   out_931065095912888979[68] = 0;
   out_931065095912888979[69] = 0;
   out_931065095912888979[70] = 0;
   out_931065095912888979[71] = 0;
   out_931065095912888979[72] = 1.0;
   out_931065095912888979[73] = 0;
   out_931065095912888979[74] = 0;
   out_931065095912888979[75] = 0;
   out_931065095912888979[76] = 0;
   out_931065095912888979[77] = 0;
   out_931065095912888979[78] = 0;
   out_931065095912888979[79] = 0;
   out_931065095912888979[80] = 0;
   out_931065095912888979[81] = 0;
   out_931065095912888979[82] = 0;
   out_931065095912888979[83] = 0;
   out_931065095912888979[84] = 1.0;
   out_931065095912888979[85] = 0;
   out_931065095912888979[86] = 0;
   out_931065095912888979[87] = 0;
   out_931065095912888979[88] = 0;
   out_931065095912888979[89] = 0;
   out_931065095912888979[90] = 0;
   out_931065095912888979[91] = 0;
   out_931065095912888979[92] = 0;
   out_931065095912888979[93] = 0;
   out_931065095912888979[94] = 0;
   out_931065095912888979[95] = 0;
   out_931065095912888979[96] = 1.0;
   out_931065095912888979[97] = 0;
   out_931065095912888979[98] = 0;
   out_931065095912888979[99] = 0;
   out_931065095912888979[100] = 0;
   out_931065095912888979[101] = 0;
   out_931065095912888979[102] = 0;
   out_931065095912888979[103] = 0;
   out_931065095912888979[104] = 0;
   out_931065095912888979[105] = 0;
   out_931065095912888979[106] = 0;
   out_931065095912888979[107] = 0;
   out_931065095912888979[108] = 1.0;
   out_931065095912888979[109] = 0;
   out_931065095912888979[110] = 0;
   out_931065095912888979[111] = 0;
   out_931065095912888979[112] = 0;
   out_931065095912888979[113] = 0;
   out_931065095912888979[114] = 0;
   out_931065095912888979[115] = 0;
   out_931065095912888979[116] = 0;
   out_931065095912888979[117] = 0;
   out_931065095912888979[118] = 0;
   out_931065095912888979[119] = 0;
   out_931065095912888979[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_8245044661289878852) {
   out_8245044661289878852[0] = dt*state[3] + state[0];
   out_8245044661289878852[1] = dt*state[4] + state[1];
   out_8245044661289878852[2] = dt*state[5] + state[2];
   out_8245044661289878852[3] = state[3];
   out_8245044661289878852[4] = state[4];
   out_8245044661289878852[5] = state[5];
   out_8245044661289878852[6] = dt*state[7] + state[6];
   out_8245044661289878852[7] = dt*state[8] + state[7];
   out_8245044661289878852[8] = state[8];
   out_8245044661289878852[9] = state[9];
   out_8245044661289878852[10] = state[10];
}
void F_fun(double *state, double dt, double *out_884770970109736067) {
   out_884770970109736067[0] = 1;
   out_884770970109736067[1] = 0;
   out_884770970109736067[2] = 0;
   out_884770970109736067[3] = dt;
   out_884770970109736067[4] = 0;
   out_884770970109736067[5] = 0;
   out_884770970109736067[6] = 0;
   out_884770970109736067[7] = 0;
   out_884770970109736067[8] = 0;
   out_884770970109736067[9] = 0;
   out_884770970109736067[10] = 0;
   out_884770970109736067[11] = 0;
   out_884770970109736067[12] = 1;
   out_884770970109736067[13] = 0;
   out_884770970109736067[14] = 0;
   out_884770970109736067[15] = dt;
   out_884770970109736067[16] = 0;
   out_884770970109736067[17] = 0;
   out_884770970109736067[18] = 0;
   out_884770970109736067[19] = 0;
   out_884770970109736067[20] = 0;
   out_884770970109736067[21] = 0;
   out_884770970109736067[22] = 0;
   out_884770970109736067[23] = 0;
   out_884770970109736067[24] = 1;
   out_884770970109736067[25] = 0;
   out_884770970109736067[26] = 0;
   out_884770970109736067[27] = dt;
   out_884770970109736067[28] = 0;
   out_884770970109736067[29] = 0;
   out_884770970109736067[30] = 0;
   out_884770970109736067[31] = 0;
   out_884770970109736067[32] = 0;
   out_884770970109736067[33] = 0;
   out_884770970109736067[34] = 0;
   out_884770970109736067[35] = 0;
   out_884770970109736067[36] = 1;
   out_884770970109736067[37] = 0;
   out_884770970109736067[38] = 0;
   out_884770970109736067[39] = 0;
   out_884770970109736067[40] = 0;
   out_884770970109736067[41] = 0;
   out_884770970109736067[42] = 0;
   out_884770970109736067[43] = 0;
   out_884770970109736067[44] = 0;
   out_884770970109736067[45] = 0;
   out_884770970109736067[46] = 0;
   out_884770970109736067[47] = 0;
   out_884770970109736067[48] = 1;
   out_884770970109736067[49] = 0;
   out_884770970109736067[50] = 0;
   out_884770970109736067[51] = 0;
   out_884770970109736067[52] = 0;
   out_884770970109736067[53] = 0;
   out_884770970109736067[54] = 0;
   out_884770970109736067[55] = 0;
   out_884770970109736067[56] = 0;
   out_884770970109736067[57] = 0;
   out_884770970109736067[58] = 0;
   out_884770970109736067[59] = 0;
   out_884770970109736067[60] = 1;
   out_884770970109736067[61] = 0;
   out_884770970109736067[62] = 0;
   out_884770970109736067[63] = 0;
   out_884770970109736067[64] = 0;
   out_884770970109736067[65] = 0;
   out_884770970109736067[66] = 0;
   out_884770970109736067[67] = 0;
   out_884770970109736067[68] = 0;
   out_884770970109736067[69] = 0;
   out_884770970109736067[70] = 0;
   out_884770970109736067[71] = 0;
   out_884770970109736067[72] = 1;
   out_884770970109736067[73] = dt;
   out_884770970109736067[74] = 0;
   out_884770970109736067[75] = 0;
   out_884770970109736067[76] = 0;
   out_884770970109736067[77] = 0;
   out_884770970109736067[78] = 0;
   out_884770970109736067[79] = 0;
   out_884770970109736067[80] = 0;
   out_884770970109736067[81] = 0;
   out_884770970109736067[82] = 0;
   out_884770970109736067[83] = 0;
   out_884770970109736067[84] = 1;
   out_884770970109736067[85] = dt;
   out_884770970109736067[86] = 0;
   out_884770970109736067[87] = 0;
   out_884770970109736067[88] = 0;
   out_884770970109736067[89] = 0;
   out_884770970109736067[90] = 0;
   out_884770970109736067[91] = 0;
   out_884770970109736067[92] = 0;
   out_884770970109736067[93] = 0;
   out_884770970109736067[94] = 0;
   out_884770970109736067[95] = 0;
   out_884770970109736067[96] = 1;
   out_884770970109736067[97] = 0;
   out_884770970109736067[98] = 0;
   out_884770970109736067[99] = 0;
   out_884770970109736067[100] = 0;
   out_884770970109736067[101] = 0;
   out_884770970109736067[102] = 0;
   out_884770970109736067[103] = 0;
   out_884770970109736067[104] = 0;
   out_884770970109736067[105] = 0;
   out_884770970109736067[106] = 0;
   out_884770970109736067[107] = 0;
   out_884770970109736067[108] = 1;
   out_884770970109736067[109] = 0;
   out_884770970109736067[110] = 0;
   out_884770970109736067[111] = 0;
   out_884770970109736067[112] = 0;
   out_884770970109736067[113] = 0;
   out_884770970109736067[114] = 0;
   out_884770970109736067[115] = 0;
   out_884770970109736067[116] = 0;
   out_884770970109736067[117] = 0;
   out_884770970109736067[118] = 0;
   out_884770970109736067[119] = 0;
   out_884770970109736067[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_7129903965475405534) {
   out_7129903965475405534[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_6076338186553436386) {
   out_6076338186553436386[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6076338186553436386[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6076338186553436386[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6076338186553436386[3] = 0;
   out_6076338186553436386[4] = 0;
   out_6076338186553436386[5] = 0;
   out_6076338186553436386[6] = 1;
   out_6076338186553436386[7] = 0;
   out_6076338186553436386[8] = 0;
   out_6076338186553436386[9] = 0;
   out_6076338186553436386[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_3680457990501382862) {
   out_3680457990501382862[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_5241420826605998171) {
   out_5241420826605998171[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5241420826605998171[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5241420826605998171[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5241420826605998171[3] = 0;
   out_5241420826605998171[4] = 0;
   out_5241420826605998171[5] = 0;
   out_5241420826605998171[6] = 1;
   out_5241420826605998171[7] = 0;
   out_5241420826605998171[8] = 0;
   out_5241420826605998171[9] = 1;
   out_5241420826605998171[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_3927344199620323249) {
   out_3927344199620323249[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_6401279497347571329) {
   out_6401279497347571329[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6401279497347571329[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6401279497347571329[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6401279497347571329[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6401279497347571329[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6401279497347571329[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6401279497347571329[6] = 0;
   out_6401279497347571329[7] = 1;
   out_6401279497347571329[8] = 0;
   out_6401279497347571329[9] = 0;
   out_6401279497347571329[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_3927344199620323249) {
   out_3927344199620323249[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_6401279497347571329) {
   out_6401279497347571329[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6401279497347571329[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6401279497347571329[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6401279497347571329[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6401279497347571329[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6401279497347571329[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6401279497347571329[6] = 0;
   out_6401279497347571329[7] = 1;
   out_6401279497347571329[8] = 0;
   out_6401279497347571329[9] = 0;
   out_6401279497347571329[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6064873246655451156) {
  err_fun(nom_x, delta_x, out_6064873246655451156);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3037113823828593355) {
  inv_err_fun(nom_x, true_x, out_3037113823828593355);
}
void gnss_H_mod_fun(double *state, double *out_931065095912888979) {
  H_mod_fun(state, out_931065095912888979);
}
void gnss_f_fun(double *state, double dt, double *out_8245044661289878852) {
  f_fun(state,  dt, out_8245044661289878852);
}
void gnss_F_fun(double *state, double dt, double *out_884770970109736067) {
  F_fun(state,  dt, out_884770970109736067);
}
void gnss_h_6(double *state, double *sat_pos, double *out_7129903965475405534) {
  h_6(state, sat_pos, out_7129903965475405534);
}
void gnss_H_6(double *state, double *sat_pos, double *out_6076338186553436386) {
  H_6(state, sat_pos, out_6076338186553436386);
}
void gnss_h_20(double *state, double *sat_pos, double *out_3680457990501382862) {
  h_20(state, sat_pos, out_3680457990501382862);
}
void gnss_H_20(double *state, double *sat_pos, double *out_5241420826605998171) {
  H_20(state, sat_pos, out_5241420826605998171);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3927344199620323249) {
  h_7(state, sat_pos_vel, out_3927344199620323249);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6401279497347571329) {
  H_7(state, sat_pos_vel, out_6401279497347571329);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3927344199620323249) {
  h_21(state, sat_pos_vel, out_3927344199620323249);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6401279497347571329) {
  H_21(state, sat_pos_vel, out_6401279497347571329);
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
