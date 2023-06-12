#include "car.h"

namespace {
#define DIM 8
#define EDIM 8
#define MEDIM 8
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

double steer_ratio;

void set_steer_ratio(double x){ steer_ratio = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3307893292679224287) {
   out_3307893292679224287[0] = delta_x[0] + nom_x[0];
   out_3307893292679224287[1] = delta_x[1] + nom_x[1];
   out_3307893292679224287[2] = delta_x[2] + nom_x[2];
   out_3307893292679224287[3] = delta_x[3] + nom_x[3];
   out_3307893292679224287[4] = delta_x[4] + nom_x[4];
   out_3307893292679224287[5] = delta_x[5] + nom_x[5];
   out_3307893292679224287[6] = delta_x[6] + nom_x[6];
   out_3307893292679224287[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6001357432987055053) {
   out_6001357432987055053[0] = -nom_x[0] + true_x[0];
   out_6001357432987055053[1] = -nom_x[1] + true_x[1];
   out_6001357432987055053[2] = -nom_x[2] + true_x[2];
   out_6001357432987055053[3] = -nom_x[3] + true_x[3];
   out_6001357432987055053[4] = -nom_x[4] + true_x[4];
   out_6001357432987055053[5] = -nom_x[5] + true_x[5];
   out_6001357432987055053[6] = -nom_x[6] + true_x[6];
   out_6001357432987055053[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_6698604941463224001) {
   out_6698604941463224001[0] = 1.0;
   out_6698604941463224001[1] = 0;
   out_6698604941463224001[2] = 0;
   out_6698604941463224001[3] = 0;
   out_6698604941463224001[4] = 0;
   out_6698604941463224001[5] = 0;
   out_6698604941463224001[6] = 0;
   out_6698604941463224001[7] = 0;
   out_6698604941463224001[8] = 0;
   out_6698604941463224001[9] = 1.0;
   out_6698604941463224001[10] = 0;
   out_6698604941463224001[11] = 0;
   out_6698604941463224001[12] = 0;
   out_6698604941463224001[13] = 0;
   out_6698604941463224001[14] = 0;
   out_6698604941463224001[15] = 0;
   out_6698604941463224001[16] = 0;
   out_6698604941463224001[17] = 0;
   out_6698604941463224001[18] = 1.0;
   out_6698604941463224001[19] = 0;
   out_6698604941463224001[20] = 0;
   out_6698604941463224001[21] = 0;
   out_6698604941463224001[22] = 0;
   out_6698604941463224001[23] = 0;
   out_6698604941463224001[24] = 0;
   out_6698604941463224001[25] = 0;
   out_6698604941463224001[26] = 0;
   out_6698604941463224001[27] = 1.0;
   out_6698604941463224001[28] = 0;
   out_6698604941463224001[29] = 0;
   out_6698604941463224001[30] = 0;
   out_6698604941463224001[31] = 0;
   out_6698604941463224001[32] = 0;
   out_6698604941463224001[33] = 0;
   out_6698604941463224001[34] = 0;
   out_6698604941463224001[35] = 0;
   out_6698604941463224001[36] = 1.0;
   out_6698604941463224001[37] = 0;
   out_6698604941463224001[38] = 0;
   out_6698604941463224001[39] = 0;
   out_6698604941463224001[40] = 0;
   out_6698604941463224001[41] = 0;
   out_6698604941463224001[42] = 0;
   out_6698604941463224001[43] = 0;
   out_6698604941463224001[44] = 0;
   out_6698604941463224001[45] = 1.0;
   out_6698604941463224001[46] = 0;
   out_6698604941463224001[47] = 0;
   out_6698604941463224001[48] = 0;
   out_6698604941463224001[49] = 0;
   out_6698604941463224001[50] = 0;
   out_6698604941463224001[51] = 0;
   out_6698604941463224001[52] = 0;
   out_6698604941463224001[53] = 0;
   out_6698604941463224001[54] = 1.0;
   out_6698604941463224001[55] = 0;
   out_6698604941463224001[56] = 0;
   out_6698604941463224001[57] = 0;
   out_6698604941463224001[58] = 0;
   out_6698604941463224001[59] = 0;
   out_6698604941463224001[60] = 0;
   out_6698604941463224001[61] = 0;
   out_6698604941463224001[62] = 0;
   out_6698604941463224001[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_4727966728113728144) {
   out_4727966728113728144[0] = state[0];
   out_4727966728113728144[1] = state[1];
   out_4727966728113728144[2] = state[2];
   out_4727966728113728144[3] = state[3];
   out_4727966728113728144[4] = dt*((-state[3] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[3]))*state[5] - 9.8000000000000007*state[7] + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[4]/(mass*state[3]) + stiffness_front*(-state[1] - state[2] + state[6])*state[0]/(mass*steer_ratio)) + state[4];
   out_4727966728113728144[5] = dt*(center_to_front*stiffness_front*(-state[1] - state[2] + state[6])*state[0]/(rotational_inertia*steer_ratio) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[4]/(rotational_inertia*state[3]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[3])) + state[5];
   out_4727966728113728144[6] = state[6];
   out_4727966728113728144[7] = state[7];
}
void F_fun(double *state, double dt, double *out_6985378252109603664) {
   out_6985378252109603664[0] = 1;
   out_6985378252109603664[1] = 0;
   out_6985378252109603664[2] = 0;
   out_6985378252109603664[3] = 0;
   out_6985378252109603664[4] = 0;
   out_6985378252109603664[5] = 0;
   out_6985378252109603664[6] = 0;
   out_6985378252109603664[7] = 0;
   out_6985378252109603664[8] = 0;
   out_6985378252109603664[9] = 1;
   out_6985378252109603664[10] = 0;
   out_6985378252109603664[11] = 0;
   out_6985378252109603664[12] = 0;
   out_6985378252109603664[13] = 0;
   out_6985378252109603664[14] = 0;
   out_6985378252109603664[15] = 0;
   out_6985378252109603664[16] = 0;
   out_6985378252109603664[17] = 0;
   out_6985378252109603664[18] = 1;
   out_6985378252109603664[19] = 0;
   out_6985378252109603664[20] = 0;
   out_6985378252109603664[21] = 0;
   out_6985378252109603664[22] = 0;
   out_6985378252109603664[23] = 0;
   out_6985378252109603664[24] = 0;
   out_6985378252109603664[25] = 0;
   out_6985378252109603664[26] = 0;
   out_6985378252109603664[27] = 1;
   out_6985378252109603664[28] = 0;
   out_6985378252109603664[29] = 0;
   out_6985378252109603664[30] = 0;
   out_6985378252109603664[31] = 0;
   out_6985378252109603664[32] = dt*((-stiffness_front - stiffness_rear)*state[4]/(mass*state[3]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(mass*state[3]) + stiffness_front*(-state[1] - state[2] + state[6])/(mass*steer_ratio));
   out_6985378252109603664[33] = -dt*stiffness_front*state[0]/(mass*steer_ratio);
   out_6985378252109603664[34] = -dt*stiffness_front*state[0]/(mass*steer_ratio);
   out_6985378252109603664[35] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[3], 2)))*state[5] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[4]/(mass*pow(state[3], 2)));
   out_6985378252109603664[36] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[3]) + 1;
   out_6985378252109603664[37] = dt*(-state[3] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[3]));
   out_6985378252109603664[38] = dt*stiffness_front*state[0]/(mass*steer_ratio);
   out_6985378252109603664[39] = -9.8000000000000007*dt;
   out_6985378252109603664[40] = dt*(center_to_front*stiffness_front*(-state[1] - state[2] + state[6])/(rotational_inertia*steer_ratio) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[4]/(rotational_inertia*state[3]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[5]/(rotational_inertia*state[3]));
   out_6985378252109603664[41] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*steer_ratio);
   out_6985378252109603664[42] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*steer_ratio);
   out_6985378252109603664[43] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[4]/(rotational_inertia*pow(state[3], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[3], 2)));
   out_6985378252109603664[44] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[3]);
   out_6985378252109603664[45] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[3]) + 1;
   out_6985378252109603664[46] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*steer_ratio);
   out_6985378252109603664[47] = 0;
   out_6985378252109603664[48] = 0;
   out_6985378252109603664[49] = 0;
   out_6985378252109603664[50] = 0;
   out_6985378252109603664[51] = 0;
   out_6985378252109603664[52] = 0;
   out_6985378252109603664[53] = 0;
   out_6985378252109603664[54] = 1;
   out_6985378252109603664[55] = 0;
   out_6985378252109603664[56] = 0;
   out_6985378252109603664[57] = 0;
   out_6985378252109603664[58] = 0;
   out_6985378252109603664[59] = 0;
   out_6985378252109603664[60] = 0;
   out_6985378252109603664[61] = 0;
   out_6985378252109603664[62] = 0;
   out_6985378252109603664[63] = 1;
}
void h_25(double *state, double *unused, double *out_1759107569896847539) {
   out_1759107569896847539[0] = state[5];
}
void H_25(double *state, double *unused, double *out_8208846802942008690) {
   out_8208846802942008690[0] = 0;
   out_8208846802942008690[1] = 0;
   out_8208846802942008690[2] = 0;
   out_8208846802942008690[3] = 0;
   out_8208846802942008690[4] = 0;
   out_8208846802942008690[5] = 1;
   out_8208846802942008690[6] = 0;
   out_8208846802942008690[7] = 0;
}
void h_24(double *state, double *unused, double *out_6382576662856627788) {
   out_6382576662856627788[0] = state[3];
   out_6382576662856627788[1] = state[4];
}
void H_24(double *state, double *unused, double *out_170883007819729627) {
   out_170883007819729627[0] = 0;
   out_170883007819729627[1] = 0;
   out_170883007819729627[2] = 0;
   out_170883007819729627[3] = 1;
   out_170883007819729627[4] = 0;
   out_170883007819729627[5] = 0;
   out_170883007819729627[6] = 0;
   out_170883007819729627[7] = 0;
   out_170883007819729627[8] = 0;
   out_170883007819729627[9] = 0;
   out_170883007819729627[10] = 0;
   out_170883007819729627[11] = 0;
   out_170883007819729627[12] = 1;
   out_170883007819729627[13] = 0;
   out_170883007819729627[14] = 0;
   out_170883007819729627[15] = 0;
}
void h_30(double *state, double *unused, double *out_4637493100797381955) {
   out_4637493100797381955[0] = state[3];
}
void H_30(double *state, double *unused, double *out_3819533153493876732) {
   out_3819533153493876732[0] = 0;
   out_3819533153493876732[1] = 0;
   out_3819533153493876732[2] = 0;
   out_3819533153493876732[3] = 1;
   out_3819533153493876732[4] = 0;
   out_3819533153493876732[5] = 0;
   out_3819533153493876732[6] = 0;
   out_3819533153493876732[7] = 0;
}
void h_26(double *state, double *unused, double *out_1120491813196356001) {
   out_1120491813196356001[0] = state[6];
}
void H_26(double *state, double *unused, double *out_7081947745291931860) {
   out_7081947745291931860[0] = 0;
   out_7081947745291931860[1] = 0;
   out_7081947745291931860[2] = 0;
   out_7081947745291931860[3] = 0;
   out_7081947745291931860[4] = 0;
   out_7081947745291931860[5] = 0;
   out_7081947745291931860[6] = 1;
   out_7081947745291931860[7] = 0;
}
void h_27(double *state, double *unused, double *out_7517092483890585495) {
   out_7517092483890585495[0] = state[2];
}
void H_27(double *state, double *unused, double *out_4647261601950982249) {
   out_4647261601950982249[0] = 0;
   out_4647261601950982249[1] = 0;
   out_4647261601950982249[2] = 1;
   out_4647261601950982249[3] = 0;
   out_4647261601950982249[4] = 0;
   out_4647261601950982249[5] = 0;
   out_4647261601950982249[6] = 0;
   out_4647261601950982249[7] = 0;
}
void h_29(double *state, double *unused, double *out_1404140875136843641) {
   out_1404140875136843641[0] = steer_ratio;
}
void H_29(double *state, double *unused, double *out_8370634557688195934) {
   out_8370634557688195934[0] = 0;
   out_8370634557688195934[1] = 0;
   out_8370634557688195934[2] = 0;
   out_8370634557688195934[3] = 0;
   out_8370634557688195934[4] = 0;
   out_8370634557688195934[5] = 0;
   out_8370634557688195934[6] = 0;
   out_8370634557688195934[7] = 0;
}
void h_28(double *state, double *unused, double *out_8919056942041042463) {
   out_8919056942041042463[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2397400043882984835) {
   out_2397400043882984835[0] = 1;
   out_2397400043882984835[1] = 0;
   out_2397400043882984835[2] = 0;
   out_2397400043882984835[3] = 0;
   out_2397400043882984835[4] = 0;
   out_2397400043882984835[5] = 0;
   out_2397400043882984835[6] = 0;
   out_2397400043882984835[7] = 0;
}
void h_31(double *state, double *unused, double *out_7378609714787003643) {
   out_7378609714787003643[0] = state[7];
}
void H_31(double *state, double *unused, double *out_5660203173478410483) {
   out_5660203173478410483[0] = 0;
   out_5660203173478410483[1] = 0;
   out_5660203173478410483[2] = 0;
   out_5660203173478410483[3] = 0;
   out_5660203173478410483[4] = 0;
   out_5660203173478410483[5] = 0;
   out_5660203173478410483[6] = 0;
   out_5660203173478410483[7] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3307893292679224287) {
  err_fun(nom_x, delta_x, out_3307893292679224287);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6001357432987055053) {
  inv_err_fun(nom_x, true_x, out_6001357432987055053);
}
void car_H_mod_fun(double *state, double *out_6698604941463224001) {
  H_mod_fun(state, out_6698604941463224001);
}
void car_f_fun(double *state, double dt, double *out_4727966728113728144) {
  f_fun(state,  dt, out_4727966728113728144);
}
void car_F_fun(double *state, double dt, double *out_6985378252109603664) {
  F_fun(state,  dt, out_6985378252109603664);
}
void car_h_25(double *state, double *unused, double *out_1759107569896847539) {
  h_25(state, unused, out_1759107569896847539);
}
void car_H_25(double *state, double *unused, double *out_8208846802942008690) {
  H_25(state, unused, out_8208846802942008690);
}
void car_h_24(double *state, double *unused, double *out_6382576662856627788) {
  h_24(state, unused, out_6382576662856627788);
}
void car_H_24(double *state, double *unused, double *out_170883007819729627) {
  H_24(state, unused, out_170883007819729627);
}
void car_h_30(double *state, double *unused, double *out_4637493100797381955) {
  h_30(state, unused, out_4637493100797381955);
}
void car_H_30(double *state, double *unused, double *out_3819533153493876732) {
  H_30(state, unused, out_3819533153493876732);
}
void car_h_26(double *state, double *unused, double *out_1120491813196356001) {
  h_26(state, unused, out_1120491813196356001);
}
void car_H_26(double *state, double *unused, double *out_7081947745291931860) {
  H_26(state, unused, out_7081947745291931860);
}
void car_h_27(double *state, double *unused, double *out_7517092483890585495) {
  h_27(state, unused, out_7517092483890585495);
}
void car_H_27(double *state, double *unused, double *out_4647261601950982249) {
  H_27(state, unused, out_4647261601950982249);
}
void car_h_29(double *state, double *unused, double *out_1404140875136843641) {
  h_29(state, unused, out_1404140875136843641);
}
void car_H_29(double *state, double *unused, double *out_8370634557688195934) {
  H_29(state, unused, out_8370634557688195934);
}
void car_h_28(double *state, double *unused, double *out_8919056942041042463) {
  h_28(state, unused, out_8919056942041042463);
}
void car_H_28(double *state, double *unused, double *out_2397400043882984835) {
  H_28(state, unused, out_2397400043882984835);
}
void car_h_31(double *state, double *unused, double *out_7378609714787003643) {
  h_31(state, unused, out_7378609714787003643);
}
void car_H_31(double *state, double *unused, double *out_5660203173478410483) {
  H_31(state, unused, out_5660203173478410483);
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
void car_set_steer_ratio(double x) {
  set_steer_ratio(x);
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
    { "steer_ratio", car_set_steer_ratio },
  },
  .extra_routines = {
  },
};

ekf_init(car);
