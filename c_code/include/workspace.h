#ifndef WORKSPACE_H
#define WORKSPACE_H

/*
 * This file was autogenerated by OSQP-Matlab on April 24, 2019 at 15:33:55.
 * 
 * This file contains the prototypes for all the workspace variables needed
 * by OSQP. The actual data is contained inside workspace.c.
 */

#include "types.h"
#include "qdldl_interface.h"

// Data structure prototypes
extern csc Pdata;
extern csc Adata;
extern c_float qdata[15];
extern c_float ldata[198];
extern c_float udata[198];
extern OSQPData data;

// Settings structure prototype
extern OSQPSettings settings;

// Scaling structure prototypes
extern c_float Dscaling[15];
extern c_float Dinvscaling[15];
extern c_float Escaling[198];
extern c_float Einvscaling[198];
extern OSQPScaling scaling;

// Prototypes for linsys_solver structure
extern csc linsys_solver_L;
extern c_float linsys_solver_Dinv[213];
extern c_int linsys_solver_P[213];
extern c_float linsys_solver_bp[213];
extern c_int linsys_solver_Pdiag_idx[15];
extern csc linsys_solver_KKT;
extern c_int linsys_solver_PtoKKT[120];
extern c_int linsys_solver_AtoKKT[736];
extern c_int linsys_solver_rhotoKKT[198];
extern QDLDL_float linsys_solver_D[213];
extern QDLDL_int linsys_solver_etree[213];
extern QDLDL_int linsys_solver_Lnz[213];
extern QDLDL_int   linsys_solver_iwork[639];
extern QDLDL_bool  linsys_solver_bwork[213];
extern QDLDL_float linsys_solver_fwork[213];
extern qdldl_solver linsys_solver;

// Prototypes for solution
extern c_float xsolution[15];
extern c_float ysolution[198];

extern OSQPSolution solution;

// Prototype for info structure
extern OSQPInfo info;

// Prototypes for the workspace
extern c_float work_rho_vec[198];
extern c_float work_rho_inv_vec[198];
extern c_int work_constr_type[198];
extern c_float work_x[15];
extern c_float work_y[198];
extern c_float work_z[198];
extern c_float work_xz_tilde[213];
extern c_float work_x_prev[15];
extern c_float work_z_prev[198];
extern c_float work_Ax[198];
extern c_float work_Px[15];
extern c_float work_Aty[15];
extern c_float work_delta_y[198];
extern c_float work_Atdelta_y[15];
extern c_float work_delta_x[15];
extern c_float work_Pdelta_x[15];
extern c_float work_Adelta_x[198];
extern c_float work_D_temp[15];
extern c_float work_D_temp_A[15];
extern c_float work_E_temp[198];

extern OSQPWorkspace workspace;

#endif // ifndef WORKSPACE_H
