/*
FrankaFORCESNLPsolver_dyn_sqr_obsts : A fast customized optimization solver.

Copyright (C) 2013-2022 EMBOTECH AG [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

/* Generated by FORCESPRO v6.0.0 on Wednesday, August 31, 2022 at 9:46:24 AM */
#ifndef FrankaFORCESNLPsolver_dyn_sqr_obsts_H
#define FrankaFORCESNLPsolver_dyn_sqr_obsts_H

#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif
#ifndef SOLVER_STRING_H
#define SOLVER_STRING_H
#include <string.h>
#endif


#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif


/* DATA TYPE ------------------------------------------------------------*/
typedef double FrankaFORCESNLPsolver_dyn_sqr_obsts_float;
typedef double FrankaFORCESNLPsolver_dyn_sqr_obsts_ldl_s_float;
typedef double FrankaFORCESNLPsolver_dyn_sqr_obsts_ldl_r_float;
typedef double FrankaFORCESNLPsolver_dyn_sqr_obsts_callback_float;

typedef double FrankaFORCESNLPsolver_dyn_sqr_obstsinterface_float;

/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_FrankaFORCESNLPsolver_dyn_sqr_obsts
#define MISRA_C_FrankaFORCESNLPsolver_dyn_sqr_obsts (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_FrankaFORCESNLPsolver_dyn_sqr_obsts
#define RESTRICT_CODE_FrankaFORCESNLPsolver_dyn_sqr_obsts (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_FrankaFORCESNLPsolver_dyn_sqr_obsts
#define SET_PRINTLEVEL_FrankaFORCESNLPsolver_dyn_sqr_obsts    (0)
#endif

/* timing */
#ifndef SET_TIMING_FrankaFORCESNLPsolver_dyn_sqr_obsts
#define SET_TIMING_FrankaFORCESNLPsolver_dyn_sqr_obsts    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_FrankaFORCESNLPsolver_dyn_sqr_obsts			(200)	

/* scaling factor of line search (FTB rule) */
#define SET_FLS_SCALE_FrankaFORCESNLPsolver_dyn_sqr_obsts		(FrankaFORCESNLPsolver_dyn_sqr_obsts_float)(0.99)      

/* maximum number of supported elements in the filter */
#define MAX_FILTER_SIZE_FrankaFORCESNLPsolver_dyn_sqr_obsts	(200) 

/* maximum number of supported elements in the filter */
#define MAX_SOC_IT_FrankaFORCESNLPsolver_dyn_sqr_obsts			(4) 

/* desired relative duality gap */
#define SET_ACC_RDGAP_FrankaFORCESNLPsolver_dyn_sqr_obsts		(FrankaFORCESNLPsolver_dyn_sqr_obsts_float)(0.0001)

/* desired maximum residual on equality constraints */
#define SET_ACC_RESEQ_FrankaFORCESNLPsolver_dyn_sqr_obsts		(FrankaFORCESNLPsolver_dyn_sqr_obsts_float)(1E-06)

/* desired maximum residual on inequality constraints */
#define SET_ACC_RESINEQ_FrankaFORCESNLPsolver_dyn_sqr_obsts	(FrankaFORCESNLPsolver_dyn_sqr_obsts_float)(1E-06)

/* desired maximum violation of complementarity */
#define SET_ACC_KKTCOMPL_FrankaFORCESNLPsolver_dyn_sqr_obsts	(FrankaFORCESNLPsolver_dyn_sqr_obsts_float)(1E-06)


/* SOLVER RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_FrankaFORCESNLPsolver_dyn_sqr_obsts      (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_FrankaFORCESNLPsolver_dyn_sqr_obsts (0)

/* solver has stopped due to a timeout */
#define TIMEOUT_FrankaFORCESNLPsolver_dyn_sqr_obsts   (2)

/* solver stopped externally */
#define EXIT_EXTERNAL_FrankaFORCESNLPsolver_dyn_sqr_obsts (3)

/* wrong number of inequalities error */
#define INVALID_NUM_INEQ_ERROR_FrankaFORCESNLPsolver_dyn_sqr_obsts  (-4)

/* factorization error */
#define FACTORIZATION_ERROR_FrankaFORCESNLPsolver_dyn_sqr_obsts   (-5)

/* NaN encountered in function evaluations */
#define BADFUNCEVAL_FrankaFORCESNLPsolver_dyn_sqr_obsts  (-6)

/* no progress in method possible */
#define NOPROGRESS_FrankaFORCESNLPsolver_dyn_sqr_obsts   (-7)

/* regularization error */
#define REGULARIZATION_ERROR_FrankaFORCESNLPsolver_dyn_sqr_obsts   (-9)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_FrankaFORCESNLPsolver_dyn_sqr_obsts   (-11)

/* too small timeout given */
#define INVALID_TIMEOUT_FrankaFORCESNLPsolver_dyn_sqr_obsts   (-12)

/* thread error */
#define THREAD_FAILURE_FrankaFORCESNLPsolver_dyn_sqr_obsts  (-98)

/* locking mechanism error */
#define LOCK_FAILURE_FrankaFORCESNLPsolver_dyn_sqr_obsts  (-99)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_FrankaFORCESNLPsolver_dyn_sqr_obsts  (-100)

/* Insufficient number of internal memory instances.
 * Increase codeoptions.max_num_mem. */
#define MEMORY_INVALID_FrankaFORCESNLPsolver_dyn_sqr_obsts (-101)
/* Number of threads larger than specified.
 * Increase codeoptions.nlp.max_num_threads. */
#define NUMTHREADS_INVALID_FrankaFORCESNLPsolver_dyn_sqr_obsts (-102)

/* INTEGRATORS RETURN CODE ------------*/
/* Integrator ran successfully */
#define INTEGRATOR_SUCCESS (11)
/* Number of steps set by user exceeds maximum number of steps allowed */
#define INTEGRATOR_MAXSTEPS_EXCEEDED (12)


/* MEMORY STRUCT --------------------------------------------------------*/
typedef struct FrankaFORCESNLPsolver_dyn_sqr_obsts_mem FrankaFORCESNLPsolver_dyn_sqr_obsts_mem;
#ifdef __cplusplus
extern "C" {
#endif
/* MEMORY STRUCT --------------------------------------------------------*/
extern FrankaFORCESNLPsolver_dyn_sqr_obsts_mem * FrankaFORCESNLPsolver_dyn_sqr_obsts_external_mem(void * mem_ptr, solver_int32_unsigned i_mem, size_t mem_size);
extern size_t FrankaFORCESNLPsolver_dyn_sqr_obsts_get_mem_size( void );
extern size_t FrankaFORCESNLPsolver_dyn_sqr_obsts_get_const_size( void );
#ifdef __cplusplus
}
#endif

/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct
{
    /* vector of size 70 */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float x0[70];

    /* vector of size 3 */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float xinit[3];

    /* vector of size 180 */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float all_parameters[180];


} FrankaFORCESNLPsolver_dyn_sqr_obsts_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct
{
    /* vector of size 7 */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float x01[7];

    /* vector of size 7 */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float x02[7];

    /* vector of size 7 */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float x03[7];

    /* vector of size 7 */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float x04[7];

    /* vector of size 7 */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float x05[7];

    /* vector of size 7 */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float x06[7];

    /* vector of size 7 */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float x07[7];

    /* vector of size 7 */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float x08[7];

    /* vector of size 7 */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float x09[7];

    /* vector of size 7 */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float x10[7];


} FrankaFORCESNLPsolver_dyn_sqr_obsts_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct
{
    /* iteration number */
    solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;
	
    /* inf-norm of equality constraint residuals */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float res_eq;
	
    /* inf-norm of inequality constraint residuals */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float res_ineq;

	/* norm of stationarity condition */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float rsnorm;

	/* max of all complementarity violations */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float rcompnorm;

    /* primal objective */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float pobj;	
	
    /* dual objective */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float dobj;	

    /* duality gap := pobj - dobj */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float rdgap;		

    /* duality measure */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float mu;

	/* duality measure (after affine step) */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float mu_aff;
	
    /* centering parameter */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float sigma;
	
    /* number of backtracking line search steps (affine direction) */
    solver_int32_default lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    solver_int32_default lsit_cc;
    
    /* step size (affine direction) */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float step_aff;
    
    /* step size (combined direction) */
    FrankaFORCESNLPsolver_dyn_sqr_obsts_float step_cc;    

	/* solvertime */
	FrankaFORCESNLPsolver_dyn_sqr_obsts_float solvetime;   

	/* time spent in function evaluations */
	FrankaFORCESNLPsolver_dyn_sqr_obsts_float fevalstime;  

    /* solver ID of FORCESPRO solver */
    solver_int32_default solver_id[8];  


} FrankaFORCESNLPsolver_dyn_sqr_obsts_info;









/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* Time of Solver Generation: (UTC) Wednesday, August 31, 2022 9:46:25 AM */
/* User License expires on: (UTC) Monday, October 3, 2022 10:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: (UTC) Monday, October 3, 2022 10:00:00 PM (approx.) */
/* Solver Id: b7b3ec99-fa9d-4d3b-ae75-a8683b526193 */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif		

typedef solver_int32_default (*FrankaFORCESNLPsolver_dyn_sqr_obsts_extfunc)(FrankaFORCESNLPsolver_dyn_sqr_obsts_float* x, FrankaFORCESNLPsolver_dyn_sqr_obsts_float* y, FrankaFORCESNLPsolver_dyn_sqr_obsts_float* lambda, FrankaFORCESNLPsolver_dyn_sqr_obsts_float* params, FrankaFORCESNLPsolver_dyn_sqr_obsts_float* pobj, FrankaFORCESNLPsolver_dyn_sqr_obsts_float* g, FrankaFORCESNLPsolver_dyn_sqr_obsts_float* c, FrankaFORCESNLPsolver_dyn_sqr_obsts_float* Jeq, FrankaFORCESNLPsolver_dyn_sqr_obsts_float* h, FrankaFORCESNLPsolver_dyn_sqr_obsts_float* Jineq, FrankaFORCESNLPsolver_dyn_sqr_obsts_float* H, solver_int32_default stage, solver_int32_default iterations, solver_int32_default threadID);

extern solver_int32_default FrankaFORCESNLPsolver_dyn_sqr_obsts_solve(FrankaFORCESNLPsolver_dyn_sqr_obsts_params *params, FrankaFORCESNLPsolver_dyn_sqr_obsts_output *output, FrankaFORCESNLPsolver_dyn_sqr_obsts_info *info, FrankaFORCESNLPsolver_dyn_sqr_obsts_mem *mem, FILE *fs, FrankaFORCESNLPsolver_dyn_sqr_obsts_extfunc evalextfunctions_FrankaFORCESNLPsolver_dyn_sqr_obsts);













#ifdef __cplusplus
}
#endif

#endif
