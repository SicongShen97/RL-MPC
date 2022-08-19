/*
FORCESNLPsolver1_1 : A fast customized optimization solver.

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

#include "mex.h"
#include "math.h"
#include "../include/FORCESNLPsolver1_1.h"
#include "../include/FORCESNLPsolver1_1_memory.h"
#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif



/* copy functions */

void copyCArrayToM_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double) (*src++) ;
    }
}

void copyMValueToC_double(double * src, double * dest)
{
	*dest = (double) *src;
}

/* copy functions */

void copyCArrayToM_solver_int32_default(solver_int32_default *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_solver_int32_default(double *src, solver_int32_default *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (solver_int32_default) (*src++) ;
    }
}

void copyMValueToC_solver_int32_default(double * src, solver_int32_default * dest)
{
	*dest = (solver_int32_default) *src;
}



extern solver_int32_default (FORCESNLPsolver1_1_float *x, FORCESNLPsolver1_1_float *y, FORCESNLPsolver1_1_float *l, FORCESNLPsolver1_1_float *p, FORCESNLPsolver1_1_float *f, FORCESNLPsolver1_1_float *nabla_f, FORCESNLPsolver1_1_float *c, FORCESNLPsolver1_1_float *nabla_c, FORCESNLPsolver1_1_float *h, FORCESNLPsolver1_1_float *nabla_h, FORCESNLPsolver1_1_float *hess, solver_int32_default stage, solver_int32_default iteration, solver_int32_default threadID);
FORCESNLPsolver1_1_extfunc pt2function_FORCESNLPsolver1_1 = &;


/* Some memory for mex-function */
static FORCESNLPsolver1_1_params params;
static FORCESNLPsolver1_1_output output;
static FORCESNLPsolver1_1_info info;
static FORCESNLPsolver1_1_mem * mem;

/* THE mex-function */
void mexFunction( solver_int32_default nlhs, mxArray *plhs[], solver_int32_default nrhs, const mxArray *prhs[] )  
{
	/* file pointer for printing */
	FILE *fp = NULL;

	/* define variables */	
	mxArray *par;
	mxArray *outvar;
	const mxArray *PARAMS = prhs[0]; 
	double *pvalue;
	solver_int32_default i;
	solver_int32_default exitflag;
	const solver_int8_default *fname;
	const solver_int8_default *outputnames[8] = {"x1","x2","x3","x4","x5","x6","x7","x8"};
	const solver_int8_default *infofields[11] = { "it", "it2opt", "res_eq", "res_ineq", "rsnorm", "rcompnorm", "pobj", "mu", "solvetime", "fevalstime", "solver_id"};
	
	/* Check for proper number of arguments */
    if (nrhs != 1)
	{
		mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help FORCESNLPsolver1_1_mex' for details.");
	}    
	if (nlhs > 3) 
	{
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help FORCESNLPsolver1_1_mex' for details.");
    }

	/* Check whether params is actually a structure */
	if( !mxIsStruct(PARAMS) ) 
	{
		mexErrMsgTxt("PARAMS must be a structure.");
	}
	 

    /* initialize memory */
    if (mem == NULL)
    {
        mem = FORCESNLPsolver1_1_internal_mem(0);
    }

	/* copy parameters into the right location */
	par = mxGetField(PARAMS, 0, "xinit");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.xinit not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.xinit must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.xinit must be of size [6 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.xinit,6);

	}
	par = mxGetField(PARAMS, 0, "x0");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.x0 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.x0 must be a double.");
    }
    if( mxGetM(par) != 80 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.x0 must be of size [80 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.x0,80);

	}
	par = mxGetField(PARAMS, 0, "all_parameters");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.all_parameters not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.all_parameters must be a double.");
    }
    if( mxGetM(par) != 144 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.all_parameters must be of size [144 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.all_parameters,144);

	}


	#if SET_PRINTLEVEL_FORCESNLPsolver1_1 > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* call solver */

	exitflag = FORCESNLPsolver1_1_solve(&params, &output, &info, mem, fp, pt2function_FORCESNLPsolver1_1);
	
	#if SET_PRINTLEVEL_FORCESNLPsolver1_1 > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			mexPrintf("%c",i);
		}
		fclose(fp);
	#endif

	/* copy output to matlab arrays */
	plhs[0] = mxCreateStructMatrix(1, 1, 8, outputnames);
		outvar = mxCreateDoubleMatrix(10, 1, mxREAL);
	copyCArrayToM_double( output.x1, mxGetPr(outvar), 10);
	mxSetField(plhs[0], 0, "x1", outvar);

	outvar = mxCreateDoubleMatrix(10, 1, mxREAL);
	copyCArrayToM_double( output.x2, mxGetPr(outvar), 10);
	mxSetField(plhs[0], 0, "x2", outvar);

	outvar = mxCreateDoubleMatrix(10, 1, mxREAL);
	copyCArrayToM_double( output.x3, mxGetPr(outvar), 10);
	mxSetField(plhs[0], 0, "x3", outvar);

	outvar = mxCreateDoubleMatrix(10, 1, mxREAL);
	copyCArrayToM_double( output.x4, mxGetPr(outvar), 10);
	mxSetField(plhs[0], 0, "x4", outvar);

	outvar = mxCreateDoubleMatrix(10, 1, mxREAL);
	copyCArrayToM_double( output.x5, mxGetPr(outvar), 10);
	mxSetField(plhs[0], 0, "x5", outvar);

	outvar = mxCreateDoubleMatrix(10, 1, mxREAL);
	copyCArrayToM_double( output.x6, mxGetPr(outvar), 10);
	mxSetField(plhs[0], 0, "x6", outvar);

	outvar = mxCreateDoubleMatrix(10, 1, mxREAL);
	copyCArrayToM_double( output.x7, mxGetPr(outvar), 10);
	mxSetField(plhs[0], 0, "x7", outvar);

	outvar = mxCreateDoubleMatrix(10, 1, mxREAL);
	copyCArrayToM_double( output.x8, mxGetPr(outvar), 10);
	mxSetField(plhs[0], 0, "x8", outvar);



	/* copy exitflag */
	if( nlhs > 1 )
	{
	plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
	*mxGetPr(plhs[1]) = (double)exitflag;
	}

	/* copy info struct */
	if( nlhs > 2 )
	{
	        plhs[2] = mxCreateStructMatrix(1, 1, 11, infofields);
         
		
		/* iterations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it;
		mxSetField(plhs[2], 0, "it", outvar);

		/* iterations to optimality (branch and bound) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it2opt;
		mxSetField(plhs[2], 0, "it2opt", outvar);
		
		/* res_eq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_eq;
		mxSetField(plhs[2], 0, "res_eq", outvar);

		/* res_ineq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_ineq;
		mxSetField(plhs[2], 0, "res_ineq", outvar);

		/* rsnorm */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.rsnorm;
		mxSetField(plhs[2], 0, "rsnorm", outvar);

		/* rcompnorm */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.rcompnorm;
		mxSetField(plhs[2], 0, "rcompnorm", outvar);
		
		/* pobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.pobj;
		mxSetField(plhs[2], 0, "pobj", outvar);

		/* mu */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu;
		mxSetField(plhs[2], 0, "mu", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.solvetime;
		mxSetField(plhs[2], 0, "solvetime", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.fevalstime;
		mxSetField(plhs[2], 0, "fevalstime", outvar);

		/* solver ID */
		outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
		copyCArrayToM_solver_int32_default(info.solver_id, mxGetPr(outvar), 8);
		mxSetField(plhs[2], 0, "solver_id", outvar);	

	}
}
