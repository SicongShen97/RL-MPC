/*
 * AD tool to FORCESPRO Template - missing information to be filled in by createADTool.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2022. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "include/FrankaFORCESNLPsolver_5mm.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "FrankaFORCESNLPsolver_5mm_model.h"



/* copies data from sparse matrix into a dense one */
static void FrankaFORCESNLPsolver_5mm_sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, FrankaFORCESNLPsolver_5mm_callback_float *data, FrankaFORCESNLPsolver_5mm_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((FrankaFORCESNLPsolver_5mm_float) data[j]);
        }
    }
}




/* AD tool to FORCESPRO interface */
extern solver_int32_default FrankaFORCESNLPsolver_5mm_adtool2forces(FrankaFORCESNLPsolver_5mm_float *x,        /* primal vars                                         */
                                 FrankaFORCESNLPsolver_5mm_float *y,        /* eq. constraint multiplers                           */
                                 FrankaFORCESNLPsolver_5mm_float *l,        /* ineq. constraint multipliers                        */
                                 FrankaFORCESNLPsolver_5mm_float *p,        /* parameters                                          */
                                 FrankaFORCESNLPsolver_5mm_float *f,        /* objective function (scalar)                         */
                                 FrankaFORCESNLPsolver_5mm_float *nabla_f,  /* gradient of objective function                      */
                                 FrankaFORCESNLPsolver_5mm_float *c,        /* dynamics                                            */
                                 FrankaFORCESNLPsolver_5mm_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FrankaFORCESNLPsolver_5mm_float *h,        /* inequality constraints                              */
                                 FrankaFORCESNLPsolver_5mm_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FrankaFORCESNLPsolver_5mm_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* AD tool input and output arrays */
    const FrankaFORCESNLPsolver_5mm_callback_float *in[4];
    FrankaFORCESNLPsolver_5mm_callback_float *out[7];
	

	/* Allocate working arrays for AD tool */
	FrankaFORCESNLPsolver_5mm_float w[8];
	
    /* temporary storage for AD tool sparse output */
    FrankaFORCESNLPsolver_5mm_callback_float this_f;
    FrankaFORCESNLPsolver_5mm_float nabla_f_sparse[4];
    FrankaFORCESNLPsolver_5mm_float h_sparse[1];
    FrankaFORCESNLPsolver_5mm_float nabla_h_sparse[2];
    FrankaFORCESNLPsolver_5mm_float c_sparse[2];
    FrankaFORCESNLPsolver_5mm_float nabla_c_sparse[4];
            
    
    /* pointers to row and column info for 
     * column compressed format used by AD tool */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for AD tool */
    in[0] = x;
    in[1] = p;
    in[2] = l;
    in[3] = y;

	if ((0 <= stage && stage <= 8))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FrankaFORCESNLPsolver_5mm_objective_0(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = FrankaFORCESNLPsolver_5mm_objective_0_sparsity_out(1)[0];
			ncol = FrankaFORCESNLPsolver_5mm_objective_0_sparsity_out(1)[1];
			colind = FrankaFORCESNLPsolver_5mm_objective_0_sparsity_out(1) + 2;
			row = FrankaFORCESNLPsolver_5mm_objective_0_sparsity_out(1) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_5mm_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		FrankaFORCESNLPsolver_5mm_dynamics_0(in, out, NULL, w, 0);
		if( c )
		{
			nrow = FrankaFORCESNLPsolver_5mm_dynamics_0_sparsity_out(0)[0];
			ncol = FrankaFORCESNLPsolver_5mm_dynamics_0_sparsity_out(0)[1];
			colind = FrankaFORCESNLPsolver_5mm_dynamics_0_sparsity_out(0) + 2;
			row = FrankaFORCESNLPsolver_5mm_dynamics_0_sparsity_out(0) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_5mm_sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}
		if( nabla_c )
		{
			nrow = FrankaFORCESNLPsolver_5mm_dynamics_0_sparsity_out(1)[0];
			ncol = FrankaFORCESNLPsolver_5mm_dynamics_0_sparsity_out(1)[1];
			colind = FrankaFORCESNLPsolver_5mm_dynamics_0_sparsity_out(1) + 2;
			row = FrankaFORCESNLPsolver_5mm_dynamics_0_sparsity_out(1) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_5mm_sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FrankaFORCESNLPsolver_5mm_inequalities_0(in, out, NULL, w, 0);
		if( h )
		{
			nrow = FrankaFORCESNLPsolver_5mm_inequalities_0_sparsity_out(0)[0];
			ncol = FrankaFORCESNLPsolver_5mm_inequalities_0_sparsity_out(0)[1];
			colind = FrankaFORCESNLPsolver_5mm_inequalities_0_sparsity_out(0) + 2;
			row = FrankaFORCESNLPsolver_5mm_inequalities_0_sparsity_out(0) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_5mm_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = FrankaFORCESNLPsolver_5mm_inequalities_0_sparsity_out(1)[0];
			ncol = FrankaFORCESNLPsolver_5mm_inequalities_0_sparsity_out(1)[1];
			colind = FrankaFORCESNLPsolver_5mm_inequalities_0_sparsity_out(1) + 2;
			row = FrankaFORCESNLPsolver_5mm_inequalities_0_sparsity_out(1) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_5mm_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
	if ((9 == stage))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FrankaFORCESNLPsolver_5mm_objective_1(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = FrankaFORCESNLPsolver_5mm_objective_1_sparsity_out(1)[0];
			ncol = FrankaFORCESNLPsolver_5mm_objective_1_sparsity_out(1)[1];
			colind = FrankaFORCESNLPsolver_5mm_objective_1_sparsity_out(1) + 2;
			row = FrankaFORCESNLPsolver_5mm_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_5mm_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FrankaFORCESNLPsolver_5mm_inequalities_1(in, out, NULL, w, 0);
		if( h )
		{
			nrow = FrankaFORCESNLPsolver_5mm_inequalities_1_sparsity_out(0)[0];
			ncol = FrankaFORCESNLPsolver_5mm_inequalities_1_sparsity_out(0)[1];
			colind = FrankaFORCESNLPsolver_5mm_inequalities_1_sparsity_out(0) + 2;
			row = FrankaFORCESNLPsolver_5mm_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_5mm_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = FrankaFORCESNLPsolver_5mm_inequalities_1_sparsity_out(1)[0];
			ncol = FrankaFORCESNLPsolver_5mm_inequalities_1_sparsity_out(1)[1];
			colind = FrankaFORCESNLPsolver_5mm_inequalities_1_sparsity_out(1) + 2;
			row = FrankaFORCESNLPsolver_5mm_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_5mm_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((FrankaFORCESNLPsolver_5mm_float) this_f);
    }

    return 0;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
