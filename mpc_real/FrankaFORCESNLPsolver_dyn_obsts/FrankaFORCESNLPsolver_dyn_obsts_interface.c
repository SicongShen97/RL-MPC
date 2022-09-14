/*
 * AD tool to FORCESPRO Template - missing information to be filled in by createADTool.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2022. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "include/FrankaFORCESNLPsolver_dyn_obsts.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "FrankaFORCESNLPsolver_dyn_obsts_model.h"



/* copies data from sparse matrix into a dense one */
static void FrankaFORCESNLPsolver_dyn_obsts_sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, FrankaFORCESNLPsolver_dyn_obsts_callback_float *data, FrankaFORCESNLPsolver_dyn_obsts_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((FrankaFORCESNLPsolver_dyn_obsts_float) data[j]);
        }
    }
}




/* AD tool to FORCESPRO interface */
extern solver_int32_default FrankaFORCESNLPsolver_dyn_obsts_adtool2forces(FrankaFORCESNLPsolver_dyn_obsts_float *x,        /* primal vars                                         */
                                 FrankaFORCESNLPsolver_dyn_obsts_float *y,        /* eq. constraint multiplers                           */
                                 FrankaFORCESNLPsolver_dyn_obsts_float *l,        /* ineq. constraint multipliers                        */
                                 FrankaFORCESNLPsolver_dyn_obsts_float *p,        /* parameters                                          */
                                 FrankaFORCESNLPsolver_dyn_obsts_float *f,        /* objective function (scalar)                         */
                                 FrankaFORCESNLPsolver_dyn_obsts_float *nabla_f,  /* gradient of objective function                      */
                                 FrankaFORCESNLPsolver_dyn_obsts_float *c,        /* dynamics                                            */
                                 FrankaFORCESNLPsolver_dyn_obsts_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FrankaFORCESNLPsolver_dyn_obsts_float *h,        /* inequality constraints                              */
                                 FrankaFORCESNLPsolver_dyn_obsts_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FrankaFORCESNLPsolver_dyn_obsts_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* AD tool input and output arrays */
    const FrankaFORCESNLPsolver_dyn_obsts_callback_float *in[4];
    FrankaFORCESNLPsolver_dyn_obsts_callback_float *out[7];
	

	/* Allocate working arrays for AD tool */
	FrankaFORCESNLPsolver_dyn_obsts_float w[18];
	
    /* temporary storage for AD tool sparse output */
    FrankaFORCESNLPsolver_dyn_obsts_callback_float this_f;
    FrankaFORCESNLPsolver_dyn_obsts_float nabla_f_sparse[5];
    FrankaFORCESNLPsolver_dyn_obsts_float h_sparse[2];
    FrankaFORCESNLPsolver_dyn_obsts_float nabla_h_sparse[6];
    FrankaFORCESNLPsolver_dyn_obsts_float c_sparse[3];
    FrankaFORCESNLPsolver_dyn_obsts_float nabla_c_sparse[5];
            
    
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
		FrankaFORCESNLPsolver_dyn_obsts_objective_0(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = FrankaFORCESNLPsolver_dyn_obsts_objective_0_sparsity_out(1)[0];
			ncol = FrankaFORCESNLPsolver_dyn_obsts_objective_0_sparsity_out(1)[1];
			colind = FrankaFORCESNLPsolver_dyn_obsts_objective_0_sparsity_out(1) + 2;
			row = FrankaFORCESNLPsolver_dyn_obsts_objective_0_sparsity_out(1) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_dyn_obsts_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		FrankaFORCESNLPsolver_dyn_obsts_dynamics_0(in, out, NULL, w, 0);
		if( c )
		{
			nrow = FrankaFORCESNLPsolver_dyn_obsts_dynamics_0_sparsity_out(0)[0];
			ncol = FrankaFORCESNLPsolver_dyn_obsts_dynamics_0_sparsity_out(0)[1];
			colind = FrankaFORCESNLPsolver_dyn_obsts_dynamics_0_sparsity_out(0) + 2;
			row = FrankaFORCESNLPsolver_dyn_obsts_dynamics_0_sparsity_out(0) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_dyn_obsts_sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}
		if( nabla_c )
		{
			nrow = FrankaFORCESNLPsolver_dyn_obsts_dynamics_0_sparsity_out(1)[0];
			ncol = FrankaFORCESNLPsolver_dyn_obsts_dynamics_0_sparsity_out(1)[1];
			colind = FrankaFORCESNLPsolver_dyn_obsts_dynamics_0_sparsity_out(1) + 2;
			row = FrankaFORCESNLPsolver_dyn_obsts_dynamics_0_sparsity_out(1) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_dyn_obsts_sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FrankaFORCESNLPsolver_dyn_obsts_inequalities_0(in, out, NULL, w, 0);
		if( h )
		{
			nrow = FrankaFORCESNLPsolver_dyn_obsts_inequalities_0_sparsity_out(0)[0];
			ncol = FrankaFORCESNLPsolver_dyn_obsts_inequalities_0_sparsity_out(0)[1];
			colind = FrankaFORCESNLPsolver_dyn_obsts_inequalities_0_sparsity_out(0) + 2;
			row = FrankaFORCESNLPsolver_dyn_obsts_inequalities_0_sparsity_out(0) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_dyn_obsts_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = FrankaFORCESNLPsolver_dyn_obsts_inequalities_0_sparsity_out(1)[0];
			ncol = FrankaFORCESNLPsolver_dyn_obsts_inequalities_0_sparsity_out(1)[1];
			colind = FrankaFORCESNLPsolver_dyn_obsts_inequalities_0_sparsity_out(1) + 2;
			row = FrankaFORCESNLPsolver_dyn_obsts_inequalities_0_sparsity_out(1) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_dyn_obsts_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
	if ((9 == stage))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		FrankaFORCESNLPsolver_dyn_obsts_objective_1(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = FrankaFORCESNLPsolver_dyn_obsts_objective_1_sparsity_out(1)[0];
			ncol = FrankaFORCESNLPsolver_dyn_obsts_objective_1_sparsity_out(1)[1];
			colind = FrankaFORCESNLPsolver_dyn_obsts_objective_1_sparsity_out(1) + 2;
			row = FrankaFORCESNLPsolver_dyn_obsts_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_dyn_obsts_sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		FrankaFORCESNLPsolver_dyn_obsts_inequalities_1(in, out, NULL, w, 0);
		if( h )
		{
			nrow = FrankaFORCESNLPsolver_dyn_obsts_inequalities_1_sparsity_out(0)[0];
			ncol = FrankaFORCESNLPsolver_dyn_obsts_inequalities_1_sparsity_out(0)[1];
			colind = FrankaFORCESNLPsolver_dyn_obsts_inequalities_1_sparsity_out(0) + 2;
			row = FrankaFORCESNLPsolver_dyn_obsts_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_dyn_obsts_sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = FrankaFORCESNLPsolver_dyn_obsts_inequalities_1_sparsity_out(1)[0];
			ncol = FrankaFORCESNLPsolver_dyn_obsts_inequalities_1_sparsity_out(1)[1];
			colind = FrankaFORCESNLPsolver_dyn_obsts_inequalities_1_sparsity_out(1) + 2;
			row = FrankaFORCESNLPsolver_dyn_obsts_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			FrankaFORCESNLPsolver_dyn_obsts_sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((FrankaFORCESNLPsolver_dyn_obsts_float) this_f);
    }

    return 0;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
