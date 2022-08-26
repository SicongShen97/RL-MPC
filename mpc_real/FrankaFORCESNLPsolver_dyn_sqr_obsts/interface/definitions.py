import numpy
import ctypes

name = "FrankaFORCESNLPsolver_dyn_sqr_obsts"
requires_callback = True
lib = "lib/libFrankaFORCESNLPsolver_dyn_sqr_obsts.so"
lib_static = "lib/libFrankaFORCESNLPsolver_dyn_sqr_obsts.a"
c_header = "include/FrankaFORCESNLPsolver_dyn_sqr_obsts.h"
nstages = 10

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, ( 50,   1),   50),
 ("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (120,   1),  120)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x02"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x03"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x04"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x05"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x06"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x07"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x08"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x09"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5),
 ("x10"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  5,),    5)]

# Info Struct Fields
info = \
[('it', ctypes.c_int),
 ('it2opt', ctypes.c_int),
 ('res_eq', ctypes.c_double),
 ('res_ineq', ctypes.c_double),
 ('rsnorm', ctypes.c_double),
 ('rcompnorm', ctypes.c_double),
 ('pobj', ctypes.c_double),
 ('dobj', ctypes.c_double),
 ('dgap', ctypes.c_double),
 ('rdgap', ctypes.c_double),
 ('mu', ctypes.c_double),
 ('mu_aff', ctypes.c_double),
 ('sigma', ctypes.c_double),
 ('lsit_aff', ctypes.c_int),
 ('lsit_cc', ctypes.c_int),
 ('step_aff', ctypes.c_double),
 ('step_cc', ctypes.c_double),
 ('solvetime', ctypes.c_double),
 ('fevalstime', ctypes.c_double),
 ('solver_id', ctypes.c_int * 8)
]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(5, 2, 2, 12, 3, 2, 2, 0), 
	(5, 2, 2, 12, 4, 3, 2, 0), 
	(5, 2, 2, 12, 4, 3, 2, 0), 
	(5, 2, 2, 12, 4, 3, 2, 0), 
	(5, 2, 2, 12, 4, 3, 2, 0), 
	(5, 2, 2, 12, 4, 3, 2, 0), 
	(5, 2, 2, 12, 4, 3, 2, 0), 
	(5, 2, 2, 12, 4, 3, 2, 0), 
	(5, 2, 2, 12, 4, 3, 2, 0), 
	(5, 0, 2, 12, 4, 3, 2, 0)
]