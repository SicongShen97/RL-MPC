import numpy
import ctypes

name = "FrankaFORCESNLPsolver_5mm"
requires_callback = True
lib = "lib/libFrankaFORCESNLPsolver_5mm.so"
lib_static = "lib/libFrankaFORCESNLPsolver_5mm.a"
c_header = "include/FrankaFORCESNLPsolver_5mm.h"
nstages = 10

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, ( 40,   1),   40),
 ("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, ( 80,   1),   80)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  4,),    4),
 ("x02"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  4,),    4),
 ("x03"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  4,),    4),
 ("x04"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  4,),    4),
 ("x05"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  4,),    4),
 ("x06"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  4,),    4),
 ("x07"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  4,),    4),
 ("x08"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  4,),    4),
 ("x09"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  4,),    4),
 ("x10"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  4,),    4)]

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
	(4, 2, 1, 8, 2, 2, 1, 0), 
	(4, 2, 1, 8, 3, 3, 1, 0), 
	(4, 2, 1, 8, 3, 3, 1, 0), 
	(4, 2, 1, 8, 3, 3, 1, 0), 
	(4, 2, 1, 8, 3, 3, 1, 0), 
	(4, 2, 1, 8, 3, 3, 1, 0), 
	(4, 2, 1, 8, 3, 3, 1, 0), 
	(4, 2, 1, 8, 3, 3, 1, 0), 
	(4, 2, 1, 8, 3, 3, 1, 0), 
	(4, 0, 1, 8, 3, 3, 1, 0)
]