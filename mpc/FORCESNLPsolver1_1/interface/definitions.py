import numpy
import ctypes

name = "FORCESNLPsolver1_1"
requires_callback = True
lib = "lib/libFORCESNLPsolver1_1.so"
lib_static = "lib/libFORCESNLPsolver1_1.a"
c_header = "include/FORCESNLPsolver1_1.h"
nstages = 8

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,   1),    6),
 ("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, ( 80,   1),   80),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (144,   1),  144)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x1"                  , ""      , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x2"                  , ""      , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x3"                  , ""      , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x4"                  , ""      , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x5"                  , ""      , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x6"                  , ""      , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x7"                  , ""      , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10),
 ("x8"                  , ""      , ""               , ctypes.c_double, numpy.float64,     ( 10,),   10)]

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
('fevalstime', ctypes.c_double)
]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(10, 6, 3, 18, 9, 8, 3, 0), 
	(10, 6, 3, 18, 9, 8, 3, 0), 
	(10, 6, 3, 18, 9, 8, 3, 0), 
	(10, 6, 3, 18, 9, 8, 3, 0), 
	(10, 6, 3, 18, 9, 8, 3, 0), 
	(10, 6, 3, 18, 9, 8, 3, 0), 
	(10, 6, 3, 18, 9, 8, 3, 0), 
	(10, 6, 3, 18, 9, 8, 3, 0)
]