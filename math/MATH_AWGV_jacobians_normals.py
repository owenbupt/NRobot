# Copyright (C) 2017 Sotiris Papatheodorou
#
# This file is part of NRobot.
#
# NRobot is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# NRobot is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with NRobot.  If not, see <http://www.gnu.org/licenses/>.

# Using SymPy 1.0
# http://docs.sympy.org/latest/index.html

# TODO
# find how not to print xi as greek ksi in unicode pprint
# add std:: before math functions

from sympy import *
from sympy.printing import print_ccode
from time import time
from shutil import copyfile
import datetime
init_printing(use_unicode=False)
begin = time()

# Select export formats
EXPORT_CPP = True
EXPORT_TXT = False

# Select symbolic computations
COMPUTE_JACOBIANS = True
COMPUTE_NORMALS = True
COMPUTE_PRODUCT = True

################################################################################
# Symbolic computation #########################################################
################################################################################
x, y, xi, yi, xj, yj, ri, rj, Ri, Rj, t = \
symbols("x, y, xi, yi, xj, yj, ri, rj, Ri, Rj, t", real=True)

# Hyperbola parameters
qi = Matrix([xi, yi])
qj = Matrix([xj, yj])
c = sqrt( (xi-xj)**2 + (yi-yj)**2 ) / 2
ai = (ri + rj + Rj - Ri) / 2;
bi = sqrt(c**2 - ai**2);
aj = (ri + rj + Ri - Rj) / 2;
bj = sqrt(c**2 - aj**2);

# Hyperbolic branches
Hij = Matrix([ai*cosh(t), bi*sinh(t)])
Hji = Matrix([aj*cosh(t), bj*sinh(t)]) # From the hyperbolic branch definition, this doesn't seem to need a minus

# Functions for cos(atan(y,x)) and sin(atan(y,x))
class cos_atan(Function):
    @classmethod
    def eval(cls, y, x):
        return x/sqrt(x**2+y**2)

class sin_atan(Function):
    @classmethod
    def eval(cls, y, x):
        return y/sqrt(x**2+y**2)

# Rotation matrices
Rij = Matrix([[cos_atan( yi-yj, xi-xj ), -sin_atan( yi-yj, xi-xj )], \
[sin_atan( yi-yj, xi-xj ), cos_atan( yi-yj, xi-xj )]])
Rji = Matrix([[cos_atan( yj-yi, xj-xi ), -sin_atan( yj-yi, xj-xi )], \
[sin_atan( yj-yi, xj-xi ), cos_atan( yj-yi, xj-xi )]])

# Rotate branches
Hij = Rij * Hij
Hji = Rji * Hji

# Translate branches
Hij = Hij + (qi+qj)/2
Hji = Hji + (qi+qj)/2

# Hyperbola branch Jacobian matrices
if COMPUTE_JACOBIANS:
    Ji = Hij.jacobian(qi)
    Jj = Hji.jacobian(qi)

# Hyperbola branch normal vector
if COMPUTE_NORMALS:
    # Hyperbola boundary derivatives
    dHij = diff(Hij, t)
    ddHij = diff(dHij, t)
    dHji = diff(Hji, t)
    ddHji = diff(dHji, t)
    # Normal vectors (direction towards the center of curvature)
    ni = ddHij - ddHij.dot( dHij/dHij.norm() ) * dHij/dHij.norm()
    nj = ddHji - ddHji.dot( dHji/dHji.norm() ) * dHji/dHji.norm()
    # Create unit normal vectors
    ni = ni / ni.norm()
    nj = nj / nj.norm()
    # Make normal vectors outwards pointing
    # Whether the cell is convex or not depends on the sign of a
    # If the cell is convex, the normal vector direction must be reversed
    ni = - sign(ai) * ni
    nj = - sign(aj) * nj

# Jacobian-normal products
if COMPUTE_PRODUCT and COMPUTE_NORMALS and COMPUTE_JACOBIANS:
    Jni = Ji*ni
    Jnj = Jj*nj




################################################################################
# Create C++ functions #########################################################
################################################################################
if EXPORT_CPP:
    now = datetime.datetime.now()

    if COMPUTE_JACOBIANS:
        # Create J source file
        f = open('J.cpp','w')
        f.write( "/* This file was created by SymPy 1.0 on "+str(now)+" */\n\n" )
        f.write( "#include <cmath>\n\n" )
        f.write( "#include \"J.hpp\"\n\n" )
        f.write( "using namespace std;\n\n" )

        f.write( "double nr_FJi_x_xi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(Ji[0,0]) )
        f.write( ";\n}\n\n" )

        f.write( "double nr_FJi_x_yi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(Ji[0,1]) )
        f.write( ";\n}\n\n" )

        f.write( "double nr_FJi_y_xi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(Ji[1,0]) )
        f.write( ";\n}\n\n" )

        f.write( "double nr_FJi_y_yi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(Ji[1,1]) )
        f.write( ";\n}\n" )

        f.write( "double nr_FJj_x_xi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(Jj[0,0]) )
        f.write( ";\n}\n\n" )

        f.write( "double nr_FJj_x_yi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(Jj[0,1]) )
        f.write( ";\n}\n\n" )

        f.write( "double nr_FJj_y_xi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(Jj[1,0]) )
        f.write( ";\n}\n\n" )

        f.write( "double nr_FJj_y_yi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(Jj[1,1]) )
        f.write( ";\n}\n" )

        f.close()

        # Create J header file
        f = open('J.hpp','w')
        f.write( "/* This file was created by SymPy 1.0 on "+str(now)+" */\n\n" )
        f.write( "#ifndef __J_hpp\n" )
        f.write( "#define __J_hpp\n\n" )
        f.write( "double nr_FJi_x_xi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "double nr_FJi_x_yi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "double nr_FJi_y_xi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "double nr_FJi_y_yi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "double nr_FJj_x_xi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "double nr_FJj_x_yi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "double nr_FJj_y_xi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "double nr_FJj_y_yi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "\n#endif\n" )
        f.close()

        # Copy files to source code directory
        copyfile( './J.cpp', '../src/NRobot/J.cpp' )
        copyfile( './J.hpp', '../src/NRobot/J.hpp' )

    if COMPUTE_NORMALS:
        # Create n source file
        f = open('n.cpp','w')
        f.write( "/* This file was created by SymPy 1.0 on "+str(now)+" */\n\n" )
        f.write( "#include <cmath>\n\n" )
        f.write( "#include \"n.hpp\"\n\n" )
        f.write( "using namespace std;\n\n" )

        f.write( "double nr_Fni_x( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(ni[0]) )
        f.write( ";\n}\n\n" )

        f.write( "double nr_Fni_y( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(ni[1]) )
        f.write( ";\n}\n\n" )

        f.write( "double nr_Fnj_x( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(nj[0]) )
        f.write( ";\n}\n\n" )

        f.write( "double nr_Fnj_y( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(nj[1]) )
        f.write( ";\n}\n" )

        f.close()

        # Create n header file
        f = open('n.hpp','w')
        f.write( "/* This file was created by SymPy 1.0 on "+str(now)+" */\n\n" )
        f.write( "#ifndef __n_hpp\n" )
        f.write( "#define __n_hpp\n\n" )
        f.write( "double nr_Fni_x( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "double nr_Fni_y( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "double nr_Fnj_x( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "double nr_Fnj_y( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "\n#endif\n" )
        f.close()

        # Copy files to source code directory
        copyfile( './n.cpp', '../src/NRobot/n.cpp' )
        copyfile( './n.hpp', '../src/NRobot/n.hpp' )

    if COMPUTE_PRODUCT:
        # Create Jn source file
        f = open('Jn.cpp','w')
        f.write( "/* This file was created by SymPy 1.0 on "+str(now)+" */\n\n" )
        f.write( "#include <cmath>\n\n" )
        f.write( "#include \"Jn.hpp\"\n\n" )
        f.write( "using namespace std;\n\n" )

        f.write( "double nr_FJni_x( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(Jni[0]) )
        f.write( ";\n}\n\n" )

        f.write( "double nr_FJni_y( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(Jni[1]) )
        f.write( ";\n}\n\n" )

        f.write( "double nr_FJnj_x( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(Jnj[0]) )
        f.write( ";\n}\n\n" )

        f.write( "double nr_FJnj_y( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {\n\treturn " )
        f.write( ccode(Jnj[1]) )
        f.write( ";\n}\n" )

        f.close()

        # Create Jn header file
        f = open('Jn.hpp','w')
        f.write( "/* This file was created by SymPy 1.0 on "+str(now)+" */\n\n" )
        f.write( "#ifndef __Jn_hpp\n" )
        f.write( "#define __Jn_hpp\n\n" )
        f.write( "double nr_FJni_x( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "double nr_FJni_y( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "double nr_FJnj_x( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "double nr_FJnj_y( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj );\n" )
        f.write( "\n#endif\n" )
        f.close()

        # Copy files to source code directory
        copyfile( './Jn.cpp', '../src/NRobot/Jn.cpp' )
        copyfile( './Jn.hpp', '../src/NRobot/Jn.hpp' )




################################################################################
# Export expressions in txt ####################################################
################################################################################
if COMPUTE_PRODUCT and EXPORT_TXT:
    # Jni
    f = open('Jni.txt','w')
    f.write( pretty(Jni[0],wrap_line=False) )
    f.write( "\n\n" )
    f.write( pretty(Jni[1],wrap_line=False) )
    f.close()
    # Jnj
    f = open('Jnj.txt','w')
    f.write( pretty(Jnj[0],wrap_line=False) )
    f.write( "\n\n" )
    f.write( pretty(Jnj[1],wrap_line=False) )
    f.close()

end = time()
print("Elapsed time "+"{0:.2f}".format(end-begin)+" seconds")
