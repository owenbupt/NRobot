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
# use jacobian matrix method

from sympy import *
from sympy.printing import print_ccode
from time import time
init_printing(use_unicode=False,wrap_line=False)
begin = time()

x, y, xi, yi, xj, yj, ri, rj, Ri, Rj, t = \
symbols("x, y, xi, yi, xj, yj, ri, rj, Ri, Rj, t", real=True)

# Hyperbola parameters
qi = Matrix([xi, yi])
qj = Matrix([xj, yj])
c = sqrt( (xi-xj)**2 + (yi-yj)**2 )
ai = (ri + rj + Rj - Ri) / 2;
bi = sqrt(c**2 - ai**2);
aj = (ri + rj + Ri - Rj) / 2;
bj = sqrt(c**2 - aj**2);

# Hyperbolic branches
Hij = Matrix([ai*cosh(t), bi*sinh(t)])
Hji = Matrix([aj*cosh(t), bj*sinh(t)])

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

# Jacobian matrices
Ji = zeros(2,2)
Ji[0,0] = diff(Hij[0], xi) # dx/dxi
Ji[0,1] = diff(Hij[0], yi) # dx/dyi
Ji[1,0] = diff(Hij[1], xi) # dy/dxi
Ji[1,1] = diff(Hij[1], yi) # dy/dyi
Jj = zeros(2,2)
Jj[0,0] = diff(Hji[0], xi) # dx/dxi
Jj[0,1] = diff(Hji[0], yi) # dx/dyi
Jj[1,0] = diff(Hji[1], xi) # dy/dxi
Jj[1,1] = diff(Hji[1], yi) # dy/dyi

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
Jni = Ji*ni
Jnj = Jj*nj

# Create c functions ###########################################################
# Jni
f = open('Jn.c','w')
f.write( "/* This file was created by SymPy 1.0 */\n" )

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

# Export expressions in txt ####################################################
# Jni
f = open('Jni.txt','w')
f.write( pretty(Jni[0]) )
f.write( "\n\n" )
f.write( pretty(Jni[1]) )
f.close()
# Jnj
f = open('Jnj.txt','w')
f.write( pretty(Jnj[0]) )
f.write( "\n\n" )
f.write( pretty(Jnj[1]) )
f.close()

end = time()
print("Elapsed time "+"{0:.2f}".format(end-begin)+" seconds")
