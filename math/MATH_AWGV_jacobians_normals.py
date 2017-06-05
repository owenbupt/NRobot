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

# http://docs.sympy.org/latest/index.html
# http://docs.sympy.org/latest/modules/printing.html#module-sympy.printing.ccode

# TODO
# find how not to print xi as greek ksi in unicode pprint
# use jacobian matrix method

from sympy import *
from sympy.printing import print_ccode
from time import time
init_printing(use_unicode=True)
begin = time()

x, y, xi, yi, xj, yj, ri, rj, Ri, Rj, t = \
symbols("x, y, x_i, yi, xj, yj, ri, rj, Ri, Rj, t", real=True)

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

# Normal vectors
dHij = diff(Hij, t)
ddHij = diff(dHij, t)
dHji = diff(Hji, t)
ddHji = diff(dHji, t)
ni = ddHij - ddHij.dot( dHij/dHij.norm() ) * dHij/dHij.norm();
nj = ddHji - ddHji.dot( dHji/dHji.norm() ) * dHji/dHji.norm();
# Whether the cell is convex or not depends on the sign of a
ni = - sign(ai) * ni / ni.norm();
nj = - sign(aj) * nj / nj.norm();

# Jacobian-normal Products
Jni = Ji*ni
Jnj = Jj*nj

# print("Hij=")
# pprint(Hij)
# print("Jni=")
# pprint(Jni)

end = time()
print("Elapsed time "+str(end-begin))
