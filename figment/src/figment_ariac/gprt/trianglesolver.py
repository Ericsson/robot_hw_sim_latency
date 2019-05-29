"""
Written by Steven Byrnes, http://sjbyrnes.com/

Download: https://pypi.python.org/pypi/trianglesolver/
Source code repository: https://github.com/sbyrnes321/trianglesolver

This little package applies the law of sines or cosines to find all the
sides and angles of a triangle, if you know some of the sides and/or
angles.

The main function defined by this package is solve(...). Simple example::

    from math import pi
    from trianglesolver import solve
    a,b,c,A,B,C = solve(b=7.6, c=8.3, A=pi/3)

Following the usual convention, lower-case letters are side lengths and
capital letters are angles. Corresponding letters are opposite each other,
e.g. side b is opposite angle B.

All angles are in radians! However, you can use the degree constant to
convert::

    from trianglesolver import solve, degree
    a,b,c,A,B,C = solve(b=7, A=5*degree, B=70*degree)
    print(C / degree)
"""

from __future__ import division, print_function
from math import sin, cos, pi, sqrt, acos, asin

import sys
EPSILON = sys.float_info.epsilon # typical floating-point calculation error


# 5 * degree is 5 degrees in radians
# X / degree is the angle X expressed in degrees
degree = pi/180

# ---------- Helper functions -------------------------------------- #

def aaas(D, E, F, f):
    """ This function solves the triangle and returns (d,e,f,D,E,F) """
    d = f * sin(D) / sin(F)
    e = f * sin(E) / sin(F)
    return (d,e,f,D,E,F)

def sss(d,e,f):
    """ This function solves the triangle and returns (d,e,f,D,E,F) """
    assert d + e > f and e + f > d and f + d > e
    F = acos((d**2 + e**2 - f**2) / (2 * d * e))
    E = acos((d**2 + f**2 - e**2) / (2 * d * f))
    D = pi - F - E
    return (d,e,f,D,E,F)

def sas(d,e,F):
    """ This function solves the triangle and returns (d,e,f,D,E,F) """
    f = sqrt(d**2 + e**2 - 2 * d * e * cos(F))
    return sss(d,e,f)


def ssa(d, e, D, ssa_flag):
    """ This function solves the triangle and returns (d,e,f,D,E,F) """
    sinE = sin(D) * e / d
    if abs(sinE - 1) < 100 * EPSILON:
        # Right triangle, where the solution is unique
        E = pi/2
    elif sinE > 1:
        raise ValueError('No such triangle')
    elif ssa_flag == 'forbid':
        raise ValueError('Two different triangles fit this description')
    else:
        E = asin(sinE)
        if ssa_flag == 'obtuse':
            E = pi - E
    F = pi - D - E
    e,f,d,E,F,D = aaas(E,F,D,d)
    return (d,e,f,D,E,F)


# ---------- Main function you should use ------------------------- #
    
def solve(a=None, b=None, c=None, A=None, B=None, C=None, ssa_flag='forbid'):
    """
    Solve to find all the information about a triangle, given partial
    information.
    
    a, b, c, A, B, and C are the three sides and angles. (e.g. A is the angle
    opposite the side of length a.) Out of these six possibilities, you need 
    to tell the program exactly three. Then the program will tell you all six.
    
    It returns a tuple (a, b, c, A, B, C).
    
    "ssa" is the situation when you give two sides and an angle which is not
    between them. This is usually not enough information to specify a unique
    triangle. (Except in one special case involving right triangles.) Instead
    there are usually two possibilities.
    
    Therefore there is an 'ssa_flag'. You can set it to'forbid' (raise an error
    if the answer is not unique - the default setting), or 'acute' (where the
    unknown angle across from the known side is chosen to be acute) or 'obtuse'
    (similarly).
    """
    if a:
        a=abs(a)
    if b:
        b=abs(b)
    if c:
        c=abs(c)
    if A:
        A=abs(A)
    if B:
        B=abs(B)
    if C:
        C=abs(C)
    if sum(x is not None for x in (a,b,c,A,B,C)) != 3:
        raise ValueError('Must provide exactly 3 inputs')
    if sum(x is None for x in (a,b,c)) == 3:
        raise ValueError('Must provide at least 1 side length')
    assert all(x > 0 for x in (a,b,c,A,B,C) if x is not None)
    assert all(x < pi for x in (A,B,C) if x is not None)
    assert ssa_flag in ('forbid', 'acute', 'obtuse')
    
    # If three sides are known...
    if sum(x is not None for x in (a,b,c)) == 3:
        a,b,c,A,B,C = sss(a,b,c)
        return (a,b,c,A,B,C)

    # If two sides and one angle are known...
    if sum(x is not None for x in (a,b,c)) == 2:
        # ssa case
        if all(x is not None for x in (a, A, b)):
            a,b,c,A,B,C = ssa(a, b, A, ssa_flag)
        elif all(x is not None for x in (a, A, c)):
            a,c,b,A,C,B = ssa(a, c, A, ssa_flag)
        elif all(x is not None for x in (b, B, a)):
            b,a,c,B,A,C = ssa(b, a, B, ssa_flag)
        elif all(x is not None for x in (b, B, c)):
            b,c,a,B,C,A = ssa(b, c, B, ssa_flag)
        elif all(x is not None for x in (c, C, a)):
            c,a,b,C,A,B = ssa(c, a, C, ssa_flag)
        elif all(x is not None for x in (c, C, b)):
            c,b,a,C,B,A = ssa(c, b, C, ssa_flag)
        
        # sas case
        elif all(x is not None for x in (a, b, C)):
            a,b,c,A,B,C = sas(a, b, C)
        elif all(x is not None for x in (b, c, A)):
            b,c,a,B,C,A = sas(b, c, A)
        elif all(x is not None for x in (c, a, B)):
            c,a,b,C,A,B = sas(c, a, B)
        else:
            raise ValueError('Oops, this code should never run')
        return (a,b,c,A,B,C)
    
    # If one side and two angles are known...
    if sum(x is not None for x in (a,b,c)) == 1:
        # Find the third angle...
        if A is None:
            A = pi - B - C
        elif B is None:
            B = pi - A - C
        else:
            C = pi - A - B
        assert A > 0 and B > 0 and C > 0
        # Then solve the triangle...
        if c is not None:
            a,b,c,A,B,C = aaas(A,B,C,c)
        elif a is not None:
            b,c,a,B,C,A = aaas(B,C,A,a)
        else:
            c,a,b,C,A,B = aaas(C,A,B,b)
        return (a,b,c,A,B,C)
    raise ValueError('Oops, this code should never run')



#---------------- Tests ------------------------------------------------- #

def floats_are_equal(x,y):
    return abs(x - y) <= 100 * EPSILON * (abs(x) + abs(y))

def float_tuples_are_equal(x,y):
    return all(floats_are_equal(x[i], y[i]) for i in range(len(x)))

def test_triangle(a,b,c,A,B,C):
    """ Check that the triangle satisfies the law of cosines and law of
    sines"""
    assert floats_are_equal(a/sin(A), b/sin(B))
    assert floats_are_equal(a/sin(A), c/sin(C))
    assert floats_are_equal(c**2, a**2 + b**2 - 2 * a * b * cos(C))
    assert floats_are_equal(a**2, b**2 + c**2 - 2 * b * c * cos(A))
    assert floats_are_equal(b**2, c**2 + a**2 - 2 * c * a * cos(B))

def test_solver(a,b,c):
    """ Test that the program works, for a triangle of sides a,b,c."""
    # first find the angles, testing SSS
    a1,b1,c1,A,B,C = solve(a=a,b=b,c=c)
    assert float_tuples_are_equal((a,b,c), (a1,b1,c1))
    test_triangle(a,b,c,A,B,C)
    tri = (a,b,c,A,B,C)
    
    # SAS tests
    assert float_tuples_are_equal(tri, solve(a=a,b=b,C=C))
    assert float_tuples_are_equal(tri, solve(a=a,c=c,B=B))
    assert float_tuples_are_equal(tri, solve(b=b,c=c,A=A))
    
    # SAA / ASA tests
    assert float_tuples_are_equal(tri, solve(a=a, A=A, B=B))
    assert float_tuples_are_equal(tri, solve(a=a, A=A, C=C))
    assert float_tuples_are_equal(tri, solve(a=a, B=B, C=C))
    assert float_tuples_are_equal(tri, solve(b=b, A=A, B=B))
    assert float_tuples_are_equal(tri, solve(b=b, A=A, C=C))
    assert float_tuples_are_equal(tri, solve(b=b, B=B, C=C))
    assert float_tuples_are_equal(tri, solve(c=c, A=A, B=B))
    assert float_tuples_are_equal(tri, solve(c=c, A=A, C=C))
    assert float_tuples_are_equal(tri, solve(c=c, B=B, C=C))

    Atype = 'acute' if A < pi/2 else 'obtuse'
    Btype = 'acute' if B < pi/2 else 'obtuse'
    Ctype = 'acute' if C < pi/2 else 'obtuse'
    
    # SSA tests
    assert float_tuples_are_equal(tri, solve(a=a, b=b, A=A, ssa_flag=Btype))
    assert float_tuples_are_equal(tri, solve(a=a, b=b, B=B, ssa_flag=Atype))
    assert float_tuples_are_equal(tri, solve(a=a, c=c, A=A, ssa_flag=Ctype))
    assert float_tuples_are_equal(tri, solve(a=a, c=c, C=C, ssa_flag=Atype))
    assert float_tuples_are_equal(tri, solve(b=b, c=c, B=B, ssa_flag=Ctype))
    assert float_tuples_are_equal(tri, solve(b=b, c=c, C=C, ssa_flag=Btype))
    
def run_lots_of_tests():
    for a,b,c in ((7,8,9), (5,6,10), (5,10,6), (10,5,6), (5,12,13),
                  (12,5,13), (12,13,5)):
        test_solver(a,b,c)
    print('All tests pass!')
