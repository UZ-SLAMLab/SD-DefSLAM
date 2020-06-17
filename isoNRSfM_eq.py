from sympy import *

init_printing(use_unicode=True)
# Three images minimum required to solve the normals of the system
# without isometry

#For original image
m1, n1, km1, kn1, B1 = symbols('m1 n1 km1 kn1 B1',real=True)
#For second image
m2, n2, km2, kn2, B2 = symbols('m2 n2 km2 kn2 B2',real=True)
#For third image
m3, n3, km3, kn3, B3 = symbols('m3 n3 km3 kn3 B3',real=True)

# Metric tensor estimation without depth
def metric_tensor_symbolic(m,n,km,kn):
    """Metric tensor estimation without depth"""
    g11 = (km*m-1)**2+(km*n)**2+km**2 # Abreviated as E
    g12 = km*kn*(m**2+n**2+1)-kn*m-km*n # Abreviated as F
    g22 = (kn*m)**2+(kn*n-1)**2+kn**2 # Abreviated as G
    G =  Matrix([[g11,g12],[g12,g22]])
    return g11,g12,g22,G


e1, e2, t1, t2 = symbols('e1 e2 t1 t2',real=True)


# Metric tensor estimation without depth
def metric_tensor_symbolic_symplify(m,n,km,kn,e):
    """Metric tensor estimation without depth"""
    g11 = km**2*e-2*km*m+1 # Abreviated as E
    g12 = km*kn*e-kn*m-km*n # Abreviated as F
    g22 = kn**2*e-2*kn*n+1 # Abreviated as G
    G =  Matrix([[g11,g12],[g12,g22]])
    return g11,g12,g22,G

g111,g121,g221,G1 = metric_tensor_symbolic(m1,n1,km1,kn1)
g112,g122,g222,G2 = metric_tensor_symbolic(m2,n2,km2,kn2)
g111s,g121s,g221s,G1s = metric_tensor_symbolic_symplify(m1,n1,km1,kn1,e1)
g112s,g122s,g222s,G2s = metric_tensor_symbolic_symplify(m2,n2,km2,kn2,e2)

# Cristophel symbols first kind
def derivate_metric_tensor(m,n,km,kn,g11,g12,g22):
    # Derivatives without taking into account depth
    Em_par = 2*(km*m-1)*(-km**2*m+km)-2*n*km**3*n-2*km**3
    Fm_par = -2*km**2*kn*(m**2+n**2+1)+km*kn*3*m-kn+km**2*n
    Gm_par =  2*kn*m*(-kn*km*m+kn)+2*(kn*n-1)*(-kn*km*n)-kn**2*km*2
    En_par =  2*(km*m-1)*(-kn*km*m)+2*km*n*(-km*kn*n+km)-2*kn*km**2
    Fn_par =  -2*km*kn**2*(m**2+n**2+1)+km*kn*3*n-km+kn**2*m
    Gn_par =  -2*kn**3*m**2+2*(kn*n-1)*(-kn**2*n+kn)-2*kn**3
    # Derivatives taking into account depth but without including it. The depth 
    # vanishes in the Christophel symbols
    Em_noB = Em_par - 2*g11*km
    Fm_noB = Fm_par - 2*g12*km
    Gm_noB = Gm_par - 2*g22*km
    En_noB = En_par - 2*g11*kn
    Fn_noB = Fn_par - 2*g12*kn
    Gn_noB = Gn_par - 2*g22*kn
    return Em_noB,Fm_noB,Gm_noB,En_noB,Fn_noB,Gn_noB

Em_noB1,Fm_noB1,Gm_noB1,En_noB1,Fn_noB1,Gn_noB1 = derivate_metric_tensor(m1,n1,km1,kn1,g111,g121,g221)
Em_noB2,Fm_noB2,Gm_noB2,En_noB2,Fn_noB2,Gn_noB2 = derivate_metric_tensor(m2,n2,km2,kn2,g112,g122,g222)

# Metric tensor inverse
ginv1 = 1/(g111*g221-g121**2)*Matrix([[g221,-g121],[-g121,g111]])
ginv2 = 1/(g112*g222-g122**2)*Matrix([[g222,-g122],[-g122,g112]])

# Cristophel symbols second kind
def christophel_symbols_second_kind(ginv,Em_noB,Fm_noB,Gm_noB,En_noB,Fn_noB,Gn_noB):
    Gamma001 =  ginv[0,0]*(1/2*Em_noB)+ginv[0,1]*(Fm_noB-1/2*En_noB)
    Gamma002 =  ginv[1,0]*(1/2*Em_noB)+ginv[1,1]*(Fm_noB-1/2*En_noB)
    Gamma011 =  ginv[0,0]*(1/2*En_noB)+ginv[0,1]*(1/2*Gm_noB)
    Gamma012 =  ginv[1,0]*(1/2*En_noB)+ginv[1,1]*(1/2*Gm_noB)
    Gamma101 =  Gamma011
    Gamma102 =  Gamma012
    Gamma111 =  ginv[0,0]*(Fn_noB-1/2*Gm_noB)+ginv[0,1]*(1/2*Gn_noB)
    Gamma112 =  ginv[1,0]*(Fn_noB-1/2*Gm_noB)+ginv[1,1]*(1/2*Gn_noB)
    Gamma1 = simplify(Matrix([[Gamma001,Gamma101],[Gamma011,Gamma111]]))
    Gamma2 = simplify(Matrix([[Gamma002,Gamma102],[Gamma012,Gamma112]]))
    return Gamma1,Gamma2

Tau11,Tau21 = christophel_symbols_second_kind(ginv1,Em_noB1,Fm_noB1,Gm_noB1,En_noB1,Fn_noB1,Gn_noB1)
Tau12,Tau22 = christophel_symbols_second_kind(ginv2,Em_noB2,Fm_noB2,Gm_noB2,En_noB2,Fn_noB2,Gn_noB2)


# Connect Warp Connections
j11, j12,j21, j22, H111, H121, H211,H221, H112, H122,H212, H222  = symbols('j11 j12 j21 j22 H111 H121 H211 H221 H112 H122 H122 H222',real=True)
J12 = Matrix([[j11,j12],[j21,j22]])
H0 = Matrix([[H111,H121],[H121,H221]])
H1 = Matrix([[H112,H122],[H122,H222]])
J12inv = J12**(-1)

# G2 in surface 1
G2Js = J12.T*G2s*J12

# 6 inc /2 eq -- Christophel symbols give you 2
EE1 = simplify(expand(g111s/g121s)-expand(G2Js[0,0]/G2Js[0,1]))
EE2 = simplify(expand(g221s/g121s)-expand(G2Js[1,1]/G2Js[0,1]))

# 2 equations /4 incognites (From change the variables)
E1_gamma = (J12inv[0,0]*(J12.T*Tau12*J12+H0)+J12inv[0,1]*(J12.T*Tau22*J12+H1))
E2_gamma = (J12inv[1,0]*(J12.T*Tau12*J12+H0)+J12inv[1,1]*(J12.T*Tau22*J12+H1))

def Equations_with_CS(E1_gamma,E2_gamma,km1,kn1,km2,kn2,H111,H121,H211,H221,H112,H212,H122,H222,j11,j12,j21,j22,t1,t2):
    EE3 = expand(simplify(-E1_gamma[0,0]/2-km1)*det(J12))
    EE4 = expand(simplify(-E2_gamma[1,1]/2-kn1)*det(J12))
    EE5 = expand(simplify(-E1_gamma[1,0]-kn1)*det(J12))
    EE6 = expand(simplify(-E2_gamma[1,0]-km1)*det(J12))
    #Gamma001
    b1 = H111*j22/2-H112*j12/2
    b1 = t2
    a11 = EE3.coeff(km1)
    a12 = EE3.coeff(kn1)
    a13 = EE3.coeff(km2)
    a14 = EE3.coeff(kn2)
    #Gamma112
    b2 = -H221*j21/2 + H222*j11/2
    b2 = t1
    a21 =  EE4.coeff(km1)
    a22 =  EE4.coeff(kn1)
    a23 =  EE4.coeff(km2)
    a24 =  EE4.coeff(kn2)
    #Gamma101
    b3 = H122*j22 - H112*j12
    a31 = EE5.coeff(km1)
    a32 = EE5.coeff(kn1)
    a33 = EE5.coeff(km2)
    a34 = EE5.coeff(kn2)
    #Gamma102
    b4 = -H121*j21 + H122*j11 
    a41 = EE6.coeff(km1)
    a42 = EE6.coeff(kn1)
    a43 = EE6.coeff(km2)
    a44 = EE6.coeff(kn2)
    expand(-b4+a41*km1+a42*kn1+a43*km2+a44*kn2)
    # Assemble matrices
    A = Matrix([[a11,a12,a13,a14],[a21,a22,a23,a24],[a31,a32,a33,a34],[a41,a42,a43,a44]])
    b = Matrix([[b1],[b2],[b3],[b4]])
    return A,b
   

A,b = Equations_with_CS(E1_gamma,E2_gamma,km1,kn1,km2,kn2,H111,H121,H211,H221,H112,H122,H212,H222,j11,j12,j21,j22,t1, t2)
# det(A) underconstrain system
print(A.rank())

# Schwarzian equations
simplify(E1_gamma[1,1])
simplify(E2_gamma[0,0])

baux = Matrix([[(b[0,0])],[b[1,0]]])
Aaux = Matrix([(A[0,0:4]),(A[1,0:4])])

#Unitary norm
a = list(linsolve([Aaux,baux],[km1,kn1,km2,kn2]))

# kn2 and km2 depending on kn1 and km1
eqkn2 = (a[0][1]-kn1)
kn2aux = simplify(solve(eqkn2,kn2)[0])
eqkm2 = a[0][0].subs(kn2,kn2aux)-km1
km2aux = simplify(solve(eqkm2,km2)[0])
kn2aux = simplify(kn2aux.subs(km2,km2aux))

kn2aux = simplify(expand((kn2aux)))
km2aux = simplify(expand((km2aux)))

EE1 = EE1.subs(kn2,kn2aux)
E1 = EE1.subs(km2,km2aux) 
EE2 = EE2.subs(kn2,kn2aux)
E2 = EE2.subs(km2,km2aux)

#Final equations
E1 = factor(E1)
E2 = factor(E2)
num1,den1 =fraction(E1)
num2,den2 =fraction(E2)
num1 = num1*2
num2 = num2*2
# Final equations
eq1 = Poly(num1,km1,kn1)
eq2 = Poly(num2,km1,kn1)

# Coeffs
eq1_3_0 = simplify(eq1.coeff_monomial(km1**3*kn1**0)) 
eq1_1_2 = simplify(eq1.coeff_monomial(km1**1*kn1**2)) # = 0
eq1_0_2 = simplify(eq1.coeff_monomial(km1**0*kn1**2)) # = 0
eq1_0_3 = simplify(eq1.coeff_monomial(km1**0*kn1**3)) # = 0
eq1_2_1 = simplify(eq1.coeff_monomial(km1**2*kn1**1)) 
eq1_2_0 = simplify(eq1.coeff_monomial(km1**2*kn1**0)) 
eq1_1_1 = simplify(eq1.coeff_monomial(km1**1*kn1**1))
eq1_1_0 = simplify(eq1.coeff_monomial(km1**1*kn1**0))
eq1_0_1 = simplify(eq1.coeff_monomial(km1**0*kn1**1))
eq1_0_0 = simplify(eq1.coeff_monomial(km1**0*kn1**0))

eq2_3_0 = simplify(eq2.coeff_monomial(km1**3*kn1**0)) # = 0
eq2_0_3 = simplify(eq2.coeff_monomial(km1**0*kn1**3))
eq2_1_2 = simplify(eq2.coeff_monomial(km1**1*kn1**2))
eq2_0_2 = simplify(eq2.coeff_monomial(km1**0*kn1**2))
eq2_2_1 = simplify(eq2.coeff_monomial(km1**2*kn1**1)) # = 0
eq2_2_0 = simplify(eq2.coeff_monomial(km1**2*kn1**0)) # = 0
eq2_1_1 = simplify(eq2.coeff_monomial(km1**1*kn1**1))
eq2_1_0 = simplify(eq2.coeff_monomial(km1**1*kn1**0))
eq2_0_1 = simplify(eq2.coeff_monomial(km1**0*kn1**1))
eq2_0_0 = simplify(eq2.coeff_monomial(km1**0*kn1**0))

def subslong(coeff,j11s,j12s,j21s,j22s,h111s,h121s,h211s,h221s,h112s,h122s,h212s,h222s,x1,y1,x2,y2):
    eq = coeff.subs(j11,j11s)
    eq = eq.subs(j12,j12s)
    eq = eq.subs(j21,j21s)
    eq = eq.subs(j22,j22s)
    eq = eq.subs(h111,h111s)
    eq = eq.subs(h121,h121s)
    eq = eq.subs(h211,h211s)
    eq = eq.subs(h221,h221s)
    eq = eq.subs(h112,h112s)
    eq = eq.subs(h122,h122s)
    eq = eq.subs(h212,h212s)
    eq = eq.subs(h222,h222s)
    eq = eq.subs(m1,x1)
    eq = eq.subs(n1,y1)
    eq = eq.subs(m2,x2)
    eq = eq.subs(n2,y2) 
    return eq

def subssymbolic(coeff,j11s,j12s,j21s,j22s,t1s,t2s,e1s,e2s,x1s,y1s,x2s,y2s):
    eq = coeff.subs(j11,j11s)
    eq = eq.subs(j12,j12s)
    eq = eq.subs(j21,j21s)
    eq = eq.subs(j22,j22s)
    eq = eq.subs(t1,t1s)
    eq = eq.subs(t2,t2s)
    eq = eq.subs(e1,e1s)
    eq = eq.subs(e2,e2s)
    eq = eq.subs(m1,x1s)
    eq = eq.subs(n1,y1s)
    eq = eq.subs(m2,x2s)
    eq = eq.subs(n2,y2s) 
    return eq


def subssymGamma(coeff,j11s,j12s,j21s,j22s,t1s,t2s,e1s,e2s,x1s,y1s,x2s,y2s,km2,kn2):
    eq = coeff.subs(j11,j11s)
    eq = eq.subs(j12,j12s)
    eq = eq.subs(j21,j21s)
    eq = eq.subs(j22,j22s)
    eq = eq.subs(t1,t1s)
    eq = eq.subs(t2,t2s)
    eq = eq.subs(e1,e1s)
    eq = eq.subs(e2,e2s)
    eq = eq.subs(m1,x1s)
    eq = eq.subs(n1,y1s)
    eq = eq.subs(m2,x2s)
    eq = eq.subs(n2,y2s) 
    return eq

av = 1.35
bv = 2.45
cv = 3.15
dv = -2.45
t1v = 1.85
t2v = -4.85
e1v = 1.25
e2v = 2.25
x1v = 1.25
y1v = 3.15
x2v =  -4.225
y2v = -5.55

array = [eq1_3_0,eq1_2_1,eq1_1_2,eq1_0_3,eq1_2_0,eq1_1_1,eq1_0_2,eq1_1_0,eq1_0_1,eq1_0_0]
array2 = [eq2_3_0,eq2_2_1,eq2_1_2,eq2_0_3,eq2_2_0,eq2_1_1,eq2_0_2,eq2_1_0,eq2_0_1,eq2_0_0]
for coef in array:
    print(subssymbolic(coef,av,cv,bv,dv,t1v,t2v,e1v,e2v,x1v,y1v,x2v,y2v))

for coef in array2:
    print(subssymbolic(coef,av,cv,bv,dv,t1v,t2v,e1v,e2v,x1v,y1v,x2v,y2v))


