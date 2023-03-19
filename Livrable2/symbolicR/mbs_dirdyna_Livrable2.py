#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Sun Mar 19 18:42:46 2023
#
#	==> Project name: Livrable2
#
#	==> Number of joints: 10
#
#	==> Function: F1 - Recursive Direct Dynamics of tree-like MBS
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos

def dirdyna(M, c, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
    S6 = sin(q[6])
    C6 = cos(q[6])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S10 = sin(q[10])
    C10 = cos(q[10])
    C6p7 = C6*C7-S6*S7
    S6p7 = C6*S7+S6*C7
    C6p8 = C6*C8-S6*S8
    S6p8 = C6*S8+S6*C8
 
# Forward Kinematics

    OM25 = qd[4]*S5
    OM35 = qd[4]*C5
    OA25 = qd[4]*qd[5]*C5
    OA35 = -qd[4]*qd[5]*S5
    AF25 = -s.g[3]*S5
    AF35 = -s.g[3]*C5
    AM25_1 = -S4*C5
    AM35_1 = S4*S5
    AM25_2 = C4*C5
    AM35_2 = -C4*S5
    OM16 = qd[5]*C6-OM35*S6
    OM26 = qd[6]+OM25
    OM36 = qd[5]*S6+OM35*C6
    OA16 = -qd[6]*OM35*C6-S6*(OA35+qd[5]*qd[6])
    OA36 = -qd[6]*OM35*S6+C6*(OA35+qd[5]*qd[6])
    BS16 = -OM26*OM26-OM36*OM36
    BS26 = OM16*OM26
    BS36 = OM16*OM36
    BS66 = OM26*OM36
    BS96 = -OM16*OM16-OM26*OM26
    BEF36 = BS36+OA25
    BEF46 = BS26+OA36
    BEF66 = BS66-OA16
    BEF76 = BS36-OA25
    AF16 = -AF35*S6
    AF36 = AF35*C6
    AM16_1 = -AM35_1*S6+C4*C6
    AM36_1 = AM35_1*C6+C4*S6
    AM16_2 = -AM35_2*S6+S4*C6
    AM36_2 = AM35_2*C6+S4*S6
    AM16_3 = -C5*S6
    AM36_3 = C5*C6
    OB16_4 = -C5*S6
    OB36_4 = C5*C6
    OM17 = OM16*C7-OM36*S7
    OM27 = qd[7]+OM26
    OM37 = OM16*S7+OM36*C7
    OA17 = C7*(OA16-qd[7]*OM36)-S7*(OA36+qd[7]*OM16)
    OA37 = C7*(OA36+qd[7]*OM16)+S7*(OA16-qd[7]*OM36)
    AF17 = C7*(AF16+BS16*s.dpt[1,5])-S7*(AF36+BEF76*s.dpt[1,5])
    AF27 = AF25+BEF46*s.dpt[1,5]
    AF37 = C7*(AF36+BEF76*s.dpt[1,5])+S7*(AF16+BS16*s.dpt[1,5])
    AM17_1 = AM16_1*C7-AM36_1*S7
    AM37_1 = AM16_1*S7+AM36_1*C7
    AM17_2 = AM16_2*C7-AM36_2*S7
    AM37_2 = AM16_2*S7+AM36_2*C7
    AM17_3 = AM16_3*C7-AM36_3*S7
    AM37_3 = AM16_3*S7+AM36_3*C7
    OB17_4 = OB16_4*C7-OB36_4*S7
    OB37_4 = OB16_4*S7+OB36_4*C7
    AM17_4 = s.dpt[1,5]*S5*S7
    AM27_4 = OB36_4*s.dpt[1,5]
    AM37_4 = -s.dpt[1,5]*S5*C7
    AM27_5 = s.dpt[1,5]*S6
    AM17_6 = s.dpt[1,5]*S7
    AM37_6 = -s.dpt[1,5]*C7
    OM18 = OM16*C8-OM36*S8
    OM28 = qd[8]+OM26
    OM38 = OM16*S8+OM36*C8
    OA18 = C8*(OA16-qd[8]*OM36)-S8*(OA36+qd[8]*OM16)
    OA38 = C8*(OA36+qd[8]*OM16)+S8*(OA16-qd[8]*OM36)
    AF18 = C8*(AF16+BEF36*s.dpt[3,6]+BS16*s.dpt[1,6])-S8*(AF36+BEF76*s.dpt[1,6]+BS96*s.dpt[3,6])
    AF28 = AF25+BEF46*s.dpt[1,6]+BEF66*s.dpt[3,6]
    AF38 = C8*(AF36+BEF76*s.dpt[1,6]+BS96*s.dpt[3,6])+S8*(AF16+BEF36*s.dpt[3,6]+BS16*s.dpt[1,6])
    AM18_1 = AM16_1*C8-AM36_1*S8
    AM38_1 = AM16_1*S8+AM36_1*C8
    AM18_2 = AM16_2*C8-AM36_2*S8
    AM38_2 = AM16_2*S8+AM36_2*C8
    AM18_3 = AM16_3*C8-AM36_3*S8
    AM38_3 = AM16_3*S8+AM36_3*C8
    OB18_4 = OB16_4*C8-OB36_4*S8
    OB38_4 = OB16_4*S8+OB36_4*C8
    AM18_4 = s.dpt[1,6]*S5*S8+s.dpt[3,6]*S5*C8
    AM28_4 = -OB16_4*s.dpt[3,6]+OB36_4*s.dpt[1,6]
    AM38_4 = -s.dpt[1,6]*S5*C8+s.dpt[3,6]*S5*S8
    AM28_5 = s.dpt[1,6]*S6-s.dpt[3,6]*C6
    AM18_6 = s.dpt[1,6]*S8+s.dpt[3,6]*C8
    AM38_6 = -s.dpt[1,6]*C8+s.dpt[3,6]*S8
    OM19 = OM18*C9+OM28*S9
    OM29 = -OM18*S9+OM28*C9
    OM39 = qd[9]+OM38
    OA19 = C9*(OA18+qd[9]*OM28)+S9*(OA25-qd[9]*OM18)
    OA29 = C9*(OA25-qd[9]*OM18)-S9*(OA18+qd[9]*OM28)
    BS39 = OM19*OM39
    BS69 = OM29*OM39
    BS99 = -OM19*OM19-OM29*OM29
    BEF39 = BS39+OA29
    BEF69 = BS69-OA19
    AF19 = AF18*C9+AF28*S9
    AF29 = -AF18*S9+AF28*C9
    AM19_1 = AM18_1*C9+AM25_1*S9
    AM29_1 = -AM18_1*S9+AM25_1*C9
    AM19_2 = AM18_2*C9+AM25_2*S9
    AM29_2 = -AM18_2*S9+AM25_2*C9
    AM19_3 = AM18_3*C9+S5*S9
    AM29_3 = -AM18_3*S9+S5*C9
    OB19_4 = OB18_4*C9+S5*S9
    OB29_4 = -OB18_4*S9+S5*C9
    AM19_4 = AM18_4*C9+AM28_4*S9
    AM29_4 = -AM18_4*S9+AM28_4*C9
    OB19_5 = C6p8*C9
    OB29_5 = -C6p8*S9
    AM19_5 = AM28_5*S9
    AM29_5 = AM28_5*C9
    AM19_6 = AM18_6*C9
    AM29_6 = -AM18_6*S9
    OM110 = OM19*C10-OM39*S10
    OM210 = qd[10]+OM29
    OM310 = OM19*S10+OM39*C10
    OA110 = C10*(OA19-qd[10]*OM39)-S10*(OA38+qd[10]*OM19)
    OA310 = C10*(OA38+qd[10]*OM19)+S10*(OA19-qd[10]*OM39)
    AF110 = C10*(AF19+BEF39*s.dpt[3,8])-S10*(AF38+BS99*s.dpt[3,8])
    AF210 = AF29+BEF69*s.dpt[3,8]
    AF310 = C10*(AF38+BS99*s.dpt[3,8])+S10*(AF19+BEF39*s.dpt[3,8])
    AM110_1 = AM19_1*C10-AM38_1*S10
    AM310_1 = AM19_1*S10+AM38_1*C10
    AM110_2 = AM19_2*C10-AM38_2*S10
    AM310_2 = AM19_2*S10+AM38_2*C10
    AM110_3 = AM19_3*C10-AM38_3*S10
    AM310_3 = AM19_3*S10+AM38_3*C10
    OB110_4 = OB19_4*C10-OB38_4*S10
    OB310_4 = OB19_4*S10+OB38_4*C10
    AM110_4 = -AM38_4*S10+C10*(AM19_4+OB29_4*s.dpt[3,8])
    AM210_4 = AM29_4-OB19_4*s.dpt[3,8]
    AM310_4 = AM38_4*C10+S10*(AM19_4+OB29_4*s.dpt[3,8])
    OB110_5 = OB19_5*C10-S6p8*S10
    OB310_5 = OB19_5*S10+S6p8*C10
    AM110_5 = C10*(AM19_5+OB29_5*s.dpt[3,8])
    AM210_5 = AM29_5-OB19_5*s.dpt[3,8]
    AM310_5 = S10*(AM19_5+OB29_5*s.dpt[3,8])
    OB110_6 = C10*S9
    OB310_6 = S10*S9
    AM110_6 = -AM38_6*S10+C10*(AM19_6+s.dpt[3,8]*C9)
    AM210_6 = AM29_6-s.dpt[3,8]*S9
    AM310_6 = AM38_6*C10+S10*(AM19_6+s.dpt[3,8]*C9)
    OB110_8 = C10*S9
    OB310_8 = S10*S9
    AM110_8 = s.dpt[3,8]*C10*C9
    AM210_8 = -s.dpt[3,8]*S9
    AM310_8 = s.dpt[3,8]*S10*C9
 
# Backward Dynamics

    FA110 = -s.frc[1,10]+s.m[10]*AF110
    FA210 = -s.frc[2,10]+s.m[10]*AF210
    FA310 = -s.frc[3,10]+s.m[10]*AF310
    CF110 = -s.trq[1,10]+s.In[1,10]*OA110-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310
    CF210 = -s.trq[2,10]+s.In[1,10]*OM110*OM310+s.In[5,10]*OA29-s.In[9,10]*OM110*OM310
    CF310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OA310
    FB110_1 = s.m[10]*AM110_1
    FB210_1 = s.m[10]*AM29_1
    FB310_1 = s.m[10]*AM310_1
    FB110_2 = s.m[10]*AM110_2
    FB210_2 = s.m[10]*AM29_2
    FB310_2 = s.m[10]*AM310_2
    FB110_3 = s.m[10]*AM110_3
    FB210_3 = s.m[10]*AM29_3
    FB310_3 = s.m[10]*AM310_3
    FB110_4 = s.m[10]*AM110_4
    FB210_4 = s.m[10]*AM210_4
    FB310_4 = s.m[10]*AM310_4
    CM110_4 = s.In[1,10]*OB110_4
    CM210_4 = s.In[5,10]*OB29_4
    CM310_4 = s.In[9,10]*OB310_4
    FB110_5 = s.m[10]*AM110_5
    FB210_5 = s.m[10]*AM210_5
    FB310_5 = s.m[10]*AM310_5
    CM110_5 = s.In[1,10]*OB110_5
    CM210_5 = s.In[5,10]*OB29_5
    CM310_5 = s.In[9,10]*OB310_5
    FB110_6 = s.m[10]*AM110_6
    FB210_6 = s.m[10]*AM210_6
    FB310_6 = s.m[10]*AM310_6
    CM110_6 = s.In[1,10]*OB110_6
    CM210_6 = s.In[5,10]*C9
    CM310_6 = s.In[9,10]*OB310_6
    FB110_8 = s.m[10]*AM110_8
    FB210_8 = s.m[10]*AM210_8
    FB310_8 = s.m[10]*AM310_8
    CM110_8 = s.In[1,10]*OB110_8
    CM210_8 = s.In[5,10]*C9
    CM310_8 = s.In[9,10]*OB310_8
    CM110_9 = -s.In[1,10]*S10
    CM310_9 = s.In[9,10]*C10
    FF19 = -s.frc[1,9]+FA110*C10+FA310*S10
    FF29 = -s.frc[2,9]+FA210
    FF39 = -s.frc[3,9]-FA110*S10+FA310*C10
    CF19 = -s.trq[1,9]+CF110*C10+CF310*S10-FA210*s.dpt[3,8]
    CF29 = -s.trq[2,9]+CF210+s.dpt[3,8]*(FA110*C10+FA310*S10)
    CF39 = -s.trq[3,9]-CF110*S10+CF310*C10
    FM19_1 = FB110_1*C10+FB310_1*S10
    FM39_1 = -FB110_1*S10+FB310_1*C10
    CM19_1 = -FB210_1*s.dpt[3,8]
    CM29_1 = s.dpt[3,8]*(FB110_1*C10+FB310_1*S10)
    FM19_2 = FB110_2*C10+FB310_2*S10
    FM39_2 = -FB110_2*S10+FB310_2*C10
    CM19_2 = -FB210_2*s.dpt[3,8]
    CM29_2 = s.dpt[3,8]*(FB110_2*C10+FB310_2*S10)
    FM19_3 = FB110_3*C10+FB310_3*S10
    FM39_3 = -FB110_3*S10+FB310_3*C10
    CM19_3 = -FB210_3*s.dpt[3,8]
    CM29_3 = s.dpt[3,8]*(FB110_3*C10+FB310_3*S10)
    FM19_4 = FB110_4*C10+FB310_4*S10
    FM39_4 = -FB110_4*S10+FB310_4*C10
    CM19_4 = CM110_4*C10+CM310_4*S10-FB210_4*s.dpt[3,8]
    CM29_4 = CM210_4+s.dpt[3,8]*(FB110_4*C10+FB310_4*S10)
    CM39_4 = -CM110_4*S10+CM310_4*C10
    FM19_5 = FB110_5*C10+FB310_5*S10
    FM39_5 = -FB110_5*S10+FB310_5*C10
    CM19_5 = CM110_5*C10+CM310_5*S10-FB210_5*s.dpt[3,8]
    CM29_5 = CM210_5+s.dpt[3,8]*(FB110_5*C10+FB310_5*S10)
    CM39_5 = -CM110_5*S10+CM310_5*C10
    FM19_6 = FB110_6*C10+FB310_6*S10
    FM39_6 = -FB110_6*S10+FB310_6*C10
    CM19_6 = CM110_6*C10+CM310_6*S10-FB210_6*s.dpt[3,8]
    CM29_6 = CM210_6+s.dpt[3,8]*(FB110_6*C10+FB310_6*S10)
    CM39_6 = -CM110_6*S10+CM310_6*C10
    CM19_8 = CM110_8*C10+CM310_8*S10-FB210_8*s.dpt[3,8]
    CM29_8 = CM210_8+s.dpt[3,8]*(FB110_8*C10+FB310_8*S10)
    CM39_8 = -CM110_8*S10+CM310_8*C10
    CM39_9 = -CM110_9*S10+CM310_9*C10
    FF18 = FF19*C9-FF29*S9
    FF28 = FF19*S9+FF29*C9
    CF18 = CF19*C9-CF29*S9
    CF28 = CF19*S9+CF29*C9
    FM18_1 = -FB210_1*S9+FM19_1*C9
    FM28_1 = FB210_1*C9+FM19_1*S9
    CM18_1 = CM19_1*C9-CM29_1*S9
    CM28_1 = CM19_1*S9+CM29_1*C9
    FM18_2 = -FB210_2*S9+FM19_2*C9
    FM28_2 = FB210_2*C9+FM19_2*S9
    CM18_2 = CM19_2*C9-CM29_2*S9
    CM28_2 = CM19_2*S9+CM29_2*C9
    FM18_3 = -FB210_3*S9+FM19_3*C9
    FM28_3 = FB210_3*C9+FM19_3*S9
    CM18_3 = CM19_3*C9-CM29_3*S9
    CM28_3 = CM19_3*S9+CM29_3*C9
    FM18_4 = -FB210_4*S9+FM19_4*C9
    FM28_4 = FB210_4*C9+FM19_4*S9
    CM18_4 = CM19_4*C9-CM29_4*S9
    CM28_4 = CM19_4*S9+CM29_4*C9
    FM18_5 = -FB210_5*S9+FM19_5*C9
    FM28_5 = FB210_5*C9+FM19_5*S9
    CM18_5 = CM19_5*C9-CM29_5*S9
    CM28_5 = CM19_5*S9+CM29_5*C9
    FM18_6 = -FB210_6*S9+FM19_6*C9
    CM28_6 = CM19_6*S9+CM29_6*C9
    CM28_8 = CM19_8*S9+CM29_8*C9
    FA17 = -s.frc[1,7]+s.m[7]*AF17
    FA27 = -s.frc[2,7]+s.m[7]*AF27
    FA37 = -s.frc[3,7]+s.m[7]*AF37
    CF17 = -s.trq[1,7]+s.In[1,7]*OA17-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37
    CF27 = -s.trq[2,7]+s.In[1,7]*OM17*OM37+s.In[5,7]*OA25-s.In[9,7]*OM17*OM37
    CF37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OA37
    FB17_1 = s.m[7]*AM17_1
    FB27_1 = s.m[7]*AM25_1
    FB37_1 = s.m[7]*AM37_1
    FB17_2 = s.m[7]*AM17_2
    FB27_2 = s.m[7]*AM25_2
    FB37_2 = s.m[7]*AM37_2
    FB17_3 = s.m[7]*AM17_3
    FB27_3 = s.m[7]*S5
    FB37_3 = s.m[7]*AM37_3
    FB17_4 = s.m[7]*AM17_4
    FB27_4 = s.m[7]*AM27_4
    FB37_4 = s.m[7]*AM37_4
    CM17_4 = s.In[1,7]*OB17_4
    CM27_4 = s.In[5,7]*S5
    CM37_4 = s.In[9,7]*OB37_4
    FB27_5 = s.m[7]*AM27_5
    CM17_5 = s.In[1,7]*C6p7
    CM37_5 = s.In[9,7]*S6p7
    FB17_6 = s.m[7]*AM17_6
    FB37_6 = s.m[7]*AM37_6
    FA16 = -s.frc[1,6]+s.m[6]*(AF16+BEF36*s.l[3,6]+BS16*s.l[1,6])
    FA26 = -s.frc[2,6]+s.m[6]*(AF25+BEF46*s.l[1,6]+BEF66*s.l[3,6])
    FA36 = -s.frc[3,6]+s.m[6]*(AF36+BEF76*s.l[1,6]+BS96*s.l[3,6])
    FF16 = FA16+FA17*C7+FA37*S7+FF18*C8+FF39*S8
    FF26 = FA26+FA27+FF28
    FF36 = FA36-FA17*S7+FA37*C7-FF18*S8+FF39*C8
    CF16 = -s.trq[1,6]+s.In[1,6]*OA16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+CF17*C7+CF18*C8+CF37*S7+CF39*S8-FA26*s.l[3,6]-FF28*s.dpt[3,6]
    CF26 = -s.trq[2,6]+CF27+CF28+s.In[1,6]*OM16*OM36+s.In[5,6]*OA25-s.In[9,6]*OM16*OM36+FA16*s.l[3,6]-FA36*s.l[1,6]-s.dpt[1,5]*(-FA17*S7+FA37*C7)-s.dpt[1,6]*(-FF18*S8+FF39*C8)+s.dpt[3,6]*(FF18*C8+FF39*S8)
    CF36 = -s.trq[3,6]-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OA36-CF17*S7-CF18*S8+CF37*C7+CF39*C8+FA26*s.l[1,6]+FA27*s.dpt[1,5]+FF28*s.dpt[1,6]
    FB16_1 = s.m[6]*AM16_1
    FB26_1 = s.m[6]*AM25_1
    FB36_1 = s.m[6]*AM36_1
    FM16_1 = FB16_1+FB17_1*C7+FB37_1*S7+FM18_1*C8+FM39_1*S8
    FM26_1 = FB26_1+FB27_1+FM28_1
    FM36_1 = FB36_1-FB17_1*S7+FB37_1*C7-FM18_1*S8+FM39_1*C8
    CM16_1 = CM18_1*C8-FB26_1*s.l[3,6]-FM28_1*s.dpt[3,6]
    CM26_1 = CM28_1+FB16_1*s.l[3,6]-FB36_1*s.l[1,6]-s.dpt[1,5]*(-FB17_1*S7+FB37_1*C7)-s.dpt[1,6]*(-FM18_1*S8+FM39_1*C8)+s.dpt[3,6]*(FM18_1*C8+FM39_1*S8)
    CM36_1 = -CM18_1*S8+FB26_1*s.l[1,6]+FB27_1*s.dpt[1,5]+FM28_1*s.dpt[1,6]
    FB16_2 = s.m[6]*AM16_2
    FB26_2 = s.m[6]*AM25_2
    FB36_2 = s.m[6]*AM36_2
    FM16_2 = FB16_2+FB17_2*C7+FB37_2*S7+FM18_2*C8+FM39_2*S8
    FM26_2 = FB26_2+FB27_2+FM28_2
    FM36_2 = FB36_2-FB17_2*S7+FB37_2*C7-FM18_2*S8+FM39_2*C8
    CM16_2 = CM18_2*C8-FB26_2*s.l[3,6]-FM28_2*s.dpt[3,6]
    CM26_2 = CM28_2+FB16_2*s.l[3,6]-FB36_2*s.l[1,6]-s.dpt[1,5]*(-FB17_2*S7+FB37_2*C7)-s.dpt[1,6]*(-FM18_2*S8+FM39_2*C8)+s.dpt[3,6]*(FM18_2*C8+FM39_2*S8)
    CM36_2 = -CM18_2*S8+FB26_2*s.l[1,6]+FB27_2*s.dpt[1,5]+FM28_2*s.dpt[1,6]
    FB16_3 = s.m[6]*AM16_3
    FB26_3 = s.m[6]*S5
    FB36_3 = s.m[6]*AM36_3
    FM16_3 = FB16_3+FB17_3*C7+FB37_3*S7+FM18_3*C8+FM39_3*S8
    FM26_3 = FB26_3+FB27_3+FM28_3
    FM36_3 = FB36_3-FB17_3*S7+FB37_3*C7-FM18_3*S8+FM39_3*C8
    CM16_3 = CM18_3*C8-FB26_3*s.l[3,6]-FM28_3*s.dpt[3,6]
    CM26_3 = CM28_3+FB16_3*s.l[3,6]-FB36_3*s.l[1,6]-s.dpt[1,5]*(-FB17_3*S7+FB37_3*C7)-s.dpt[1,6]*(-FM18_3*S8+FM39_3*C8)+s.dpt[3,6]*(FM18_3*C8+FM39_3*S8)
    CM36_3 = -CM18_3*S8+FB26_3*s.l[1,6]+FB27_3*s.dpt[1,5]+FM28_3*s.dpt[1,6]
    FB16_4 = s.m[6]*s.l[3,6]*S5
    FB26_4 = s.m[6]*(-OB16_4*s.l[3,6]+OB36_4*s.l[1,6])
    FB36_4 = -s.m[6]*s.l[1,6]*S5
    CM16_4 = s.In[1,6]*OB16_4+CM17_4*C7+CM18_4*C8+CM37_4*S7+CM39_4*S8-FB26_4*s.l[3,6]-FM28_4*s.dpt[3,6]
    CM26_4 = CM27_4+CM28_4+s.In[5,6]*S5+FB16_4*s.l[3,6]-FB36_4*s.l[1,6]-s.dpt[1,5]*(-FB17_4*S7+FB37_4*C7)-s.dpt[1,6]*(-FM18_4*S8+FM39_4*C8)+s.dpt[3,6]*(FM18_4*C8+FM39_4*S8)
    CM36_4 = s.In[9,6]*OB36_4-CM17_4*S7-CM18_4*S8+CM37_4*C7+CM39_4*C8+FB26_4*s.l[1,6]+FB27_4*s.dpt[1,5]+FM28_4*s.dpt[1,6]
    FB26_5 = s.m[6]*(s.l[1,6]*S6-s.l[3,6]*C6)
    CM16_5 = s.In[1,6]*C6+CM17_5*C7+CM18_5*C8+CM37_5*S7+CM39_5*S8-FB26_5*s.l[3,6]-FM28_5*s.dpt[3,6]
    CM26_5 = CM28_5-s.dpt[1,6]*(-FM18_5*S8+FM39_5*C8)+s.dpt[3,6]*(FM18_5*C8+FM39_5*S8)
    CM36_5 = s.In[9,6]*S6-CM17_5*S7-CM18_5*S8+CM37_5*C7+CM39_5*C8+FB26_5*s.l[1,6]+FB27_5*s.dpt[1,5]+FM28_5*s.dpt[1,6]
    FB16_6 = s.m[6]*s.l[3,6]
    FB36_6 = -s.m[6]*s.l[1,6]
    CM26_6 = s.In[5,6]+s.In[5,7]+CM28_6+FB16_6*s.l[3,6]-FB36_6*s.l[1,6]-s.dpt[1,5]*(-FB17_6*S7+FB37_6*C7)-s.dpt[1,6]*(-FM18_6*S8+FM39_6*C8)+s.dpt[3,6]*(FM18_6*C8+FM39_6*S8)
    FF15 = FF16*C6+FF36*S6
    FF35 = -FF16*S6+FF36*C6
    CF15 = CF16*C6+CF36*S6
    CF35 = -CF16*S6+CF36*C6
    FM15_1 = FM16_1*C6+FM36_1*S6
    FM35_1 = -FM16_1*S6+FM36_1*C6
    CM15_1 = CM16_1*C6+CM36_1*S6
    CM35_1 = -CM16_1*S6+CM36_1*C6
    FM15_2 = FM16_2*C6+FM36_2*S6
    FM35_2 = -FM16_2*S6+FM36_2*C6
    CM15_2 = CM16_2*C6+CM36_2*S6
    CM35_2 = -CM16_2*S6+CM36_2*C6
    FM35_3 = -FM16_3*S6+FM36_3*C6
    CM15_3 = CM16_3*C6+CM36_3*S6
    CM35_3 = -CM16_3*S6+CM36_3*C6
    CM15_4 = CM16_4*C6+CM36_4*S6
    CM35_4 = -CM16_4*S6+CM36_4*C6
    CM15_5 = CM16_5*C6+CM36_5*S6
    FF24 = FF26*C5-FF35*S5
    FF34 = FF26*S5+FF35*C5
    CF34 = CF26*S5+CF35*C5
    FM24_1 = FM26_1*C5-FM35_1*S5
    FM34_1 = FM26_1*S5+FM35_1*C5
    CM34_1 = CM26_1*S5+CM35_1*C5
    FM24_2 = FM26_2*C5-FM35_2*S5
    FM34_2 = FM26_2*S5+FM35_2*C5
    CM34_2 = CM26_2*S5+CM35_2*C5
    FM34_3 = FM26_3*S5+FM35_3*C5
    CM34_3 = CM26_3*S5+CM35_3*C5
    CM34_4 = CM26_4*S5+CM35_4*C5
    FF13 = FF15*C4-FF24*S4
    FF23 = FF15*S4+FF24*C4
    FM13_1 = FM15_1*C4-FM24_1*S4
    FM23_1 = FM15_1*S4+FM24_1*C4
    FM23_2 = FM15_2*S4+FM24_2*C4
 
# Symbolic model output

    c[1] = FF13
    c[2] = FF23
    c[3] = FF34
    c[4] = CF34
    c[5] = CF15
    c[6] = CF26
    c[7] = CF27
    c[8] = CF28
    c[9] = CF39
    c[10] = CF210
    M[1,1] = FM13_1
    M[1,2] = FM23_1
    M[1,3] = FM34_1
    M[1,4] = CM34_1
    M[1,5] = CM15_1
    M[1,6] = CM26_1
    M[1,8] = CM28_1
    M[2,1] = FM23_1
    M[2,2] = FM23_2
    M[2,3] = FM34_2
    M[2,4] = CM34_2
    M[2,5] = CM15_2
    M[2,6] = CM26_2
    M[2,8] = CM28_2
    M[3,1] = FM34_1
    M[3,2] = FM34_2
    M[3,3] = FM34_3
    M[3,4] = CM34_3
    M[3,5] = CM15_3
    M[3,6] = CM26_3
    M[3,8] = CM28_3
    M[4,1] = CM34_1
    M[4,2] = CM34_2
    M[4,3] = CM34_3
    M[4,4] = CM34_4
    M[4,5] = CM15_4
    M[4,6] = CM26_4
    M[4,7] = CM27_4
    M[4,8] = CM28_4
    M[4,9] = CM39_4
    M[4,10] = CM210_4
    M[5,1] = CM15_1
    M[5,2] = CM15_2
    M[5,3] = CM15_3
    M[5,4] = CM15_4
    M[5,5] = CM15_5
    M[5,6] = CM26_5
    M[5,8] = CM28_5
    M[5,9] = CM39_5
    M[5,10] = CM210_5
    M[6,1] = CM26_1
    M[6,2] = CM26_2
    M[6,3] = CM26_3
    M[6,4] = CM26_4
    M[6,5] = CM26_5
    M[6,6] = CM26_6
    M[6,7] = s.In[5,7]
    M[6,8] = CM28_6
    M[6,9] = CM39_6
    M[6,10] = CM210_6
    M[7,4] = CM27_4
    M[7,6] = s.In[5,7]
    M[7,7] = s.In[5,7]
    M[8,1] = CM28_1
    M[8,2] = CM28_2
    M[8,3] = CM28_3
    M[8,4] = CM28_4
    M[8,5] = CM28_5
    M[8,6] = CM28_6
    M[8,8] = CM28_8
    M[8,9] = CM39_8
    M[8,10] = CM210_8
    M[9,4] = CM39_4
    M[9,5] = CM39_5
    M[9,6] = CM39_6
    M[9,8] = CM39_8
    M[9,9] = CM39_9
    M[10,4] = CM210_4
    M[10,5] = CM210_5
    M[10,6] = CM210_6
    M[10,8] = CM210_8
    M[10,10] = s.In[5,10]

# Number of continuation lines = 0


