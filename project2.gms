set k 'number of steps' / 0 * 9 /;

scalar
    h 'time interval' /1/
    alpha 'limit of u' /2/;

variables
    x1(k)      z_1 -coordinate at time kh
    x2(k)      the velocity in the z_1 -direction at time kh
    x3(k)      z_2 -coordinate at time kh
    x4(k)      the velocity in the z_2 -direction at time kh
    u1(k)      control signal in z_1 -direction at time kh
    u2(k)      control signal in z_2 -direction at time kh
    z(k)       whether the robot need one more step to reach the destination
    T          totalstep;

binary variables z;


u1.up(k) = alpha;
u1.lo(k) = -alpha;
u2.up(k) = alpha;
u2.lo(k) = -alpha;


equations
    x1_equ(k)    the updated location in direction z_1
    x2_equ(k)    the updated velocity in direction z_1
    x3_equ(k)    the updated location in direction z_2
    x4_equ(k)    the updated velocity in direction z_2     
    x11_loc(k)     whether the robot need one more step to reach the destination according to constraint11 at time kh
    x12_loc(k)     whether the robot need one more step to reach the destination according to constraint12 at time kh
    x21_loc(k)     whether the robot need one more step to reach the destination according to constraint21 at time kh
    x22_loc(k)     whether the robot need one more step to reach the destination according to constraint22 at time kh
    x31_loc(k)     whether the robot need one more step to reach the destination according to constraint31 at time kh
    x32_loc(k)     whether the robot need one more step to reach the destination according to constraint32 at time kh
    x41_loc(k)     whether the robot need one more step to reach the destination according to constraint41 at time kh
    x42_loc(k)     whether the robot need one more step to reach the destination according to constraint42 at time kh
    T_equ    totalstep;

x1_equ(k) .. x1(k+1)  =e= x1(k) + x2(k) * h + sqr(h)/2 * u1(k);
x2_equ(k) .. x2(k+1)  =e= x2(k) + h * u1(k) ;
x3_equ(k) .. x3(k+1)  =e= x3(k) + x4(k) * h + sqr(h)/2 * u2(k);
x4_equ(k) .. x4(k+1)  =e= x4(k) + h * u2(k) ;
x11_loc(k) .. z(k) =g= (5.9 - x1(k))/100 + z(k-1) - 1; 
x12_loc(k) .. z(k) =g= (x1(k) -6.1)/100 + z(k-1) - 1; 
x21_loc(k) .. z(k) =g= (-0.1 -x2(k))/100 + z(k-1) - 1; 
x22_loc(k) .. z(k) =g= (x2(k) -0.1)/100 + z(k-1) - 1; 
x31_loc(k) ..  z(k) =g= (6.9 - x3(k))/100 + z(k-1) - 1; 
x32_loc(k) .. z(k) =g= (x3(k) -7.1)/100 + z(k-1) - 1; 
x41_loc(k) ..z(k) =g= (-0.1 - x4(k))/100 + z(k-1) - 1; 
x42_loc(k) .. z(k) =g= (x4(k) - 0.1)/100 + z(k-1) - 1;
T_equ  .. T =e= sum(k,z(k));

x1.fx('0') = 0;
x2.fx('0') = 0;
x3.fx('0') = 0;
x4.fx('0') = 0;
z.fx('0') = 1;




Model project2 / all /;

solve project2 miniziming T using mip;

display T.l,u1.l,u2.l,z.l,x1.l,x2.l,x3.l,x4.l;