function A0n = fwkin(theta,d,a)
%Computation of the transformation matrix based on DH parameters
q = deg2rad(theta);
%precision of round function
pr = 5;
for i = 1:7
    temp      = [ round(cos(q(i)),pr)         -round(sin(q(i))*cos(a(i)),pr)   round(sin(q(i))*sin(a(i)),pr)    0;
                  round(sin(q(i)),pr)         round(cos(q(i))*cos(a(i)),pr)   -round(cos(q(i))*sin(a(i)),pr)    0;  
                  0                           round(sin(a(i)),pr)              round(cos(a(i)),pr)              d(i);
                  0                                     0                         0                             1;
        ];
   A(i)= {temp};
end 
A0n = A{1} * A{2} * A{3} * A{4} * A{5} * A{6}* A{7};
end