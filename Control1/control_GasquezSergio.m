function out=control_GasquezSergio(T,q)

persistent a;
if isempty(a)
    a=1
end

dx=T(a+1,1) - T(a,1);
dy=T(a+1,2) - T(a,2);
v =[dx;dy];
vT=v';
vN=[dy;-dx];
r= q(1:2) - T(a,:);

u= (r*v)/((v')*v);

if u>1 && a < size(T,1)-1
 a = a + 1;
 dx=T(a+1,1) - T(a,1);
 dy=T(a+1,2) - T(a,2);
 v =[dx;dy];
 vT=v';
 vN=[dy;-dx];
 r= q(1:2) - T(a,:);
end

%  Normalized robot distance to line segment
 dn = (r*vN)/(vN'*vN);

 fiLin= atan2(dy,dx);
 K1=12;
 fiRot= atan(K1*dn);
 fiRef=(fiLin+fiRot);
 fiErr=wrapToPi(fiRef-q(3));
 K2=10;
 w=K2*fiErr;
 v=0.5*cos(fiErr);          
 
out=[v,w];
end