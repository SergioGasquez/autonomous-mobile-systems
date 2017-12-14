function out=controlTrayectory_GasquezSergio(T,q)

Xr=2*cos(2*pi*T/30)+5*cos(2/3*2*pi*T/30);  %  hypotrochoids  (Wiki, R = 5, r = 3, d = 5)
Yr=2*sin(2*pi*T/30)-5*sin(2/3*2*pi*T/30);


Kphi=3;
Kv=1;

Fir = atan2(Yr -q(2) , Xr - q(1));
qRef = [Xr Yr Fir];

e=qRef - q;
e(3)=wrapToPi(e(3));

alpha = e(3)*Kphi;
v = sqrt(e(1)^2+e(2)^2)*Kv;
w = alpha;


out=[v,w];

end