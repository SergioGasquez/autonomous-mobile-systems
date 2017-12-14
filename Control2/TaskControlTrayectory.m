 close all, clear all

 r = 0.04; % Wheel radius 
 L = 0.08; % Axle length 
 Ts = 0.033; % Sampling time 
 tt = 0:Ts:90; % Simulation time 
 limitations=1; % use limitations or not

  % reference path
 Xr=2*cos(2*pi*tt/30)+5*cos(2/3*2*pi*tt/30);  %  hypotrochoids  (Wiki, R = 5, r = 3, d = 5)
 Yr=2*sin(2*pi*tt/30)-5*sin(2/3*2*pi*tt/30);
 

q = [Xr(1), Yr(1), -pi/2] + [1 .5 +pi/201]*1;   % ONLY initial state can be CHANGED

 
%%%%%%%%%%%%% the rest of this file will NOT CHANGE %%%%%%%%%%%%%%%%%%%%%%  


 tt = 0:Ts:90; % Simulation time 


Q=[];U=[];Tvzorcenja=[];
t=0; v=0; w=0; 
Q=[Q;q];U=[U;[v, w]]; Tvzorcenja=[Tvzorcenja; t];

vOldR=0;vOldL=0;


for j=1:length(tt)

   
   %%%%%%%%%%%%%%%% CONTROLER - write your own function control_SurnameName in a separate file %%%%%%%%%%%
    
   vw=controlTrayectory_GasquezSergio(tt(j),q);%Input is time and pose
   v=vw(1);
   w=vw(2);
   %%%%%%%%%%%%%%%% end: CONTROLER %%%%%%%%%%


   if limitations
        % consider robot limits (velocity, acceleration, radial acceleration) 
        Vmax=1.5; Amax=3; ArMax=1;
        vr=v+w*L/2;
        vl=v-w*L/2;

        % lateral slip
        VmaxR=ArMax/w;
        if(abs(v)>abs(VmaxR)&&0) 
            dddd=(abs(v)-abs(VmaxR))*Ts
            if(vr>vl)
                q=q+[sin(q(3)),-cos(q(3)),0]*dddd; 
            elseif(vl<vr)
                q=q-[sin(q(3)),-cos(q(3)),0]*dddd; 
            end
        end

        % limit acc. and velocity of the wheels
        if( abs(vr-vOldR)>Amax*Ts), vr= vOldR+Amax*Ts*sign(vr-vOldR); end
        if( abs(vl-vOldL)>Amax*Ts), vl= vOldL+Amax*Ts*sign(vl-vOldL); end

        if abs(vr)>Vmax, vr=Vmax*sign(vr); end;
        if abs(vl)>Vmax, vl=Vmax*sign(vl); end;

        v=(vr+vl)/2 + randn(1,1)*.01;  % noise on inputs
        w=(vr-vl)/L + randn(1,1)*.01;

        vOldR=vr;vOldL=vl;
   end
    
    %%%%%%%%%%%%%%% simulate robot motion
    Theta=q(1,3);

    if(w~=0)
        q(1)=q(1)+v/w*(sin(q(3)+w*Ts)-sin(q(3)));     % exact method
        q(2)=q(2)-v/w*(cos(q(3)+w*Ts)-cos(q(3)));
    else
        q(1)=q(1)+v*Ts*cos(q(3));     
        q(2)=q(2)+v*Ts*sin(q(3));
    end
    q(3)=q(3)+w*Ts;
     
    q(3)=wrapToPi(q(3));     % Correct orientation
   
    q=q+randn(1,3).*[.005 , .005, .01] ;   % noise on sensor
    
    t=t+Ts;
    

    
    
    % evaluate results
   % distToGoal=sqrt( (Qr(end,1)-q(1))^2+ (Qr(end,2)-q(2))^2);
    if( j==length(tt))
        disp('================================================')
        disp('Squared sum error [m2]'); 
         
        crit= sum( (Q(:,1)-Xr').^2 + (Q(:,2)-Yr').^2 );
                
        [crit]
 
        disp('================================================')
        break;
        
    end
    
    Q=[Q;q];U=[U;[v, w]];
    Tvzorcenja=[Tvzorcenja; t];

    
end


figure
plot(Q(:,1),Q(:,2),Xr,Yr,'--'), xlabel('x[m]'), ylabel('y[m]')
%print -depsc -tiff -r300 sekvencaXY 
figure
plot(Tvzorcenja,U(:,1),Tvzorcenja,U(:,2)), xlabel('t[s]'), ylabel('v[m/s],\omega[1/s]'), legend('v','w')
%print -depsc -tiff -r300 sekvencaU 
