 close all, clear all

 % Reference path can be CHANGED (points and their number)
 T=[1 1;   %reference points that define line segments
   5 5;
   1 5;
   1 2;
  -3 3];

%   T=[1 1;  5 3; 2 5;-3 3];  %reference points that define line segments

q=[3 2 0.6*pi];  % initial state can be CHANGED
 
%%%%%%%%%%%%% the rest of this file will NOT CHANGE %%%%%%%%%%%%%%%%%%%%%%  
r = 0.04; % Wheel radius 
L = 0.08; % Axle length 
Ts = 0.033; % Sampling time 
tt = 0:Ts:200; % Simulation time 
limitations=1; % use limitations or not


Q=[];Qr=[];U=[];Tvzorcenja=[];
t=0; v=0; w=0; 
Q=[Q;q];U=[U;[v, w]]; Tvzorcenja=[Tvzorcenja; t];

vOldR=0;vOldL=0;


for j=1:length(tt)

    
   %%%%%%%%%%%%%%%% CONTROLER - write your own function control_SurnameName in a separate file %%%%%%%%%%%
    
   vw=control_GasquezSergio(T,q);
   %vw=control_SurnameName;
   v=vw(1);
   w=vw(2);
   %%%%%%%%%%%%%%%% end: CONTROLER %%%%%%%%%%


   if limitations
        % consider robot limits (velocity, acceleration, radial acceleration) 
        Vmax=0.6; Amax=0.8; ArMax=0.7;
        vr=v+w*L/2;
        vl=v-w*L/2;

        % lateral slip
        VmaxR=ArMax/w;
        if(abs(v)>abs(VmaxR)&&1) 
            dddd=(abs(v)-abs(VmaxR))*Ts;
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

        v=(vr+vl)/2;
        w=(vr-vl)/L;

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
   
    t=t+Ts;
    
    Q=[Q;q];U=[U;[v, w]];
    Tvzorcenja=[Tvzorcenja; t];

    
    
    % evaluate results
    distToGoal=sqrt( (T(end,1)-q(1))^2+ (T(end,2)-q(2))^2);
    if(distToGoal<0.1 || j==length(tt))
        disp('================================================')
        disp('Driving time [s] and  squared sum error [m2]'); 
        TT=[];
        
        %simplified driving critiria
        for g=1:size(T,1)-1
         %   TT=[TT; T(g,:)+ (T(g+1,:)-T(g,:)).*(0:.01:1)'];
            TTx=[ T(g,1)+ (T(g+1,1)-T(g,1)).*(0:.01:1)'];
            TTy=[ T(g,2)+ (T(g+1,2)-T(g,2)).*(0:.01:1)'];
            TT=[TT; TTx,TTy]; 
        end
        crit=0;
        for g=1:size(Q,1)
            dd=(repmat(Q(g,1),size(TT,1),1) -TT(:,1)).^2+ (repmat(Q(g,2),size(TT,1),1)-TT(:,2)).^2;
            crit=crit+min(dd);
        end
        
        [t,crit]
 
        disp('================================================')
        break;
        
    end

    
end


figure
plot(Q(:,1),Q(:,2),T(:,1),T(:,2),'--',T(:,1),T(:,2),'o'), xlabel('x[m]'), ylabel('y[m]')
%print -depsc -tiff -r300 sekvencaXY 
figure
plot(Tvzorcenja,U(:,1),Tvzorcenja,U(:,2)), xlabel('t[s]'), ylabel('v[m/s],\omega[1/s]'), legend('v','w')
%print -depsc -tiff -r300 sekvencaU 
