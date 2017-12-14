function diffPogonOZ
close all, clear all %path planning of differential drive
global R L Ts Okolje

ANIMATE=1; % enable animation

% We define times
Ts=0.033; % sampling time
t=0:Ts:10; % time samples in simulation

% We define length of the wheel and distance between wheels
R=0.04; % wheel radious
L=0.08; % distance between the wheels

% Inital pose(then q is the current pose)
q=[0.5 0.5 0];	% true initial state
%q=[2.2 1.8 0];	%true initial state


Q=[];Qr=[];Qo=[];E=[];Etrans=[];U=[];Tvzorcenja=[]; EE=[];
tt=0;

%init. graphic
global hhh  
if (ANIMATE) InitGrafic; end

% We define the goal point
goal=[2, 2.5];    % desired goal

q_ref=goal;   

% define and draw environment
Okolje=[];
Okolje=DefinirajOkolje();


for i=1:length(t)
    %%%%%%%%%%% distance sensor
    [d,xo,yo,fio]=DistanceSensor(q); % returns min distance to obstacle, closest point on the obstacle and tangent to the obstacle shape in this point 
    % We define the distance that we want to keep with the osbtacles
    dThreshold = 0.15;
    % V gain(K1) and W gain(K2)
    K1=2;
    K2=25;
    % If the distance is greater than the threshold we will calculate the
    % phiT angle,v, the phi error and w
    if (d > dThreshold)
        phiT=atan2((q_ref(2)-q(2)),(q_ref(1)-q(1)));
        v=(K1*sqrt((q_ref(1)-q(1))^2+(q_ref(2)-q(2))^2));
        ePhi=phiT-q(3);
        w=K2*(ePhi);      
    % If we are close enough to the wall we will just rotate
    else
        phiT=fio;
        v=0;
        ePhi=phiT-q(3);
        w=K2*(ePhi);
        % And then if the abs value of the phi error is lower than a
        % threshold we will keep advancing again
        if (abs(ePhi)<(0.15))
            v=K1*sqrt((q_ref(1)-q(1))^2+(q_ref(2)-q(2))^2);
        end
    end
    
    

    
    
    %%%%%%%%%%% control determination of inputs considerig sensed obstacles
%     v=.5;
%     w=.5;    
       
    
    %%%%%%%%%%% true robot motion simulation
    [q]=SimulacijaDejanskegaRobora(q,v,w);
       
    %%%%%%%%%%% store results
    Q=[Q;q]; 
    Qr=[Qr;q_ref]; 
    U=[U;[v, w]];  
    Tvzorcenja=[Tvzorcenja; tt];
    tt=tt+Ts;
       
   if (ANIMATE)
            

       DrawRobot(q',1);     % drugi parameter: robot=1, odometrija=2
     % DrawRobot(q_odo',2); % drugi parameter: robot=1, odometrija=2
        set(hhh(3),'XData',Q(:,1),'YData',Q(:,2));  % izris poti
     %  set(hhh(4),'XData',Qo(:,1),'YData',Qo(:,2));  % izris odometrije
        set(hhh(5),'XData',Qr(:,1),'YData',Qr(:,2));  % izris referenène poti
    
        pause(0.03);
        drawnow;        
   end
   
end

% figure
% plot(Q(:,1),Q(:,2),Qo(:,1),Qo(:,2),'--')
% xlabel('t [s]'),ylabel('x [m], y [m]')
% figure
% plot(Tvzorcenja,U(:,1),Tvzorcenja,U(:,2),'--')
% xlabel('t [s]'),ylabel('v [m/s], \omega [rad/s]'),legend('v','\omega'),







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% FUNKCIJE    %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [d,xo,yo,tano] = DistanceSensor(q)
    global Okolje
    
    persistent gg;
    if isempty(gg), gg=0; end
    
    
    
    d=20;  % maks dsensor distance range
    x=q(1); y=q(2);
    xo=0;yo=0;  % returned point on obstacle
    tangenta=0;
    
    
    for i=1:size(Okolje,1)
        
        x1=Okolje(i,1); x2=Okolje(i,3); 
        y1=Okolje(i,2); y2=Okolje(i,4);
        
        A=y2-y1;  % enaèba premice Ax+By+C=0  
        B=-x2+x1;
        C=-x1*A - y1*B;
        mi=-1/(sqrt(A^2+B^2)); if C<0, mi=-mi; end
        
        % distance between robot (its center) and straight line 
        dp=abs(A*mi*x+B*mi*y+C*mi);
   
        d1=sqrt((x1-x)^2+(y1-y)^2);
        d2=sqrt((x2-x)^2+(y2-y)^2);
        ds=sqrt(((x1+x2)/2-x)^2+((y1+y2)/2-y)^2);
        tangenta_t=atan2(A,-B);
        
          A1=-B;
          B1=A;
          C1=-x*A1-y*B1;
          Det=A1*B-A*B1;
          
     %     if(Det~=0)
          
          xot=(B1*C-B*C1)/Det;   % closest point on the straight line
          yot=(C1*A-C*A1)/Det;
          % check if the point is inside line segment 
          pogoj1= (x1<= xot && xot <= x2) ||(x2<= xot && xot <= x1);
          pogoj2= (y1<= yot && yot <= y2) ||(y2<= yot && yot <= y1);
        
       %   end
          
          
        if (pogoj1&pogoj2) 
          d_tmp=dp;
        elseif d1<d2
          d_tmp=d1;  
          xot=x1; yot=y1;
        else
          d_tmp=d2; 
          xot=x2; yot=y2;
        end
        
        if(d_tmp<d)  
            d=d_tmp;
            xo=xot; yo=yot;
            
            tano=tangenta_t;
%             if(cos(tangenta_t-q(3))>cos(tangenta_t+pi-q(3)))
%                 tano=tangenta_t+pi;
%             end
            tano = wrapToPi(tano);
            
            
        end
        
        
        if(d<0.1)
            gg=1;
        elseif(gg&& d>.1)
            
            gg=2;
        end
        
    end
     
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Okolje = DefinirajOkolje
    global Okolje
    % basic elemrnt is line segment give nby two points: x1 y1 x2 y2
    Okolje=[ 0 0 3 0;... % environment
             3 0 3 3;...
             3 3 0 3;...
             0 3 0 0;....
             0.5 1 2 1;.... % obstacle
             1.5 2 2.5 2;....
           1.5 2.5  1.5 2 ;
             ];
 
       for i=1:size(Okolje,1)
           line(Okolje(i,[1,3]),Okolje(i,[2,4]),'LineWidth',3,'Color','k'); 
       end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [q]=SimulacijaDejanskegaRobora(q,v,w) % simulate robot motion
  global R L Ts
  flag_sum = 0; % add noise (set 0 or 1)
  flag_pri = 0; % add modeling error (set 0 or 1) 
  
  %nimamo toènega podatka o dimenzijah robota R in L, ki so v resnici
  Rt=R+0.003*flag_pri;
  Lt=L-0.004*flag_pri;
  % dodatno se dejansko kolo zaradi majnih zdrsov, neravnin, itd... vrti z
  % nekim dodanim šumom
  SIGMA_W=0.3;
  % kotne hitrosti koles glede na vhode z dodanim šumom
  wR=1/(R)*(v+w*(L)/2)+SIGMA_W*randn(1,1)*flag_sum;
  wL=1/(R)*(v-w*(L)/2)+SIGMA_W*randn(1,1)*flag_sum;
  
  % dejanska hitrost robota je torej
    vt=Rt/2*(wR+wL);
    wt=Rt/Lt*(wR-wL);
   
 	q(1)=q(1) + Ts*vt*cos( q(3) + Ts*wt/2 );
	q(2)=q(2) + Ts*vt*sin( q(3) + Ts*wt/2 );
    q(3)=q(3) + Ts*wt;
    
    q(3)=wrapToPi(q(3));    % popravi kot q(3)    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function DrawRobot(Xr,tip)  % draw robot shape
global hhh

P=[1 4 4 -4 -4 -1 -1 -4 -4 4 4 1 1;...  % oblika robota
   3 3 4 4 3 3 -3 -3 -4 -4 -3 -3 3]*.01;         

theta = Xr(3); 
R=[cos(theta) -sin(theta); sin(theta) cos(theta)];
T=repmat([Xr(1);Xr(2)],1,size(P,2)) ;

% toèke obrisa robota transliramo in rotiramo
P=R*P+T; 
set(hhh(tip),'XData',P(1,:),'YData',P(2,:))   % izris dejanskega robota
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function InitGrafic()
global hhh
figure(10) ;clf ; 
hold on;
zoom on;
title('Robot with differential drive');xlabel('x (m)');ylabel('y (m)');
axis([-.5,3.5,-.5,3.5]); 
axis equal

% hhh(1)=plot(0,0,'b','erasemode','xor') ;     % dejanski robot 
% %hhh(2)=plot(0,0,'g','erasemode','xor') ;     % robot z odometrijo 
% 
% hhh(3)=plot(0,0,'--b','erasemode','none') ;  % dejanska pot
% %hhh(4)=plot(0,0,'--g','erasemode','none') ;  % pot dobljena z odometrijo
% hhh(5)=plot(0,0,'*r','erasemode','none') ;   % referenèna pot 


hhh(1)=plot(0,0,'b') ;     % dejanski robot 
hhh(3)=plot(0,0,'--b') ;  % dejanska pot
hhh(5)=plot(0,0,'*r') ;   % referenèna pot 



%hhh(6)=line([0,1],[0 1],'LineWidth',2,'Color','k') ;   % okolje 



%legend('rob','robOdo','pot','odo.','ref.','okolje')
legend('rob.','path','ref.')

hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




