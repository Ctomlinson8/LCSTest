%Linear Control systems 
%Control System Design using Root Locus

num = [ 0 0.1 16 650];          %numurator of orginal plant
den = [ 1 179 7978 7800];       %denomorator of orginal plan
den_s = [1 179 7978 7800 0];    %denominator multiplied by s 
G = tf(num,den);                %transfer function of the plant
y = feedback(G,1);              %feed back response of plant

figure(1);
rlocus(G);                      %Root locus of the transfer function

figure(2);
step(y);                        %step response of the tansfer function

figure(3);
impulse(y);                     %impluse response of transfer function

gs = tf(num,den_s);             %transfer function of denomintor multiplies by s            
figure(4);
step(gs);                       %unit ramp responce 


stepinfo(y)                     %step response information, helps determine the proper PID and lead lag
                                %responce to create a system that meet the inital conditions

P = pole(G)                     %poles and zeros to help determine stability 
Z = zero(G)


kp = 150;                       %Kp value
ki = 1;                         %Ki value
kd = 9;                         %Kd value
K =pid(kp,ki,kd);               %PID function


num_3 = [1 78];                 %zero of the lag compensator is equal to 78
den_3 = [1 1];                  %pole of the lag is set to equal a pole less then 78
lag = tf(num_3,den_3);          %Lag transfer function

num_4 = [1 1];                  %zero of the lead compensator is equal to 1 same as pole of lag
den_4 = [1 4];                  %pole of the lead is set to equal a value less then the next real axis pole which is 4
lead = tf(num_4,den_4);         %Lead transfer function


kg = tf(K*G*lead*lag);                   %Transfer function for the plant and PID in series along with lead/lag comp.
y2 =feedback(K*G*lead*lag,1);            %Feedback function of plant and PID in series and lead lag 


figure(5);                      %root locus of plant and PID controller in series
rlocus(kg);

figure(6);                      %step responce response 
step(y2);


stepinfo(y2)                    %step response information

P = pole(kg)                    %poles and zeros to help determine stability 
Z = zero(kg)

num_f = [0.9 230.1 20880 761700 8348000 7658000 50700];
den_f= [1.9 414.1 29760 810100 8419000 7689000 50700 0];

yf = tf(num_f,den_f);             %transfer function of denomintor multiplies by s            
figure(7);
step(yf);                       %unit ramp responce 

