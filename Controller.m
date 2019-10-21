function [u, omega, phi] = Controller(Ku,Kw,Kp,q,Xf,MapX,MapY,dMapYx,dMapYy,dMapXx,dMapXy,D)

% Ku and Kw are controller gains.
% q current state
% Final location



% Now we have the map for this time-step. We can use
% them to calculate control inputs

% Calculate distance to final point for velocity input

RAD=norm(q(1:2)-Xf(1:2));
% Calculate angle for omega input

phi=atan2(MapY,MapX);

% Expressions from panagou paper. Derivatives in x and y direction.

u=Ku*tanh(RAD);

phidot = [(dMapYx*cos(q(3))+dMapYy*sin(q(3)))*MapX-(dMapXx*cos(q(3))+dMapXy*sin(q(3)))*MapY]*u;

omega=-Kw*(q(3)-phi)+Kp*phidot;
