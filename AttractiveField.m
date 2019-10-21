function [AMapX, AMapY, dAMapXx, dAMapXy, dAMapYx, dAMapYy]=AttractiveField(k,X,Xf,dx,Amin,N,AMapX, AMapY, dAMapXx, dAMapXy, dAMapYx, dAMapYy)

% This function generates an attractive vector field about a point Xf. This
% is only for "destinations" and is static on calculations.
% The vector field is only dependent on the initial position of agent k.
% This function outputs attractive vector fields.

% For coordinate calculate where it lives in the grid so we can export the
% the value to the correct "grid" location.


for i=1:N
    for ii=1:N
            POS =[Amin(1)+i*dx Amin(2)+ii*dx]; % x and y position in space
            
            % Only depends on agents k's initial & final position. This
            % does not change with time, but is scaled by (1-sigma) at each
            % time step.
            
            % Attractive Vector Field
            delta_a = POS' - Xf(1:2,k);
            
            phi2=atan2(-X(2,k)+Xf(2,k),-X(1,k)+Xf(1,k));
            pi2=[cos(phi2) sin(phi2)];
            
            if delta_a == 0 % lambda = 2;
                ax=0;
                ay=0;
                dax=0;
                day=0;
            else
                ax=(pi2(1)*(delta_a(1)^2-delta_a(2)^2)+pi2(2)*2*delta_a(1)*delta_a(2))/norm(delta_a);
                ay=(pi2(2)*(-delta_a(1)^2+delta_a(2)^2)+pi2(1)*2*delta_a(1)*delta_a(2))/norm(delta_a);
                
                % Derivative of Vector Field
                daxx=2*(pi2(1)*delta_a(1)+pi2(2)*delta_a(2))/norm(delta_a)-(delta_a(1)*ax)/norm(delta_a)^3;
                daxy=2*(pi2(2)*2*delta_a(1)-pi2(1)*delta_a(2))/norm(delta_a)-(delta_a(2)*ax)/norm(delta_a)^3;
                dayx=2*(-pi2(2)*delta_a(1)+pi2(1)*delta_a(2))/norm(delta_a)-(delta_a(1)*ay)/norm(delta_a)^3;
                dayy=2*(pi2(1)*2*delta_a(1)+pi2(2)*delta_a(2))/norm(delta_a)-(delta_a(2)*ay)/norm(delta_a)^3;
            end
            
            % Vector Field
            AMapX(i,ii)=ax;
            AMapY(i,ii)=ay;
            % Derivative of Vector Field
            dAMapXx(i,ii)=daxx;
            dAMapYx(i,ii)=dayx;
            dAMapXy(i,ii)=daxy;
            dAMapYy(i,ii)=dayy;
            
        end
    end



