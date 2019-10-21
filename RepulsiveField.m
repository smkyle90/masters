function [RMapX, RMapY, dRMapXx, dRMapXy, dRMapYx, dRMapYy, SIGMA]=RepulsiveField(k,P,Xf,dx,Amin,RMapX,RMapY,dRMapXx,dRMapXy,dRMapYx,dRMapYy,SIGMA,dc,dr,dm,NO,N)

% This function generates a repulsive vector field about a point P, that
% points towards Xf, for agent k.
% 
% j is the index for the obstacle, or vehicle you are generating field about
% at point P.
%
% The vector field takes the location of an agent, or point in space and
% calculates change in vector field due to repulsive vector fields. It also
% stores the sigma value i.e. to blend attractive and repulsive fields.


% For coordinate calculate where it lives in the grid so we can export the
% the value to the correct "grid" location.

% Calculate "area to generate map" to save time computationally.
% ONly need to consider a distance dc around point P. 
% The function figures out where
% this index lives and stores appropriately (and accordingly).

Xmin = P(1) - dc;
Ymin = P(2) - dc;

Xmax = P(1) + dc;
Ymax = P(2) + dc;

Xindexmin = floor((Xmin-Amin(1))/dx);
Yindexmin = floor((Ymin-Amin(2))/dx);
Xindexmax = floor((Xmax-Amin(1))/dx);
Yindexmax = floor((Ymax-Amin(2))/dx);

if Xindexmin <= 0
    Xindexmin = 1; end
if Yindexmin <= 0
    Yindexmin = 1; end

if Xindexmax > N
    Xindexmin = N; end
if Yindexmin > N
    Yindexmax = N; end

a1=-2./(dr-dc).^3;
a2=3.*(dr+dc)./(dr-dc).^3;
a3=-6.*(dr.*dc)./(dr-dc).^3;
a4=dc.^2.*(3.*dr-dc)./(dr-dc).^3;

for i=Xindexmin:Xindexmax % This probably does not need to be as comprehensive, as dr is small, so we know contribution is spatially small.
    for ii=Yindexmin:Yindexmax
        
        POS =[Amin(1)+i*dx Amin(2)+ii*dx]; % x and y position in space
        
        phi=atan2(-P(2)+Xf(2,k),-P(1)+Xf(1,k))+pi;
        p_i=[cos(phi) sin(phi)];
        
        % Repulsive Vector Field
        
        delta_r = POS'-P; % Find the distance from the obstable to that point in space
        test = p_i*delta_r; % Condition on which vector field to apply
        
        % Calculate sigma scaling factor
        
        d = norm(delta_r);
        
        if d > dc
            sigma = 0;
        elseif dr <= d && d <= dc
            sigma = a1*d^3 + a2*d^2 + a3*d + a4;
        elseif d <= dr
            sigma = 1;
        end
        
        if norm(delta_r) < dr
            rx=0;
            ry=0;
            drxx=0;
            drxy=0;
            dryx=0;
            dryy=0;
        elseif test < 0 && norm(delta_r) >= dr % lambda=0
            rx=(-p_i(1)*delta_r(1)^2-p_i(1)*delta_r(2)^2)/norm(delta_r);
            ry=(-p_i(2)*delta_r(1)^2-p_i(2)*delta_r(1)^2)/norm(delta_r);
            drxx=-2*delta_r(1)*p_i(1)/norm(delta_r)-rx*delta_r(1)/norm(delta_r)^3;
            drxy=-2*delta_r(2)*p_i(1)/norm(delta_r)-rx*delta_r(2)/norm(delta_r)^3;
            dryx=-2*delta_r(1)*p_i(2)/norm(delta_r)-ry*delta_r(1)/norm(delta_r)^3;
            dryy=-2*delta_r(2)*p_i(2)/norm(delta_r)-ry*delta_r(2)/norm(delta_r)^3;
        elseif test >= 0 && norm(delta_r) >= dr % lambda=1
            rx=(p_i(2)*delta_r(1)*delta_r(2)-p_i(1)*delta_r(2)^2)/norm(delta_r) ;
            ry=(p_i(1)*delta_r(1)*delta_r(2)-p_i(2)*delta_r(1)^2)/norm(delta_r) ;
            drxx=p_i(2)*delta_r(2)/norm(delta_r)-rx*delta_r(1)/norm(delta_r)^3;
            drxy=(-2*p_i(1)*delta_r(2)+p_i(2)*delta_r(1))/norm(delta_r)-rx*delta_r(2)/norm(delta_r)^3;
            dryx=(-2*p_i(1)*delta_r(2)+p_i(1)*delta_r(2))/norm(delta_r)-ry*delta_r(1)/norm(delta_r)^3;
            dryy=p_i(1)*delta_r(1)/norm(delta_r)-ry*delta_r(2)/norm(delta_r)^3;
        else
            rx=0;
            ry=0;
            drxx=0;
            drxy=0;
            dryx=0;
            dryy=0;
        end
        
                % Store sigma to adjust the attractive map (there is a
        % sigma mesh from repulsive vector fields).
        if sigma > SIGMA(i,ii)
            SIGMA(i,ii)=sigma;
        else
            SIGMA(i,ii)=SIGMA(i,ii);
        end
        
        % Add the repulsive contribution scaled by sigma
        RMapX(i,ii)=RMapX(i,ii)+1*SIGMA(i,ii)*rx;
        RMapY(i,ii)=RMapY(i,ii)+1*SIGMA(i,ii)*ry;
        
        dRMapXx(i,ii)=dRMapXx(i,ii)+1*SIGMA(i,ii)*drxx;
        dRMapXy(i,ii)=dRMapXy(i,ii)+1*SIGMA(i,ii)*drxy;
        dRMapYy(i,ii)=dRMapYy(i,ii)+1*SIGMA(i,ii)*dryy;
        dRMapYx(i,ii)=dRMapYx(i,ii)+1*SIGMA(i,ii)*dryx;
   
end
end



