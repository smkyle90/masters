function [MapX, MapY, dMapXx, dMapXy, dMapYx,dMapYy] = VehicleMap(RMapX, RMapY, dRMapXx, dRMapXy, dRMapYx,dRMapYy, SIGMA, AMapX, AMapY, dAMapXx, dAMapXy, dAMapYx,dAMapYy)

        MapX=RMapX+1*(1-SIGMA).*AMapX;
        MapY=RMapY+1*(1-SIGMA).*AMapY;
        
        dMapXx=dRMapXx+1*(1-SIGMA).*dAMapXx;
        dMapXy=dRMapXy+1*(1-SIGMA).*dAMapXy;
        dMapYy=dRMapYy+1*(1-SIGMA).*dAMapYy;
        dMapYx=dRMapYx+1*(1-SIGMA).*dAMapYx;
        
        n1=sqrt((MapX.^2+MapY.^2));
        n2=sqrt((dMapXx.^2+dMapXy.^2));
        n3=sqrt((dMapYx.^2+dMapYy.^2));
        
        % Normalize the map
        
        if MapX==0
            MapX=0;
        else
        MapX=MapX./n1;
        end
         if MapY==0
            MapY=0;
         else
        MapY=MapY./n1;
         end
         
        if dMapXx==0
            dMapXx=0;
        else 
        dMapXx=dMapXx./n2;
        end
       if dMapXy==0
            dMapXy=0;
        else
        dMapXy=dMapXy./n2;
       end
        if dMapYy==0
            dMapYy=0;
        else
        dMapYy=dMapYy./n3;
        end
        if dMapYx==0
            dMapYx=0;
        else
        dMapYx=dMapYx./n3;
        end