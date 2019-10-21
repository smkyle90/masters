for i=1:N
    for ii=1:N
        
           
        n1=sqrt((RMapX(i,ii,1,1).^2+RMapY(i,ii,1,1).^2));
        n2=sqrt((dRMapXx(i,ii,1,1).^2+dRMapXy(i,ii,1,1).^2));
        n3=sqrt((dRMapYx(i,ii,1,1).^2+dRMapYy(i,ii,1,1).^2));
        
        % Normalize the map
        
        if RMapX(i,ii,1,1)==0
            RMapX(i,ii,1,1)=0;
        else
        RMapX(i,ii,1,1)=RMapX(i,ii,1,1)./n1;
        end
        
        if RMapY(i,ii,1,1)==0
            RMapY(i,ii,1,1)=0;
         else
        RMapY(i,ii,1,1)=RMapY(i,ii,1,1)./n1;
         end
         
        if dRMapXx(i,ii,1,1)==0
            dRMapXx(i,ii,1,1)=0;
        else 
        dRMapXx(i,ii,1,1)=dRMapXx(i,ii,1,1)./n2;
        end
       if dRMapXy(i,ii,1,1)==0
            dRMapXy(i,ii,1,1)=0;
        else
        dRMapXy(i,ii,1,1)=dRMapXy(i,ii,1,1)./n2;
       end
        if dRMapYy(i,ii,1,1)==0
            dRMapYy(i,ii,1,1)=0;
        else
        dRMapYy(i,ii,1,1)=dRMapYy(i,ii,1,1)./n3;
        end
        if dRMapYx(i,ii,1,1)==0
            dRMapYx(i,ii,1,1)=0;
        else
        dRMapYx(i,ii,1,1)=dRMapYx(i,ii,1,1)./n3;
        end
        
        n1=sqrt((AMapX(i,ii,1,1).^2+AMapY(i,ii,1,1).^2));
        n2=sqrt((dAMapXx(i,ii,1,1).^2+dAMapXy(i,ii,1,1).^2));
        n3=sqrt((dAMapYx(i,ii,1,1).^2+dAMapYy(i,ii,1,1).^2));
        
                if AMapX(i,ii,1,1)==0
            AMapX(i,ii,1,1)=0;
        else
        AMapX(i,ii,1,1)=AMapX(i,ii,1,1)./n1;
        end
         if AMapY(i,ii,1,1)==0
            AMapY(i,ii,1,1)=0;
         else
        AMapY(i,ii,1,1)=AMapY(i,ii,1,1)./n1;
         end
         
        if dAMapXx(i,ii,1,1)==0
            dAMapXx(i,ii,1,1)=0;
        else 
        dAMapXx(i,ii,1,1)=dAMapXx(i,ii,1,1)./n2;
        end
       if dAMapXy(i,ii,1,1)==0
            dAMapXy(i,ii,1,1)=0;
        else
        dAMapXy(i,ii,1,1)=dAMapXy(i,ii,1,1)./n2;
       end
        if dAMapYy(i,ii,1,1)==0
            dAMapYy(i,ii,1,1)=0;
        else
        dAMapYy(i,ii,1,1)=dAMapYy(i,ii,1,1)./n3;
        end
        if dAMapYx(i,ii,1,1)==0
            dAMapYx(i,ii,1,1)=0;
        else
        dAMapYx(i,ii,1,1)=dAMapYx(i,ii,1,1)./n3;
        end
    end
    end