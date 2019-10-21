function X = DynamicObstacles(Nd,alpha,D,T,delta)

Nt=T/delta; % Number of Data points
t=linspace(0,T-delta,Nt); % Time Vector

X = zeros(2*Nd,Nt);
for i=1:Nd
    if mod(i,3)==1
        X(2*i-1,:)=D-(D-mod(i,2)*D-(-1)^(i)*(D/(T))*t.*ones(1,Nt)+rand(1,Nt)*(D/100));
        X(2*i,:)=(mod(i,2)*D+(-1)^(i)*(D/(T))*t.*ones(1,Nt))+rand(1,Nt)*(D/100);
    else
        X(2*i-1,:)=D/2+(-1)^(i)*cos(i/Nd*alpha*t-i/2*pi)*D/3+rand(1,Nt)*(D/100);
        X(2*i,:)=D/2-(-1)^(i)*sin(-i/Nd*alpha*t-i/2*pi)*D/3+rand(1,Nt)*(D/100);
    end
end