% Vector Field Modelling

clear variables;

% Simulation Parameters
T=30; % Simulation Time
delta=0.05; % Step Size

% These are dependent on  T and delta.
Nt=T/delta; % Number of Data points
t=linspace(0,T-delta,Nt); % Time Vector

% Arena Disceretization -- to determine vector field

D = 10; % Arena Size. The arena is size D, but we need to calculate a vector field 3D x 3D about D/2 (get a full picture). Let this represent Dx10 in m.
Dactual = 10; % ACTUAL ARENA size.
scale = Dactual/D; % Scaling factor. Arena size, in [m] is D*scale.
dx = 0.1; % size of cell

% Arena Centre
Ac = [0;0]; % Centred at the middle of the first cell

% Arena Limits

Amin = [Ac(1)-2*D;Ac(2)-2*D]; % X and Y coord of arena minimum (bottom left corner) Corresponds to N=1 when storing information.
Amax = [Ac(1)+2*D;Ac(2)+2*D]; % X and Y coord of arena minimum (top right corner). Corresponds to N=N when storing information.

N = max(Amax-Amin)/dx; % number of cells in X and Y direction

%%
% Agents Initial Conditions
NA = 2; % Number of agents
nz = 1; % Noise value. The bigger, the more noise.

[X,Xf,Crow] = AgentPositions(NA,D,nz); % Initial and final coordinates of each agent

% % X = [-5 5;-5 5; pi/4 -3*pi/4];
% Xf = [5 -5 ;5 -5;  0 0];
% X = [-5 5 ;-5 5 ; pi/4 -3*pi/4];
% Crow = [sqrt(2)*10;sqrt(2)*10]';

%% Agent Properties

% Vehicle Dimensions
maxL = 2; % Max Vehicle Length [m]
L = maxL*ones(1,NA)/scale; % Vehicle Lengths. The max is (form basis of repulsive vector field values dc and dr). Width is always 1 m.
% Vector Fieldc Inf0
drV = 1.5*L; % The minimum distance a vehicle can come to an agent (this is the vehicle dimension with a factor of safety.
dcV = 6*drV; % The vector field is X m further than the vehicle's minimum passing distance. ALTER THIS VALUE! This value is obsolete.

% Communication & Sensing
    
MaxRc = 20; % Maximum communication distance [m] for each agent.
MaxRs = 10; % Maximum sensing distance [m] for each agent.
Rc = MaxRc*ones(1,NA)/scale; % Radius of Communication for each agent. Each interaction is limited my the min of i,j's Rc because you only 'receive' information. Ensures minimum is 5 m.
Rs = MaxRs*ones(1,NA)/scale; % Radius of Sensing for each agent, for dynamic obstacles, or rogue agents. Ensures minimum is 1 m.
% Rogue Agents
    rogue=0; % Nust be less than or equal to NA
    if rogue > 0 && rogue <= NA
        for i=1:rogue
            Rc(NA-(i-1))=0;
        end
    elseif rogue == 0 && rogue <= NA
        sprintf('No rogue agents.')
    else
        sprintf('Number of rogue agents must be less than or equal to number of agents.')
    end
    
    %%
    % Dynamic Obstacles [these will be other agents]
    Nd = 1; % Number of obstacles
    alpha = 0.3; % angular rate of odd obstacles
    
    Pd = DynamicObstacles(Nd,alpha,D,T,delta); % Dynamic Obstacles from function DynamicObstacles.m
    
    % Dynamic Obstacle Information (similar to a rogue agent)
    
    MaxRadD = 10; % Maximum Dimension [m] of a static obstacle
    BufferD = 10; % Size of vector field produced by dynamic obstacle.
    
    drD = MaxRadD*rand(1,Nd)/scale;
    dcD = 10*drD+BufferD/scale;
    
    % Static Obstacles
    NO = 2; % Number of obstacles
    P = StaticObstacles(NO,D,Ac); % Obstacle position from function StaticObstacles.m
    % Static Obstacle Information
    
    MaxRadS = 10; % Maximum Dimension [m] of a static obstacle
    BufferS = 15; % Size of vector [m] field produced by static obstacle. i.e. obstacle contributes to vector field this distance
    
    alpha1=5;
    % for alpha1=2:2:10
    drS = 1;%MaxRadS*ones(1,NO)/scale;
    dcS = alpha1*drS; % Determined in loop %10*drS+BufferS/scale; % This might be the most critical thing.
    
    
    % P = [-2 -2 -2 -2 -2 2 2 2 2 2 ; 2 1 0 -1 -2 -2 -1 0 1 2]; % U NA=10 need to connect obstacles into a shape and push interior obstacle fields to outer obstacle.
    % P = [-1 1 1 -1 ; 1 1 -1 -1]; % Square NA=4;
    P=[0;0];
    
    % Determine Vehicles Maps
    
    % Static Map includes attractive vector field for destination & repulsive
    % field for known obstacles and objects.
    
    % Generate Potential Function or "System Map"
    % Initialize Vector Fields (including derivative fields)
    
    % Total
    MapX = zeros(N,N,Nt,NA); % Values in X direction
    MapY = zeros(N,N,Nt,NA); % Values in Y direction
    dMapXx = zeros(N,N,Nt,NA); % Values in dX direction
    dMapYy = zeros(N,N,Nt,NA); % Values in dY direction
    dMapXy = zeros(N,N,Nt,NA); % Values in dX direction
    dMapYx = zeros(N,N,Nt,NA); % Values in dY direction
    % Attractive
    AMapX = zeros(N,N,Nt,NA); % Values in X direction
    AMapY = zeros(N,N,Nt,NA); % Values in Y direction
    dAMapXx = zeros(N,N,Nt,NA); % Values in dX direction
    dAMapYy = zeros(N,N,Nt,NA); % Values in dY direction
    dAMapXy = zeros(N,N,Nt,NA); % Values in dX direction
    dAMapYx = zeros(N,N,Nt,NA); % Values in dY direction
    % Repulsive
    RMapX = zeros(N,N,Nt,NA); % Values in X direction
    RMapY = zeros(N,N,Nt,NA); % Values in Y direction
    dRMapXx = zeros(N,N,Nt,NA); % Values in dX direction
    dRMapYy = zeros(N,N,Nt,NA); % Values in dY direction
    dRMapXy = zeros(N,N,Nt,NA); % Values in dX direction
    dRMapYx = zeros(N,N,Nt,NA); % Values in dY direction
    % Coordinates & Sigma Value
    Xp=zeros(N,N,Nt,NA);
    Yp=zeros(N,N,Nt,NA);
    SIGMA = zeros(N,N,Nt,NA);
    
    
    % Static Map -- this is the prior mapping phase and finds the map, and
    % controls required for the agent to move from X0 to Xf, while considering
    % known obstacles. This is functional, and do not need to do anything.
    
    % Generate the Coordinate Mesh. This is used for Plotting the Vector
    % Fields.
    [Xp,Yp]=ArenaMesh(dx,Amin,Amax,N);
    for k=1:NA
        % Generate the Attractive Vector Fields for each agent based on final
        % destinations
        [AMapX(:,:,1,k), AMapY(:,:,1,k), dAMapXx(:,:,1,k), dAMapXy(:,:,1,k), dAMapYx(:,:,1,k), dAMapYy(:,:,1,k)]=AttractiveField(k,X,Xf,dx,Amin,N,AMapX(:,:,1,k),AMapY(:,:,1,k),dAMapXx(:,:,1,k),dAMapXy(:,:,1,k),dAMapYx(:,:,1,k),dAMapYy(:,:,1,k));
        % Generate the KNOWN Repulsive Vector Field for each agent based on
        % static objects.
        for j=1:NO
            [RMapX(:,:,1,k), RMapY(:,:,1,k), dRMapXx(:,:,1,k), dRMapXy(:,:,1,k), dRMapYx(:,:,1,k), dRMapYy(:,:,1,k), SIGMA(:,:,1,k)]=RepulsiveField(k,P(:,j),Xf,dx,Amin,RMapX(:,:,1,k),RMapY(:,:,1,k),dRMapXx(:,:,1,k),dRMapXy(:,:,1,k),dRMapYx(:,:,1,k),dRMapYy(:,:,1,k),SIGMA(:,:,1,k),dcS(j),drS(j),0,NO,N);
        end
        
    end
    
    for m=2:Nt
        for k=1:NA
            
            RMapX(:,:,m,k) = RMapX(:,:,1,k);
            RMapY(:,:,m,k) = RMapY(:,:,1,k);
            AMapX(:,:,m,k) = AMapX(:,:,1,k);
            AMapY(:,:,m,k) = AMapY(:,:,1,k);
            dAMapXx(:,:,m,k) = dAMapXx(:,:,1,k);
            dAMapXy(:,:,m,k) = dAMapXy(:,:,1,k);
            dAMapYx(:,:,m,k) = dAMapYx(:,:,1,k);
            dAMapYy(:,:,m,k) = dAMapYy(:,:,1,k);
            dRMapXx(:,:,m,k) = dRMapXx(:,:,1,k);
            dRMapXy(:,:,m,k) = dRMapXy(:,:,1,k);
            dRMapYx(:,:,m,k) = dRMapYx(:,:,1,k);
            dRMapYy(:,:,m,k) = dRMapYy(:,:,1,k);
            SIGMA(:,:,m,k) = SIGMA(:,:,1,k);
        end
    end
    
    % DO this for every obstacle, or agent.
    for i=1:N
        for ii=1:N
            POS =[Amin(1)+i*dx Amin(2)+ii*dx]; % x and y position in space
            for k=1:NA
                for kk=1:NO
                    if norm(POS'-P(:,kk)) < drS(kk) %R(kk)
                        RMapX(i,ii,:,k)=0;
                        RMapY(i,ii,:,k)=0;
                        AMapX(i,ii,:,k)=0;
                        AMapY(i,ii,:,k)=0;
                        dAMapXx(i,ii,:,k)=0;
                        dAMapXy(i,ii,:,k)=0;
                        dAMapYx(i,ii,:,k)=0;
                        dAMapYy(i,ii,:,k)=0;
                        dRMapXx(i,ii,:,k)=0;
                        dRMapXy(i,ii,:,k)=0;
                        dRMapYx(i,ii,:,k)=0;
                        dRMapYy(i,ii,:,k)=0;
                    end
                end
            end
        end
    end
    % Save the maps to run multiple trials
    SRMapX = RMapX;
    SRMapY = RMapY;
    SAMapX = AMapX;
    SAMapY = AMapY;
    SdAMapXx = dAMapXx;
    SdAMapXy = dAMapXy;
    SdAMapYx = dAMapYx;
    SdAMapYy = dAMapYy;
    SdRMapXx = dRMapXx;
    SdRMapXy = dRMapXy;
    SdRMapYx = dRMapYx;
    SdRMapYy = dRMapYy;
    SSIGMA = SIGMA;
    %%
    % Dynamic Simulation & Control
    clear Q F QF i ii k kk
    % Controller Gains
    % Ku=1.5;%1*(1+exp(-NA+1)); % After some sort of process.
    % Kw=.05;%0.1;%0.1;
    Kp=1;
    
    KU=1; % Number of velocity controller Data points you want to samples
    KW=1; % Number of omega controller Data points you want to samples
    
    Ku=linspace(1,1.5,KU);
    Kw=linspace(0.1,.1,KW);
    
    TOTALDISTANCE=zeros(NA,KU,KW);
    Agent2Agent=zeros(NA,NA,Nt);
    mindistance=D*ones(1,NA);
    
    for WW=1:KW
        
        for UU=1:KU
            
            % Dynamic Map & Simulation
            
            % Initialize Preamble
            q0=X;
            Q=zeros(3,NA,Nt);
            Q(:,:,1)=q0;
            q=q0;
            END=zeros(1,NA); % End condition
            clear PHI OMEGA U qdot
            
            % To run this section, you have to run all previous sections. You can run
            % this section multiple times, but if you change anty of the system
            % paramers or properties, you must re-initialize the system.
            % This resets the maps the a priori versions.
            RMapX = SRMapX;
            RMapY = SRMapY;
            AMapX = SAMapX;
            AMapY = SAMapY;
            dAMapXx = SdAMapXx;
            dAMapXy = SdAMapXy;
            dAMapYx = SdAMapYx;
            dAMapYy = SdAMapYy;
            dRMapXx = SdRMapXx;
            dRMapXy = SdRMapXy;
            dRMapYx = SdRMapYx;
            dRMapYy = SdRMapYy;
            SIGMA = SSIGMA;
            U=zeros(NA,Nt);
            OMEGA=zeros(NA,Nt);
            
            for i=2:Nt
               if sum(END)<NA
                newNt=i; % Store i value, as if the simulation broke we can still plot the results
                
                ll=Q(:,:,i-1);
                for j=1:NA % the loop for each vehicle
                    %             if END(j)==0
                    q=ll(:,j);
                    % check agent-to-agent distance (comms) -- repulsive field
                    
                    for k=1:NA
                        if k~=j
                            Agent2Agent(j,k,i)=norm(Q(1:2,j,i-1)-Q(1:2,k,i-1));
                        end
                        
                        if k~=j && Agent2Agent(j,k,i) < mindistance(1,k)
                            mindistance(1,k) = Agent2Agent(j,k,i);
                        end
                        if j~=k && norm(Q(1:2,j,i-1)-Q(1:2,k,i-1)) <= min(Rc(j),Rc(k))
                            
                            [RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j)]=RepulsiveField(j,Q(1:2,k,i-1),Xf,dx,Amin,RMapX(:,:,i-1,j),RMapY(:,:,i-1,j),dRMapXx(:,:,i-1,j),dRMapXy(:,:,i-1,j),dRMapYx(:,:,i-1,j),dRMapYy(:,:,i-1,j),SIGMA(:,:,i-1,j),dcV(k),drV(k),0,NA,N);
                        elseif j~=k && norm(Q(1:2,j,i-1)-Q(1:2,k,i-1)) <= Rs(j) % check agent-to-agent (uncommunicative) agent distance (sensing)
                            [RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j)]=RepulsiveField(j,Q(1:2,k,i-1),Xf,dx,Amin,RMapX(:,:,i-1,j),RMapY(:,:,i-1,j),dRMapXx(:,:,i-1,j),dRMapXy(:,:,i-1,j),dRMapYx(:,:,i-1,j),dRMapYy(:,:,i-1,j),SIGMA(:,:,i-1,j),dcV(k),drV(k),0,NA,N);
                            
                            
                        end
                    end
                    
                    % check agent-to-agent, obstacle, or uncommunicative agent distance (sensing) -- repulsive field
                    
                    %         for k=1:NA
                    %             if j~=k && norm(Q(1:2,j,i-1)-Q(1:2,k,i-1)) <= Rs(j)
                    %                 [RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j)]=RepulsiveField(j,Q(1:2,k,i-1),Xf,dx,Amin,RMapX(:,:,i-1,j),RMapY(:,:,i-1,j),dRMapXx(:,:,i-1,j),dRMapXy(:,:,i-1,j),dRMapYx(:,:,i-1,j),dRMapYy(:,:,i-1,j),SIGMA(:,:,i-1,j),dcV(k),drV(k),0,NA,N);
                    %             end
                    %         end
                    
                    for k=1:NO
                        if norm(Q(1:2,j,i-1)-P(:,k)) <= dcS(k)
                            % %                         [RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j)]=RepulsiveField(j,P(:,k),Xf,dx,Amin,RMapX(:,:,i-1,j),RMapY(:,:,i-1,j),dRMapXx(:,:,i-1,j),dRMapXy(:,:,i-1,j),dRMapYx(:,:,i-1,j),dRMapYy(:,:,i-1,j),SIGMA(:,:,i-1,j),dcS(k),drS(k),0,NA,N);
                            %
                            %
                            [RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j)] = sigmaCheck(P(:,k),dx,Amin,dcS(k),drS(k),N,RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j));
                            %
                        end
                    end
                    
                    if Nd >= 1
                        for k=1:Nd
                            if norm(Q(1:2,j,i-1)-Pd((2*k-1:2*k),1)) <= Rs(j)
                                [RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j)]=RepulsiveField(j,Pd((2*k-1:2*k),i),Xf,dx,Amin,RMapX(:,:,i-1,j),RMapY(:,:,i-1,j),dRMapXx(:,:,i-1,j),dRMapXy(:,:,i-1,j),dRMapYx(:,:,i-1,j),dRMapYy(:,:,i-1,j),SIGMA(:,:,i-1,j),Rs(j),drS(k),0,NA,N);
                            end
                        end
                    end
                    % Calculate the Vehicles Actual & Derivative Map [ Put this in a
                    % function
                    
                    
                    [MapX(:,:,i-1,j),MapY(:,:,i-1,j),dMapXx(:,:,i-1,j),dMapXy(:,:,i-1,j),dMapYx(:,:,i-1,j),dMapYy(:,:,i-1,j)]=VehicleMap(RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j),dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j), AMapX(:,:,i-1,j), AMapY(:,:,i-1,j), dAMapXx(:,:,i-1,j), dAMapXy(:,:,i-1,j), dAMapYx(:,:,i-1,j),dAMapYy(:,:,i-1,j));
                    
                    
                    % Find Vehicle in ArenaMesh
                    
                    [Qx,Qy]=findVehicle(Amin,q(1:2),dx);
                    
                    % Controller
                    if norm(Q(1:2,j,i-1)-Xf(1:2,j)) < 2*D/Dactual
                        u=0;
                        omega=0;
                        END(j)=1;
                        sprintf('Agent %d has reached their target',j)
                    else
                        [u,omega,phi] = Controller(Ku(UU),Kw(WW),Kp,q,Xf(:,j),MapX(Qx,Qy,i-1,j),MapY(Qx,Qy,i-1,j),dMapYx(Qx,Qy,i-1,j),dMapYy(Qx,Qy,i-1,j),dMapXx(Qx,Qy,i-1,j),dMapXy(Qx,Qy,i-1,j),D);
                    end
                    
                    % Input Controls to model
                    U(j,i)=u;
                    OMEGA(j,i)=omega;
                    PHI(j,i)=phi;
                    qdot=[cos(q(3)) 0 ;
                        sin(q(3)) 0 ;
                        0            1]*[u; omega];
                    q=q+delta*qdot;
                    
                    Q(:,j,i)=q;
                    QF(:,j,UU,WW)=q;
                    
                    delta_d = norm(Q(1:2,j,i)-Q(1:2,j,i-1));
                    TOTALDISTANCE(j,UU,WW)=TOTALDISTANCE(j,UU,WW)+delta_d;
                end
            end
            
        end
    end
 end
    
    for i=1:NA
        Xx(1:newNt,i) = Q(1,i,1:newNt);
        Xy(1:newNt,i) = Q(2,i,1:newNt);
        Xtheta(1:newNt,i) = Q(3,i,1:newNt);
    end
    
    figure(NA+1)
    
    
    
    %% Arena Plot
    hold on
    for kk=newNt:2:newNt
        set(gcf,'color','w');
        
        figure(NA+1)
        
        plot(Xx(1:kk-1,:),Xy(1:kk-1,:),'--')
        hold on
        plot(Xx(kk-1,:),Xy(kk-1,:),'*','Color','k')
        hold on
        %     for i=1:NA
        %         viscircles([Xx(kk-1,i),Xy(kk-1,i)],Rc(i),'color','b','LineStyle','--','LineWidth',1);
        %         hold on
        %         viscircles([Xx(kk-1,i),Xy(kk-1,i)],Rs(i),'color','g','LineStyle',':','LineWidth',1);
        %         value = sprintf('A#%d', i);
        %         text(Xx(kk-1,i)+D/50,Xy(kk-1,i),value)
        %         hold on
        %     end
        plot(P(1,:),P(2,:),'.',X(1,:),X(2,:),'O',Xf(1,:),Xf(2,:),'X')
        hold on
        for j=1:NO
            viscircles(P(:,j)',drS(j),'color','r');
            value = sprintf('O#%d', j);
            text(P(1,j)+D/50,P(2,j),value)
        end
        hold on
        for jj=1:Nd
            viscircles(Pd(2*jj-1:2*jj,kk-1)',drD(jj),'color','r');
            value = sprintf('Dyn # %d', jj);
            text(Pd(2*jj-1,kk)+D/100,Pd(2*jj,kk),value)
        end
        
        axis([-1*D+Ac(1) 1*D+Ac(1) -1*D+Ac(2) 1*D+Ac(2) ]);
        pause(delta/100)
        
        F(kk-1) = getframe(gcf);
        
    end
    
    xlabel('X/10 [m]')
    ylabel('Y/10 [m]')
    box on
    figure(NA+2)
    set(gcf,'color','w');
    
    hold on
    plot(beta,TOTALDISTANCE'./Crow,'X')
    xlabel('\R_c')
    ylabel('Relative Distance, E_i')
    box on
    
    figure(NA+3)
    set(gcf,'color','w');
    
    hold on
    plot(beta,mindistance,'X')
    xlabel('\R_c')
    ylabel('Minimum Distance, D_i_j')
    box on
end
%%
legend('\alpha=2','\alpha=3','\alpha=4','\alpha=5','\alpha=6','\alpha=7','\alpha=8','Location','best')
%%
for i=1:NA
    Xx(1:newNt,i) = Q(1,i,1:newNt);
    Xy(1:newNt,i) = Q(2,i,1:newNt);
    Xtheta(1:newNt,i) = Q(3,i,1:newNt);
    
end

figure(NA+1)
%% Play the movie
movie(figure(), F);
%% Plot Controller Tests
clf
figure(1)
for i=1:WW
    for j=1:KU
        plot3(Kw(i),Ku(j),TOTALDISTANCE(1,j,i)/Crow,'.','color','b')
        hold on
    end
end

xlabel('k_\omega')

ylabel('k_v')
zlabel('Relative Distance Travelled')


%% Error Plots for Tuning Controller
hold on
set(gcf,'color','w');
box on
plot(linspace(0,newNt*delta,newNt),Xx'-Xf(1)*ones(1,newNt),linspace(0,newNt*delta,newNt),Xy'-Xf(2)*ones(1,newNt))
xlim([0 newNt*delta])
ylim([0 max(max(Xx'-Xf(1)*ones(1,newNt)),max(Xy'-Xf(2)*ones(1,newNt)))])
xlabel('Time [s]')
ylabel('Error [m]')
legend('X error','Y error')

%% Static Arena Plot
hold on
set(gcf,'color','w');

figure(NA+1)
plot(Xx(1:kk-1,:),Xy(1:kk-1,:),'--')
hold on
plot(Xx(kk-1,:),Xy(kk-1,:),'*','Color','k')
hold on
for i=1:NA
    viscircles([Xx(kk-1,i),Xy(kk-1,i)],Rc(i),'color','b','LineStyle','--','LineWidth',1);
    hold on
    viscircles([Xx(kk-1,i),Xy(kk-1,i)],Rs(i),'color','g','LineStyle',':','LineWidth',1);
    value = sprintf('A#%d', i);
    text(Xx(kk-1,i)+D/50,Xy(kk-1,i),value)
    hold on
end
plot(P(1,:),P(2,:),'.',X(1,:),X(2,:),'O',Xf(1,:),Xf(2,:),'X')
hold on
for j=1:NO
    viscircles(P(:,j)',drS(j),'color','r');
    value = sprintf('O#%d', j);
    text(P(1,j)+D/50,P(2,j),value)
end
hold on
for jj=1:Nd
    viscircles(Pd(2*jj-1:2*jj,kk-1)',drD(jj),'color','r');
    value = sprintf('Dyn # %d', jj);
    text(Pd(2*jj-1,kk)+D/100,Pd(2*jj,kk),value)
end

axis([-2*D+Ac(1) 2*D+Ac(1) -2*D+Ac(2) 2*D+Ac(2) ]);



%% Dynamic Simulation & Control (Communication & Sensor Testing)
clear Q

RCdelta=1;
Rc = RCdelta*ones(1,NA);
Rs = 0*ones(1,NA);

% Controller Gains

Ku=1.5; % After some sort of process.
Kw=0.05; %0.1;
Kp=2;

% Dynamic Map & Simulation

RADIUS = 5; % Do for x Rc values
SENSOR = 1; % Do for x Rs values

mindistance=D*ones(RADIUS,NA);
NORM=zeros(RADIUS,Nt);
FV=zeros(RADIUS,NA);
%%

Q=zeros(3,NA,Nt,RADIUS,SENSOR);

for SS=1:SENSOR
    for RR=1:RADIUS
        
        clear Q
        
        % Controller Gains
        Ku=1.5;%1*(1+exp(-NA+1)); % After some sort of process.
        Kw=0.05;%0.1;%0.1;
        Kp=2;
        
        % Dynamic Map & Simulation
        
        % Initialize Preamble
        q0=X;
        Q=zeros(3,NA,Nt);
        Q(:,:,1)=q0;
        q=q0;
        END=zeros(1,NA); % End condition
        
        % To run this section, you have to run all previous sections. You can run
        % this section multiple times, but if you change anty of the system
        % paramers or properties, you must re-initialize the system.
        % This resets the maps the a priori versions.
        RMapX = SRMapX;
        RMapY = SRMapY;
        AMapX = SAMapX;
        AMapY = SAMapY;
        dAMapXx = SdAMapXx;
        dAMapXy = SdAMapXy;
        dAMapYx = SdAMapYx;
        dAMapYy = SdAMapYy;
        dRMapXx = SdRMapXx;
        dRMapXy = SdRMapXy;
        dRMapYx = SdRMapYx;
        dRMapYy = SdRMapYy;
        SIGMA = SSIGMA;
        U=zeros(NA,Nt);
        OMEGA=zeros(NA,Nt);
        
        for i=2:Nt
            if sum(END)<NA
                
                newNt=i; % Store i value, as if the simulation broke we can still plot the results
                
                ll=Q(:,:,i-1);
                for j=1:NA % the loop for each vehicle
                    %             if END(j)==0
                    q=ll(:,j);
                    % check agent-to-agent distance (comms) -- repulsive field
                    
                    for k=1:NA
                        if j~=k && norm(Q(1:2,j,i-1)-Q(1:2,k,i-1)) <= min(4+RR*Rc(j),4+RR*Rc(k))
                            
                            [RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j)]=RepulsiveField(j,Q(1:2,k,i-1),Xf,dx,Amin,RMapX(:,:,i-1,j),RMapY(:,:,i-1,j),dRMapXx(:,:,i-1,j),dRMapXy(:,:,i-1,j),dRMapYx(:,:,i-1,j),dRMapYy(:,:,i-1,j),SIGMA(:,:,i-1,j),4+RR*Rc(k),drV(k),0,NA,N);
                        elseif j~=k && norm(Q(1:2,j,i-1)-Q(1:2,k,i-1)) <= Rs(j) % check agent-to-agent (uncommunicative) agent distance (sensing)
                            [RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j)]=RepulsiveField(j,Q(1:2,k,i-1),Xf,dx,Amin,RMapX(:,:,i-1,j),RMapY(:,:,i-1,j),dRMapXx(:,:,i-1,j),dRMapXy(:,:,i-1,j),dRMapYx(:,:,i-1,j),dRMapYy(:,:,i-1,j),SIGMA(:,:,i-1,j),SS*Rs(k),drV(k),0,NA,N);
                            
                            
                        end
                    end
                    
                    % check agent-to-agent, obstacle, or uncommunicative agent distance (sensing) -- repulsive field
                    
                    %         for k=1:NA
                    %             if j~=k && norm(Q(1:2,j,i-1)-Q(1:2,k,i-1)) <= Rs(j)
                    %                 [RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j)]=RepulsiveField(j,Q(1:2,k,i-1),Xf,dx,Amin,RMapX(:,:,i-1,j),RMapY(:,:,i-1,j),dRMapXx(:,:,i-1,j),dRMapXy(:,:,i-1,j),dRMapYx(:,:,i-1,j),dRMapYy(:,:,i-1,j),SIGMA(:,:,i-1,j),dcV(k),drV(k),0,NA,N);
                    %             end
                    %         end
                    
                    for k=1:NO
                        if norm(Q(1:2,j,i-1)-P(:,k)) <= dcS(k)
                            % %                         [RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j)]=RepulsiveField(j,P(:,k),Xf,dx,Amin,RMapX(:,:,i-1,j),RMapY(:,:,i-1,j),dRMapXx(:,:,i-1,j),dRMapXy(:,:,i-1,j),dRMapYx(:,:,i-1,j),dRMapYy(:,:,i-1,j),SIGMA(:,:,i-1,j),dcS(k),drS(k),0,NA,N);
                            %
                            %
                            [RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j)] = sigmaCheck(P(:,k),dx,Amin,dcS(k),drS(k),N,RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j));
                            %
                        end
                    end
                    
                    if Nd >= 1
                        for k=1:Nd
                            if norm(Q(1:2,j,i-1)-Pd((2*k-1:2*k),1)) <= Rs(j)
                                [RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j), dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j)]=RepulsiveField(j,Pd((2*k-1:2*k),i),Xf,dx,Amin,RMapX(:,:,i-1,j),RMapY(:,:,i-1,j),dRMapXx(:,:,i-1,j),dRMapXy(:,:,i-1,j),dRMapYx(:,:,i-1,j),dRMapYy(:,:,i-1,j),SIGMA(:,:,i-1,j),SS*Rs(j),drS(k),0,NA,N);
                            end
                        end
                    end
                    % Calculate the Vehicles Actual & Derivative Map [ Put this in a
                    % function
                    
                    
                    [MapX(:,:,i-1,j),MapY(:,:,i-1,j),dMapXx(:,:,i-1,j),dMapXy(:,:,i-1,j),dMapYx(:,:,i-1,j),dMapYy(:,:,i-1,j)]=VehicleMap(RMapX(:,:,i-1,j), RMapY(:,:,i-1,j), dRMapXx(:,:,i-1,j), dRMapXy(:,:,i-1,j), dRMapYx(:,:,i-1,j),dRMapYy(:,:,i-1,j), SIGMA(:,:,i-1,j), AMapX(:,:,i-1,j), AMapY(:,:,i-1,j), dAMapXx(:,:,i-1,j), dAMapXy(:,:,i-1,j), dAMapYx(:,:,i-1,j),dAMapYy(:,:,i-1,j));
                    
                    
                    % Find Vehicle in ArenaMesh
                    
                    [Qx,Qy]=findVehicle(Amin,Q(1:2,j,i-1),dx);
                    
                    % Controller
                    if norm(Q(1:2,j,i-1)-Xf(1:2,j)) < D/20
                        u=0;
                        omega=0;
                        END(j)=1;
                        sprintf('Agent %d has reached their target',j)
                    else
                        [u,omega,phi] = Controller(Ku,Kw,Kp,q,Xf(:,j),MapX(Qx,Qy,i-1,j),MapY(Qx,Qy,i-1,j),dMapYx(Qx,Qy,i-1,j),dMapYy(Qx,Qy,i-1,j),dMapXx(Qx,Qy,i-1,j),dMapXy(Qx,Qy,i-1,j),D);
                    end
                    
                    % Input Controls to model
                    U(j,i)=u;
                    OMEGA(j,i)=omega;
                    PHI(j,i)=phi;
                    qdot=[cos(q(3)) 0 ;
                        sin(q(3)) 0 ;
                        0            1]*[u; omega];
                    q=q+delta*qdot;
                    
                    for k=1:NA
                        distance=norm(Q(1:2,j,i-1)-Q(1:2,k,i-1));
                        if k~=j
                            NORM(RR,i-1)=distance;
                        end
                        if k~=j && distance < mindistance(RR,k)
                            mindistance(RR,k) = distance;
                        end
                    end
                    
                    
                    
                    FV(RR,j)=norm(Xf(1:2,j)-q(1:2));
                    
                    Q(:,j,i)=q;
                end
            end
        end
        
    end
end

%%
for i=1:NA
    Xx(1:newNt,i) = Q(1,i,1:newNt);
    Xy(1:newNt,i) = Q(2,i,1:newNt);
    Xtheta(1:newNt,i) = Q(3,i,1:newNt);
end

%%
clf
set(gcf,'color','w');
subplot(2,1,1)
plot(RCdelta:RCdelta:RCdelta*RADIUS,FV(1:RADIUS,:))
xlabel('R_c [m]')
ylabel('Distance [m]')
legend('Agent 1 Delta [m]','Agent 2 Delta [m]','Location','best')
subplot(2,1,2)
plot(linspace(0,(newNt-1)*delta,newNt-1),NORM(:,1:newNt-1))
xlabel('Time [s]')
ylabel('Distance between agents [m]')


%%
for i=1:NA
    Xx(1:newNt,i) = Q(1,i,1:newNt);
    Xy(1:newNt,i) = Q(2,i,1:newNt);
    Xtheta(1:newNt,i) = Q(3,i,1:newNt);
end
%% Plot all sensor / communication data
% Hold Sensor Distance Constant
clf

for k=1:RADIUS
    plot(Q(1,NA*Nt*(k-1)+1:NA*Nt*(k-1)+NA*Nt),Q(2,NA*Nt*(k-1)+1:NA*Nt*(k-1)+NA*Nt),'.')
    hold on
    pause(1)
    
end
plot(P(1,:),P(2,:),'.',X(1,:),X(2,:),'O',Xf(1,:),Xf(2,:),'X')

xlabel('x [m]')
ylabel('y [m]')

%% Hold Communication Distance Constant

RADindex=1; % Choose communication index
for k=1:SENSOR
    
    plot(Q(1,RADindex*NA*Nt*(k-1)+1:RADindex*NA*Nt*(k-1)+1+RADindex*NA*Nt),Q(2,RADindex*NA*Nt*(k-1)+1:RADindex*NA*Nt*(k-1)+1+RADindex*NA*Nt),'.')
    hold on
    pause(1)
end

%%

clf
for j=2:newNt
    for i=1:NA
        for k=1:RADIUS
            
            plot(Q(1,NA*Nt*(RR-1)+1:(NA*Nt*(RR-1)+(3*j-(i-1)))),Q(2,NA*Nt*(RR-1)+1:(NA*Nt*(RR-1)+3*j-(i-1))),'.')
            hold on
            pause(delta/1000)
        end
    end
end
%% Field Plot

clf
for kk=100:2:newNt
    for i=1:NA
        figure(i)
        set(gcf,'color','w');
        
        clf
        quiver(Xp(:,:),Yp(:,:),MapX(:,:,kk-1,i),MapY(:,:,kk-1,i))
        hold on
        for j=1:NO
            viscircles(P(:,j)',drS(j),'color','r');
            value = sprintf('%d', j);
            text(P(1,j)+D/100,P(2,j),value)
        end
        for jj=1:Nd
            viscircles(Pd(2*jj-1:2*jj,kk-1)',drD(jj),'color','r');
            value = sprintf('Dyn # %d', jj);
            text(Pd(2*jj-1,kk)+D/100,Pd(2*jj,kk),value)
        end
        for ii=1:NA
            value = sprintf('A#%d', ii);
            text(Xx(kk-1,ii)+D/50,Xy(kk-1,ii),value)
            hold on
        end
        hold on
        plot(Xf(1,i),Xf(2,i),'X')
        plot(Xx(1:kk-1,i),Xy(1:kk-1,i),'--','Color','k')
        plot(Xx(kk-1,NA),Xy(kk-1,NA),'*','Color','k')
        axis([-2*D+Ac(1) 2*D+Ac(1) -2*D+Ac(2) 2*D+Ac(2) ]);
        axis([-10 10 -10 10 ]);
        
        pause(delta/1000)
        
    end
end
%% Agent Location Comparisons (x,y) plot and norm to other agents
figure(5)
clf
for i=2:10:newNt
    for j=1:NA
        for jj=1:NA
            subplot(NA,NA,(j-1)*NA+jj)
            if j==jj
                plot(Xx(1:i,j),Xy(1:i,j))
                hold on
                plot(P(1,:),P(2,:),'.',X(1,:),X(2,:),'O',Xf(1,:),Xf(2,:),'X')
                axis([-2*D 2*D -2*D 2*D])
                xlabel('x [m]')
                ylabel('y [m]')
                hold off
                
            else
                plot(t(1:i)',sqrt((Xx(1:i,j)-Xx(1:i,jj))'.^2 + (Xy(1:i,j)-Xy(1:i,jj))'.^2));
                xlim([0 max(t)])
                ylim([0 max(max(Xy))])
                xlabel('Time [s]')
                ylabel('Separation [m]')
            end
        end
    end
    pause(delta/1000)
end
%%

% Add up total contribution for agent k and point (x,y)
MapX=RMapX+1*(1-SIGMA).*AMapX;
MapY=RMapY+1*(1-SIGMA).*AMapY;

dMapXx=dRMapXx+1*(1-SIGMA).*dAMapXx;
dMapXy=dRMapXy+1*(1-SIGMA).*dAMapXy;
dMapYy=dRMapYy+1*(1-SIGMA).*dAMapYy;
dMapYx=dRMapYx+1*(1-SIGMA).*dAMapYx;

% Normalize the map
MapX=MapX./sqrt((MapX.^2+MapY.^2));
MapY=MapY./sqrt((MapX.^2+MapY.^2));

dMapXx=dMapXx./sqrt((dMapXx.^2+dMapXy.^2));
dMapXy=dMapXy./sqrt((dMapXx.^2+dMapXy.^2));
dMapYy=dMapYy./sqrt((dMapYx.^2+dMapYy.^2));
dMapYx=dMapYx./sqrt((dMapYx.^2+dMapYy.^2));

%%
clf
%  for
for kk=2:2
    for i=1:NA
        set(gcf,'color','w');
        
        figure(i)
        clf
        plot3(Xp,Yp,SIGMA(:,:,kk,i))
        pause(delta/2)
        box on
    end
end
%%
clf
%  for
for kk=3:1:3
    for i=1:1
        figure(i)
        clf
        set(gcf,'color','w');
        
        %         quiver(Xp,Yp,MapX(:,:,kk,i),MapY(:,:,kk,i))
        quiver(Xp,Yp,RMapX(:,:,kk,i),RMapY(:,:,kk,i))
        for j=1:NO
            viscircles(P(:,j)',drS(j),'color','r');
            value = sprintf('%d', j);
            text(P(1,j)+D/100,P(2,j),value)
        end
        for ii=1:NA
            value = sprintf('A#%d', ii);
            text(Xx(kk-1,ii)+D/50,Xy(kk-1,ii),value)
            hold on
        end
        hold on
        plot(X(1,i),X(2,i),'*',Xf(1,i),Xf(2,i),'X')
        axis([-10 10 -10 10 ]);
        
        pause(delta)
    end
end
%%

%  Controller Tuning
%
%      iii=fix(iii);
%     Legend{iii} = char(sprintf('Ku=%d,Kw=%d,Kp=%d',Ku,Kw,Kp)); % or whatever is appropriate
%
%
%  figure(1)
%     plot(Xx,Xy,'--')
%     hold on
%     axis([-2*D+Ac(1) 2*D+Ac(1) -2*D+Ac(2) 2*D+Ac(2) ]);
%     legend(Legend)
%
%     iii=iii+1;
%
%  end
%     end
%     end

%     % Remove field "inside" an object generated by another object.
%     for iii=1:N
%         for ii=1:N
%             POS =[Amin(1)+iii*dx Amin(2)+ii*dx]; % x and y position in space
%             for k=1:NA
%
%                 % Remove Field Inside Agent
%
%                 for kk=1:NO % Remove Field in Static Obstacles
%                     if norm(POS'-P(:,kk)) < dr(kk) %R(kk)
%                         RMapX(iii,ii,i-1,k)=0;
%                         RMapY(iii,ii,i-1,k)=0;
%                         AMapX(iii,ii,i-1,k)=0;
%                         AMapY(iii,ii,i-1,k)=0;
%                         dAMapXx(iii,ii,i-1,k)=0;
%                         dAMapXy(iii,ii,i-1,k)=0;
%                         dAMapYx(iii,ii,i-1,k)=0;
%                         dAMapYy(iii,ii,i-1,k)=0;
%                         dRMapXx(iii,ii,i-1,k)=0;
%                         dRMapXy(iii,ii,i-1,k)=0;
%                         dRMapYx(iii,ii,i-1,k)=0;
%                         dRMapYy(iii,ii,i-1,k)=0;
%                     end
%                 end
%                 for jj=1:Nd % Remove Field in Dynamic Obstacles
%                     if norm(POS'-Pd(2*jj-1:2*jj,i)) < dr(NO+jj)
%                         RMapX(iii,ii,i-1,k)=0;
%                         RMapY(iii,ii,i-1,k)=0;
%                         AMapX(iii,ii,i-1,k)=0;
%                         AMapY(iii,ii,i-1,k)=0;
%                         dAMapXx(iii,ii,i-1,k)=0;
%                         dAMapXy(iii,ii,i-1,k)=0;
%                         dAMapYx(iii,ii,i-1,k)=0;
%                         dAMapYy(iii,ii,i-1,k)=0;
%                         dRMapXx(iii,ii,i-1,k)=0;
%                         dRMapXy(iii,ii,i-1,k)=0;
%                         dRMapYx(iii,ii,i-1,k)=0;
%                         dRMapYy(iii,ii,i-1,k)=0;
%                     end
%                 end
%                 for kk=1:NA
%                         if kk~=k && norm(POS'-q(1:2,k)) < rho/2
%                         RMapX(iii,ii,i-1,k)=0;
%                         RMapY(iii,ii,i-1,k)=0;
%                         AMapX(iii,ii,i-1,k)=0;
%                         AMapY(iii,ii,i-1,k)=0;
%                         dAMapXx(iii,ii,i-1,k)=0;
%                         dAMapXy(iii,ii,i-1,k)=0;
%                         dAMapYx(iii,ii,i-1,k)=0;
%                         dAMapYy(iii,ii,i-1,k)=0;
%                         dRMapXx(iii,ii,i-1,k)=0;
%                         dRMapXy(iii,ii,i-1,k)=0;
%                         dRMapYx(iii,ii,i-1,k)=0;
%                         dRMapYy(iii,ii,i-1,k)=0;
%                         end
%                 end