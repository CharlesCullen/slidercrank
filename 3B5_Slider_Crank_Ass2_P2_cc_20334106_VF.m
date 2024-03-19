%3B5 Machines Ass 2 Part 2 final
%Charles Cullen 20334106 


% Slider crank mechanism position solved using complex algebra loop closure solution

close all; clear;

% Scaled parameters for simulation

r2 = 0.126;                      % Crank length (m) scaled 3:1
r3 = 0.387;                      % Coupler (connecting rod) length (m) scaled 3:1
Ax = 0;                          % Crank center position x
Ay = 0;                          % Crank center position y
theta2_initial = 0;              % Initial crank angle with respect to x-axis (radians)
theta2d = 15;                    % Crank angular velocity (radians/sec)
BP = 1.26; theta5 = 0.8;         % Dimensions for coupler, BP in m 3:1 scale, theta5 in radians
theta2dd = 0;                    %crank angular acceleration (constantly rotating)

% Actual parameters of full scal size of real piston mechanism

% r2 = 0.042;                 %crank length (m)
% r3 = 0.129;                 %coupler (connecting rod) length (cm)
% Ax = 0;                     %crank centre position x
% Ay = 0;                     %crank centre position y 
% theta2_initial = 0.15;      %initial crank angle wrt x axis (radians)
% theta2d = 6500 * (2*pi/60); %max crank angular velocity (radians/sec) 
% BP = 0.42; theta5 = 0.8;    %dimensions for coupler, BP in m, theta5 in rad
% theta2dd = 0;               %crank angular acceleration


% Plot parameters and counter
a = 12; i = 0;                   % 'a' for font size, 'i' for iteration counter
ps = 0.07;                       % Patch size for visual representation
t2 = (6*pi/theta2d);             % Total time for simulation


% Set the plot handles
figure(1); hold on;
h3 = text(-1,-1,'0');                                         % Initialize text for time display
h4 = patch([0 0 0 0],[0 0 0 0],'b-');                         % Initialise patch for piston representation
h1 = plot([0 0 0], [0 0 0],'ro','markerfacecolor',[1 0 0]);   % Initialise points for linkage joints
h2 = plot([0 0 0], [0 0 0],'c-','linewidth',4);               % Initialise lines for linkages
h2a = plot(0, 0, 'co');                                       % Auxiliary point for visual emphasis


% Configure plot appearance
axis([-1 1 -1 1]); axis('square'); axis off;
title('Slider Crank linkage position','FontSize',a);
plot([-1 5],[0 0], 'k-');        % Ground line


% Plotting the crankshaft's path in yellow
theta_path = linspace(0, 2*pi, 100);               % Generate angles from 0 to 2*pi for a full circle
Bx_path = Ax + r2*cos(theta_path);                 % X-coordinates of the crankshaft's path
By_path = Ay + r2*sin(theta_path);                 % Y-coordinates of the crankshaft's path
plot(Bx_path, By_path, 'y-', 'LineWidth', 2);      % Plot the path with a yellow line


% Start calculation loop
for t = 0:0.01:t2

    % Increment counter and update time array
    i = i+1;  time(i) = t;

    % Define crank angle at each time increment
    theta2(i) = theta2_initial + theta2d * t;

    % Adjust theta2 after each revolution to loop from 0
    if theta2(i) >= 2*pi
        theta2(i) = theta2(i) - 2*pi*floor(theta2(i)/(2*pi));
    end

    % Define position of B at each time increment
    Bx(i) = r2*cos(theta2(i));
    By(i) = r2*sin(theta2(i));
   
    % Define connecting rod angle at each time increment
    theta3(i) = asin(-(r2/r3)*sin(theta2(i)));

    % Define r1 and piston position at each time increment
    r1(i) = r2*cos(theta2(i)) + sqrt((r3^2) - (r2^2)*((sin(theta2(i)))^2));
   
    % Calculate the position of point C (piston end)
    Cx(i) = Bx(i) + r3*cos(theta3(i));
    Cy(i) = Ay;

    % Define position of coupler point P
    Px(i) = Bx(i) + BP*cos(theta3(i)+theta5);
    Py(i) = By(i) + BP*sin(theta3(i)+theta5);
  
    % VELOCITY ANALYSIS
   
        theta3d(i) = -theta2d * (r2/r3) * (cos(theta2(i)) /cos(theta3(i)));
        
        % velocity-time derivative of crank (v1 = r1d)
        v1(i) = -r2 * theta2d * sin(theta2(i)) * (1 + (r2/r3) * cos(theta2(i)));
       
        % linear velocity of point B at each time increment 
        % (1st derivate of Bx and By)
        vBx(i) = -r2 * theta2d * cos((pi/2) - theta2(i));
        vBy(i) = r2 * theta2d * sin((pi/2) - theta2(i));
       
        % linear velcocities by vector cross products
        omega2_vector(i, 1:3) = [0 0 theta2d]; % angualr velocity of link 2 (crank)
        omega3_vector(i, 1:3) = [0 0 theta3d(i)]; % angualr velocity of link 3 (con rod)
    
        r2_vector(i, 1:3) = [Bx(i)  By(i)    0];
        r3_vector(i, 1:3) = [(Cx(i)-Bx(i)) (Cy(i)-By(i))   0];
    
        v_B_vector(i, 1:3) = cross(omega2_vector(i, 1:3), r2_vector(i, 1:3));
        v_C_vector(i, 1:3) = v_B_vector(i, 1:3) + cross(omega3_vector(i, 1:3), r3_vector(i, 1:3));
       
    % ACCELERATION ANALYSIS
   
        theta3dd(i) = (r3*(theta3d(i))^2 * sin(theta3(i)) + r2*(theta2d)^2 * sin(theta2(i)) - r2*theta2dd*cos(theta2(i)))/(r3*cos(theta3(i)));
      
        % Acceleration, 2nd time derivative of r1 (a1 = r1dd) 
        a1(i) = -r2*theta2dd*sin(theta2(i)) - r2*(theta2d^2)*cos(theta2(i)) - r3*theta3dd(i)*sin(theta3(i)) - r3*(theta3d(i)^2)*cos(theta3(i));

        % 2nd derivate of Bx and By
        a_Bx(i) =  -r2 * theta2d * theta2d * cos(theta2(i));
        a_By(i) =  -r2 * theta2d * theta2d * sin(theta2(i));
       
        % Linear accelerations by vector cross products
        alpha2_vector(i,1:3) = [0 0 theta2dd];
        alpha3_vector(i,1:3) = [0 0 theta3dd(i)];
        a_B_vector(i,1:3) = cross(alpha2_vector(i,1:3),r2_vector(i,1:3)) + cross(omega2_vector(i,1:3),v_B_vector(i,1:3));
        a_C_vector(i,1:3) = a_B_vector(i, 1:3) + cross(omega3_vector(i,1:3),cross(omega3_vector(i,1:3),r3_vector(i,1:3))) + cross(alpha3_vector(i,1:3), r3_vector(i,1:3));
      
    % Update plot information
    figure(1);
    set(h4,'Xdata', [Cx(i)-ps Cx(i)+ps Cx(i)+ps Cx(i)-ps], 'Ydata', [Cy(i)-ps Cy(i)-ps Cy(i)+ps Cy(i)+ps]);
    set(h1,'Xdata', [Ax Bx(i) Cx(i)], 'Ydata', [Ay By(i) Cy(i)]);
    set(h2,'Xdata', [Ax Bx(i) Cx(i)], 'Ydata', [Ay By(i) Cy(i)]);
    set(h2a,'Xdata', [Px(i) ], 'Ydata' , [Py(i) ]);  
    set(h3,'string',['t = ' num2str(time(i))]);
    pause(0.01); 
end


%---------LINEAR VELOCITY-----------
%Linear velocity of piston (end of conn rod, C)
figure(2); hold on; grid on; box on; a = 16;
set(gcf,'color','white')
set(gca,'fontsize',14)
title('Linear velocity of piston, (v_1 = r1d)')
xlabel('Time - s','fontsize',a)
ylabel('Velocity - m/s','fontsize',a)
plot( time, v1,'b', 'linewidth',2)
plot(time, v_C_vector(:,1), 'rv', time, v_C_vector(:,2), 'g', 'linewidth',1)
legend('v1 (using loop closure eq)','vC_x (using cross product eq)', 'vC_y (using cross product eq)')

%Linear velocity of crank (point B)
figure(3); hold on; grid on; box on; a = 16;
set(gcf,'color','white')
set(gca,'fontsize',14)
title('Linear velocity of crank, B')
xlabel('Time - s','fontsize',a)
ylabel('Velocity - m/s','fontsize',a)
plot(time, vBx,'b', time, vBy, 'r', 'linewidth',2)
plot(time, v_B_vector(:,1), 'go', time, v_B_vector(:,2), 'ks', 'linewidth',1)
legend('vB_x (using loop closure eq)','vB_y (using loop closure eq)', 'vB_x (using cross product eq)', 'vB_y (using cross product eq)')

%----------ANGULAR VELOCITY----------------
%Angular velocity
figure(4); hold on; grid on; box on; a = 16;
set(gcf,'color','white')
set(gca,'fontsize',14)
title('Angular velocity  ')
xlabel('Time - s','fontsize',a)
ylabel('Angular Velocity - rad/s','fontsize',a)
plot(time, theta3d,'b', time, theta2d, 'go','linewidth',2)
legend('theta3d (Connecting rod angualr velocity)', 'theta2d (Crank rod angualr velocity(const))')

%--------LINEAR ACCELERATION------------
%Linear acceleration of piston (end of conn rod, C)
figure(5); hold on; grid on; box on; 
set(gcf,'color','white')
set(gca,'fontsize',14)
title('Linear Acceleration of Piston  (a_1 = r1dd)')
plot( time, a1,'b',time, a_C_vector(:,1),'mv', time, a_C_vector(:,2), 'g','linewidth',1)
legend('a_1 (using loop closure eq)','aC_x (using cross product eq)', 'aC_y (using cross product eq)')
xlabel('Time - s')
ylabel('Linear Acceleration - m/s^2')


%Linear acceleration of crank (point B)
figure(6); hold on; grid on; box on; 
set(gcf,'color','white')
set(gca,'fontsize',14)
title('Linear Acceleration of crank end, B')
plot( time, a_Bx,'b',time, a_By,'r', 'linewidth', 2)
plot(time, a_B_vector(:,1), 'go', time, a_B_vector(:,2), 'ks','linewidth',1)
legend('aB_x (using algebra)','aB_y (using algebra)', 'aB_x (using cross product eq)', 'aB_y (using cross product eq)')
xlabel('Time - s')
ylabel('Linear Acceleration - m/s^2')

%--------ANGUALR ACCELERATION---------
%Angular acceleration of piston (end of conn rod, C)
figure(7); hold on; grid on; box on; 
set(gcf,'color','white')
set(gca,'fontsize',14)
title('Angular Acceleration')
xlabel('Time - s')
ylabel('Angular Acceleration - rad/s^2')
plot(time, theta3dd,'b', time, theta2dd, 'go','linewidth',2)
legend('theta3dd (Connecting rod ang acceleration)', 'theta2dd (Crank ang acceleration (=0))')

