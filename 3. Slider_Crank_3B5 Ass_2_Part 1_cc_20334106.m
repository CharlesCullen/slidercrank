%3B5 Machines Ass 2 Part 1
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


% Actual parameters of full scale size of real piston mechanism
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
t2 = 2;                          % Total time for simulation


% Set the plot handles
figure(1); hold on;
set(gcf, 'Position', [100, 100, 600, 600])                    % Make figure larger
h3 = text(-1,-1,'0');                                         % Initialise text for time display
h4 = patch([0 0 0 0],[0 0 0 0],'m-');                         % Initialise patch for piston representation
h1 = plot([0 0 0], [0 0 0],'ro','markerfacecolor',[1 0 0]);   % Initialise points for linkage joints
h2 = plot([0 0 0], [0 0 0],'c-','linewidth',4);               % Initialise lines for linkages
h2a = plot(0, 0, 'co');                                       % Auxiliary point for visual emphasis


% Configure plot appearance
axis([-1 1 -1 1]); axis('square'); axis off;
title('Slider Crank linkage position','FontSize',a);
plot([-1 5],[0 0], 'k-');        


% Plotting the crankshaft's path in yellow
theta_path = linspace(0, 2*pi, 100);               % Generate angles from 0 to 2*pi for a full circle
Bx_path = Ax + r2*cos(theta_path);                 % X-coordinates of the crankshaft's path
By_path = Ay + r2*sin(theta_path);                 % Y-coordinates of the crankshaft's path
plot(Bx_path, By_path, 'y-', 'LineWidth', 2);      % Plot path with a yellow line


% Start calculation loop
for t = 0:0.01:t2

    % Increment counter and update time array
    i = i+1;
    time(i) = t;

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

    % Update plot information
    figure(1);
    set(h4,'Xdata', [Cx(i)-ps Cx(i)+ps Cx(i)+ps Cx(i)-ps], 'Ydata', [Cy(i)-ps Cy(i)-ps Cy(i)+ps Cy(i)+ps]);
    set(h1,'Xdata', [Ax Bx(i) Cx(i)], 'Ydata', [Ay By(i) Cy(i)]);
    set(h2,'Xdata', [Ax Bx(i) Cx(i)], 'Ydata', [Ay By(i) Cy(i)]);
    set(h3,'string',['t = ' num2str(time(i))]);

    pause(0.01); 
end

% Post-simulation plots
figure(2); hold on; grid on; box on;
subplot(1,2,1) % Subplot for angles
set(gcf,'color','white')
set(gca,'fontsize',9)
plot(time, theta2, 'r-', time, theta3, 'b-') % Plot crank and coupler angles
legend('crank ( \theta_2 )', 'coupler ( \theta_3 )')
xlabel('time - s')
ylabel('angle - rad')

subplot(1,2,2); % Subplot for piston position
hold on; grid on; box on;
set(gcf,'color','white')
set(gca,'fontsize',9)
plot(time, r1, 'r-') % Plot piston position
legend('piston position ( r_1 )')
xlabel('time - s')
ylabel('position - m')
