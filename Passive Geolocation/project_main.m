%visualizing the stateof an aircraft using simulink 3d animation youtube
%explore the nasa HL-20 Model matlab help center
%% Cleanup
% clear all used variables and previously used items that may cause malfunction
format compact;
clear ;clc;

%% Create a Virtual World from the Template

% CREATION OF WORLD
% bring up template
copyfile terrain.wrl terrain_copy.wrl;
myworld = vrworld('terrain_copy.wrl'); 
% open the virtual world
open(myworld); 
view(myworld);

%% Enemy Radar

% CREATION OF RADAR
% create a new Transform node, called "radar"
radar = vrnode(myworld, 'radar', 'Transform');
radar_inline = vrnode(radar, 'children', 'radar_Inline', 'Inline');
radar_inline.url='Patriot.wrl'; % the model is appointed

% RADAR'S SPECIFICATIONS
xradar = 4000;
yradar = 230;
zradar = 10500;
    radar.scale = [25 25 25]; 
    radar.translation = [xradar yradar zradar];
 
%% Airplane

% CREATION OF AIRPLANE
% create a new Transform node, called "SU_27"
plane = vrnode(myworld, 'SU_27', 'Transform');
plane_inline = vrnode(plane, 'children', 'SU_27_Inline', 'Inline');
plane_inline.url='SU_27.wrl'; % the model is appointed

% AIRPLANE'S SPECIFICATIONS
plane.scale = [4 4 4]; % scale the size of the airplane by a factor of 10, so that it is visible 
plane.rotation  = [1, 0, 0, -0.4]; % this command fix the the rotation of the model which is a little bit problematic

% the peak point of the terrain
xpeak = 6960;
ypeak = 1314;
zpeak = 5820;

% the plane initialized in the following coordinates
xplane = xpeak+2000;
yplane = ypeak+800; % the plane should be flying over this peak point
zplane = zpeak-70000;
%% ANTENNAS

% CREATION OF ANTENNAS
antenna_size = [0.5 0.5 0.5];

ant1 = vrnode(plane, 'children', 'ant1','Transform');
ant1_inline = vrnode(ant1, 'children', 'ant1_Inline', 'Inline');
ant1_inline.url='Sphere.wrl';
ant1.scale = antenna_size;

ant1b = vrnode(plane, 'children', 'ant1b','Transform');
ant1b_inline = vrnode(ant1b, 'children', 'ant1b_Inline', 'Inline');
ant1b_inline.url='Sphere.wrl';
ant1b.scale = antenna_size;

ant1c = vrnode(plane, 'children', 'ant1c','Transform');
ant1c_inline = vrnode(ant1c, 'children', 'ant1c_Inline', 'Inline');
ant1c_inline.url='Sphere.wrl';
ant1c.scale = antenna_size;

ant2 = vrnode(plane, 'children', 'ant2','Transform');
ant2_inline = vrnode(ant2, 'children', 'ant2_Inline', 'Inline');
ant2_inline.url='Sphere.wrl';
ant2.scale = antenna_size;

% ANTENNAS' SPECIFICATIONS
wing_angle = 90; % degrees
ant1_pos = [-5 0 -5];
ant1.translation  = ant1_pos; % sind & cosd get input in degree
ant1b.translation  = [ant1_pos(1)-cosd(wing_angle)*0.15 ant1_pos(2) ant1_pos(3)-sind(wing_angle)*0.15]; % 15 cm distance
ant1c.translation  = [ant1_pos(1)-cosd(wing_angle)*0.30 ant1_pos(2) ant1_pos(3)-sind(wing_angle)*0.30]; % 30 cm distance
ant2.translation  = [ant1_pos(1)-cosd(wing_angle)*3 ant1_pos(2) ant1_pos(3)-sind(wing_angle)*3]; % 3 m distance
%     ant1.center 
%     ant1.translation        %since the antennas are childs, they have two positions
%     ant1.translation_abs    %one is w.r.t. parent and the other one is absolute

%% %%%%%%%%%%%%%%%%%%   Airplane Movement & Actions   %%%%%%%%%%%%%%%%%%%%

% CREATION OF NECESSARY VARIABLES (defining before the loop increases the performance)
N = 800; % the number of steps plane changes its position, (assume every step as one second)
c = 3e8;
K =1; %derivative scaling factor

              %+ (rand(1)-0.5)*v_r_actual(i)
s=rng;
rng(s); %Restore the state of the random number generator to s; so that the values are the same as before.
SNR=30;
freq_sampling = c/50;

% VARIABLES FOR PHASE DIFFERENCE METHOD
% the variables whose name starts with "v_" are vectors of the variable sampled in time
freq = [0 zeros(1,N)];
v_xplane = [xplane zeros(1,N)];
v_yplane = [yplane zeros(1,N)];
v_zplane = [zplane zeros(1,N)];
v_xplane_dot = [0 zeros(1,N)];
v_yplane_dot = [0 zeros(1,N)];
v_zplane_dot = [0 zeros(1,N)];
v_r_actual = zeros(1,N+1); % actual distance in array form for every step

v_range_by_radar = zeros(1,N+1); % actual distance in array form for every step

v_theta = zeros(1,N+1);       %vertical angle between plane and enemy radar in array form for every step (assume it is known)
v_theta_w_n = zeros(1,N+1);       %vertical angle with noise
v_theta_ant = zeros(1,N+1);   %vertical angle between antennas
v_alpha = zeros(1,N+1);       %horizontal angle between radar and enemy radar  in array form for every step (assume it is known)
v_alpha_w_n = zeros(1,N+1);       %horizontal angle with noise
v_theta_w_n_dot = [0 zeros(1,N)];
v_theta_ant_dot = [0 zeros(1,N)];
v_alpha_w_n_dot = [0 zeros(1,N)];

d=((ant1.translation(1)-ant2.translation(1))^2 +(ant1.translation(2)- ...
        ant2.translation(2))^2 +(ant1.translation(3)-ant2.translation(3))^2 )^(1/2); %the distance between antennas (fixed)

v_t_minus_tant = zeros(1,N+1);   
v_phase_dif = zeros(1,N+1);      
v_phase_dif_dot = [0 zeros(1,N)];

s_d_freq = zeros(1,N+1); 
r1 = zeros(1,N+1);
s_d_angle1 = zeros(1,N+1); 
r2 = zeros(1,N+1);
r3 = zeros(1,N+1);
s_d_angle2 = zeros(1,N+1); 

% VARIABLES FOR WEIGHTED INSTRUMENTAL VARIABLES METHOD
n_wiv = zeros(1,N+1); 
r_wiv = zeros(N+1,2); 
a_wiv = zeros(2,N+1); 
d_wiv_old = zeros(1,N+1);
d_wiv = zeros(1,N+1); 
e_wiv = zeros(N+1,2); 
nn = zeros(1,N+1); 
% A_N = zeros(N+1,2); 
% b_N = zeros(N+1,1); 
% G_N = zeros(N+1,2); 
% W = zeros(N+1,N+1);
p_wiv = zeros(2,N+1); 
v_r_wiv = zeros(1,N+1); 
v_r_ls = zeros(1,N+1);
s_wiv= zeros(50,2);

for i=1:N+1 %for movement of the plane

      % POSITION AND VELOCITY OF THE PLANE
      % plane moves 138.9 m for every seconds which corresponds to 500 km/h
     plane.translation = [v_xplane(i) v_yplane(i)  v_zplane(i)+200];  
     
%     x = 6*(1-cos(t));
%     y=sin(t)
%     z = 7*exp(-(t-pi).^2)
    

    % defie frequency manually then add noise with predefined standard deviaton
    freq(i) = 10e9;
    if i==1
    else
    s_d_freq(i) = sqrt((6*freq_sampling^2)/(4*pi*pi*(i^3-i)*10^(SNR/10)));
    r1(i) = random(gmdistribution(0, s_d_freq(i)));
    r1(i) = randn*s_d_freq(i);
    freq(i) = freq(i)+r1(i)  ;
    end
    lambda = c/freq(i);
    
    if N+1>i
    v_xplane(i+1) = plane.translation(1);                     
    v_yplane(i+1) = plane.translation(2);
    v_zplane(i+1) = plane.translation(3);
     end
    vrdrawnow;      % update the view
    pause(0.0002);  

    v_r_actual(i) = ((xradar-v_xplane(i))^2 +(yradar-v_yplane(i))^2 +(zradar-v_zplane(i))^2 )^(1/2); %actual (ideal) distance

    % following variables and functions are establised based on "Passive Geolocation Using Phase Difference 
    % Rate For Electronic Support Systems" article
    %%%%%%%%%%%(USE GIVEN FIGURE ON THE MENTIONED ARTICLE ABOVE)%%%%%%%%%%%
    
    % HORIZONTAL AND VERTICAL ANGLES
            
    v_alpha(i) = atand( (yradar-ant1.translation_abs(2))/sqrt(  (xradar-ant1.translation_abs(1))^2+(zradar-ant1.translation_abs(3))^2)); %inverse sin in degrees
    v_theta(i) = atand( (zradar-ant1.translation_abs(3))/(xradar-ant1.translation_abs(1)))  ; %inverse tangent in degrees
    v_theta_ant(i) =  atand((ant2.translation_abs(3)-ant1.translation_abs(3))/(ant2.translation_abs(1)-ant1.translation_abs(1)))  ;
    
    s_d_angle1(i) = lambda/(d*abs(cosd(v_alpha(i)))*pi*sqrt(10^(SNR/10)));
    r2(i) = random(gmdistribution(0, s_d_angle1(i)));
    r2(i) = randn*s_d_angle1(i);
    v_alpha_w_n(i) = v_alpha(i)+r2(i);
    
    s_d_angle2(i) = lambda/(d*abs(cosd(v_theta(i)))*pi*sqrt(10^(SNR/10)));
    r3(i) = random(gmdistribution(0, s_d_angle2(i)));
    r3(i) = randn*s_d_angle2(i);
    v_theta_w_n(i) = v_theta(i)+r3(i);
%% PHASE DIFFERENCE METHOD
    
    if i>1 %derivative of plane's position (velocity)
    v_xplane_dot(i) = K*(v_xplane(i)-v_xplane(i-1));
    v_yplane_dot(i) = K*(v_yplane(i)-v_yplane(i-1));
    v_zplane_dot(i) = K*(v_zplane(i)-v_zplane(i-1));
    end

    if i>1 %derivative of angles
    v_theta_w_n_dot(i) = K*(v_theta_w_n(i)-v_theta_w_n(i-1));
    v_theta_ant_dot(i) = K*(v_theta_ant(i)-v_theta_ant(i-1));
    v_alpha_w_n_dot(i) = K*(v_alpha_w_n(i)-v_alpha_w_n(i-1));
    end

    v_t_minus_tant(i) = v_theta_w_n(i)-v_theta_ant(i);
    v_phase_dif(i) = 2*pi*freq(i)*d*cosd(v_t_minus_tant(i))*cosd(v_alpha_w_n(i))/c ;
    if i>1 %derivative of PHASE DIFFERENCE 
         v_phase_dif_dot(i) = K*(v_phase_dif(i)-v_phase_dif(i-1));
    end
%% WEIGHTED INSTRUMENTAL VARIABLES METHOD

%Necessary definitons and assignments
n_wiv(i) = v_theta_w_n(i) - v_theta(i);
r_wiv(i,1) = ant1.translation_abs(1);
r_wiv(i,2) = ant1.translation_abs(3);
a_wiv(1,i) = sind(v_theta_w_n(i));
a_wiv(2,i) = -cosd(v_theta_w_n(i));
s_wiv(i,1) = (xradar-ant1.translation_abs(1));
s_wiv(i,2) = (zradar-ant1.translation_abs(3));
d_wiv_old(i) = sqrt(s_wiv(i,1)^2 + s_wiv(i,2)^2);
e_wiv(i,:) = d_wiv_old(i)*sind(n_wiv(i)*a_wiv(:,i));

nn(i) = d_wiv_old(i)*sind(n_wiv(i));

if i==1
A_N = [(a_wiv(:,i).')];
b_N = [(a_wiv(:,i).')*r_wiv(i,:).'];
else
A_N = [A_N; (a_wiv(:,i).')];
b_N = [b_N; (a_wiv(:,i).')*r_wiv(i,:).']; 
end
 
p_LS(:,i) = inv((A_N.')*A_N)*(A_N.')*b_N;
% v_theta_w_n(i) = atand(( p_LS(i,2) - planetranslation3 )/( p_LS(i,1) - planetranslation1 ));   
d_wiv(i) = sqrt((p_LS(1,i)- r_wiv(i,1) )^2 + (p_LS(2,i)- r_wiv(i,2) )^2);

if i==1
G_N = [sind(v_theta_w_n(i)),cosd(v_theta_w_n(i))];
else
G_N = [G_N;sind(v_theta_w_n(i)),cosd(v_theta_w_n(i))]; 
end

W(i,i) = d_wiv(i)^2*s_d_angle2(i)^2;
p_wiv(:,i) = inv((G_N.')*inv(W)*A_N) * (G_N.') * inv(W)*b_N;

    if N+1>i
% distance calculation by least square method
v_r_ls(i+1) =sqrt((p_LS(1,i)-ant1.translation_abs(1))^2+(p_LS(2,i)-ant1.translation_abs(3))^2+(yradar-ant1.translation_abs(2))^2);
% distance calculation by least weighted instrumental variables
v_r_wiv(i+1) = sqrt((p_wiv(1,i)-ant1.translation_abs(1))^2+(p_wiv(2,i)-ant1.translation_abs(3))^2+(yradar-ant1.translation_abs(2))^2);
     end
end

f_wait = msgbox('Calculating..','Wait!');

%% Range Detected By Radar Returned From Signal Generator Function

% radar_range function has called to see how the radar estimated plane's positons over time
for i=1:N+1 
        v_range_by_radar(i)  = radar_range(xradar,yradar,zradar,...
        0,0,0,...
        v_xplane(i),v_yplane(i),v_zplane(i),...
        v_xplane_dot(i),v_yplane_dot(i),v_zplane_dot(i));
end
%% Range Detected By Phase Difference Method

% The antennas estimate the radar positionusing phase difference method
    v_r_phase_dif = abs( (-2*pi*freq*d./(c*(v_phase_dif_dot))).*...
        (sind(v_t_minus_tant) .* (-v_zplane_dot ).*cosd(v_theta_w_n)...
        -sind(v_t_minus_tant) .* (-v_xplane_dot ).*sind(v_theta_w_n)...
        +cosd(v_t_minus_tant) .* sind(v_alpha_w_n) .* (-v_yplane_dot ).*cosd(v_alpha_w_n)...
        -cosd(v_t_minus_tant) .* sind(v_alpha_w_n).^2 .* (-v_xplane_dot ).*cosd(v_theta_w_n)...
        -cosd(v_t_minus_tant) .* sind(v_alpha_w_n).^2 .* (-v_zplane_dot ).*sind(v_theta_w_n)...
        ))  ;

%% Figures

close(f_wait);
t = 0:N ; % total elapsed steps converted into seconds

f1 = figure('Position',[100,200,600, 500]);  % for angles and their derivatives
   plot(t,v_theta_w_n,'b');
   hold on
   plot(t,v_theta_ant,'g');
   hold on
   plot(t,v_alpha_w_n,'r');

   hold on
   plot(t,v_theta_w_n_dot,'b--');
   hold on
   plot(t,v_theta_ant_dot,'g--');
   hold on
   plot(t,v_alpha_w_n_dot,'r--');
   grid on
   grid minor
   xlabel('Time (s)');
   ylabel('Angle (degrees)');
   title('Theta, Theta_a_n_t & Alpha Angles Between Plane and Enemy Radar');
   legend('Theta (Vertical)','Theta_a_n_t','Alpha (Horizontal)','Theta dot','Theta_a_n_t dot','Alpha dot','Location','best');

   for i = 1:N+1
       if i<N+1
      v_r_phase_dif_cor(i) =( v_r_phase_dif(i) + v_r_phase_dif(i+1))/2  ;
       else
           v_r_phase_dif_cor(i) = 0;
       end
   end
   
f2 = figure('Position',[800,200,600,500]);  % for distances

   plot(t,v_r_actual/1000,'k');         axis([0 N 0 100]);
   hold on
   plot(t,v_r_phase_dif_cor/1000,'b.');     axis([0 N 0 100]);
   hold on
   plot(t,v_r_wiv/1000,'g.');           axis([0 N 0 100]);
   hold on
%    plot(t,v_r_ls/1000,'.');             axis([0 N 0 100]);
   hold on
   plot(t,v_range_by_radar/1000,'r.');  axis([0 N 0 100]);

   grid on
   grid minor
   xlabel('Time (s)');
   ylabel('Range (km)');
   legend('Real Distance','Distance Detected by The PDR','Distance Detected by The WIV','Distance Detected by The Radar','Location','best');
   title('Instant Range Estimation');

% f3 = figure;  % for gamma and alpha
%    plot(t,v_zplane);
%    hold on
%    plot(t,v_zplane_dot);
%    title('plane"s z position and z velocity');
% f4 = figure;  % range detected by the radar
%    plot(t,v_range_by_radar);
%    title('range detected by the radar');

%% Cleanup
% clear all used variables
delete(plane);
delete(radar);
close(myworld);
clear;clc;

f_success = msgbox('Program has concluded.. ','Success!');
% displayEndOfDemoMessage(mfilename);