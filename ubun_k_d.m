%% Scripts in Ubuntu
% comands:
% cut: Ctrl + W
% copy: Atl + W
% paste: Ctrl + Y
%% Original Kathib potential function

U = 0.5 * eta * ((1/f(x)) - (1/f(x0) )^2 ;

% Proposed function
U(x) = 0.5 * eta * exp((f(x)+c)^2);

% function f(x) = energy = [0,1]

% Constant values: eta and c

%% Finding constant values (ignore this section)
% Fist equation: minium value
U (x = xmin = 0) = Kmin;

Kmin = 0.5 * eta * e^((0+c)^2);
Kmin = 0.5 * eta * e^(c^2);
eta = 2 * Kmin * e^(-c^2);

% Second equation: maximum value
U(x = xmax = 1) = Kmax;

Kmax = 0.5 * eta * e^((1+c)^2);
(1+c)^2 = 1 + 2c + c^2

Kmax = 0.5 * (2 * Kmin * e^(-c^2)) * exp^(1 + 2c + c^2);
Kmax = Kmin * e^(1+2c);
ln(Kmax/Kmin) = 1 + 2c;
c = 0.5 * (ln(Kmax/Kmin)-1)


%% Loading values 
% Stiffness parameters (XYZ)
Kmin = 100; Kmax = 5000; 
c_k = 0.5 * (log(Kmax/Kmin)-1);
eta_k = 2 * Kmin * exp(-c_k^2);

% Damping parameters
Dmin = 0.01; Dmax = 1;
c_d = 0.5 * (log(Dmax/Dmin)-1);
eta_d = 2 * Dmin * exp(-c_d^2);

% Mapping values
deltaU = 0.001;
u = [0:deltaU:1];
K = zeros(1,length(u));

for i=1:length(u)
    K(i) = 0.5 * eta_k * exp((u(i)+c_k)^2);
end

% Stiffness parameters (ABC)
K_abc = zeros(1,length(u));
Kmin_ABC = 10; Kmax_ABC=300;
c_k = 0.5 * (log(Kmax_ABC/Kmin_ABC)-1);
eta_k = 2 * Kmin_ABC * exp(-c_k^2);

for i=1:length(u)
    K_abc(i) = 0.5 * eta_k * exp((u(i)+c_k)^2);
end

%
deltaE = 0.001;
Ek = [0:deltaE:1];
D = zeros(1,length(Ek));

for i=1:length(u)
    D(i) = 0.5 * eta_d * exp((Ek(i)+c_d)^2);
end

%% Plotting
figure('Name','Parameters modified due to energy')
subplot(1,3,1)
grid on
hold on
plot(u,K)
xlim([0 1])
ylim([Kmin Kmax])
xlabel(['Potential energy'])
ylabel(['K gain [N/m]'])
title('Spring stiffness on XYZ axis')
hold off

subplot(1,3,2)
grid on
hold on
plot(u,K_abc)
xlim([0 1])
ylim([Kmin_ABC Kmax_ABC])
xlabel(['Potential energy'])
ylabel(['K gain [N/m]'])
title('Spring stiffness on ABC angles')
hold off

%
subplot(1,3,3)
grid on
hold on
plot2D = plot(Ek,D);
plot2D.Color = '#A2142F';
xlim([0 1])
ylim([Dmin Dmax])
xlabel(['Kinetic energy'])
ylabel(['D gain'])
title('Spring damping')
hold off