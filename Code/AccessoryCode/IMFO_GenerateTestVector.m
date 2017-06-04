%This script generates test vector for the IMFO Predict time of impact
%code

clear

%% Model Inputs
h0 = 1; %[m]
v0 = -0.1; %[m/sec]
t = (20:20:150)*1e-3; %[sec]
g=9.81; %[m/sec^2]

h = h0+v0*t-1/2*g*t.^2;
h = round(h*1000)/1000;

fprintf('Time[ms]\tDist[m]\n');
fprintf('%.0f\t\t\t%.3f\n',[t(:)*1000,h(:)]');

%% What is the predicted v0,h0
fprintf('NSamples\th0[m]\tv0[m]\ttImp[msec]\n');
for nM = 3:length(t) %Variating number of measurments
    h_ = h(1:nM);
    t_ = t(1:nM);
    x=[ones(size(t_(:))) t_(:)];
    c=-1/2*g*[t_(:).^2];
    p = (transpose(x)*x)^-1*transpose(x)*(h_(:)-c);
    h0 = p(1);
    v0 = p(2);
    
    tImp = (v0 + sqrt(v0^2 + 2*g*h0))/g;
    
    fprintf('%d\t\t\t%.4f\t%.4f\t%.0f\n',nM,h0,v0,floor(tImp*1e3));
    
end