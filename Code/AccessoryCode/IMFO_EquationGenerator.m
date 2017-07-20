%% Inputs
clear;
nMeasurments = 7;

%% Generate solution
syms g;
for i=1:nMeasurments
    eval(sprintf('syms ta%db ha%db;',i-1,i-1));
end

eval(['x=[' sprintf('1 ta%db; ',0:(nMeasurments-1)) '];']);
eval(['c=-1/2*g*[' sprintf('ta%db^2; ',0:(nMeasurments-1)) '];']);
eval(['h=[' sprintf('ha%db; ',0:(nMeasurments-1)) '];']);

sol = (transpose(x)*x)^-1*transpose(x)*(h-c);

h_0 = sol(1);
v_0 = sol(2);

%% String manipulations
h0_str = char(h_0);
h0_str(h0_str=='a')='[';
h0_str(h0_str=='b')=']';
for i=1:nMeasurments
    h0_str=strrep(h0_str,sprintf('t[%d]^2',i-1),sprintf('t[%d]*t[%d]',i-1,i-1));
end

v0_str = char(v_0);
v0_str(v0_str=='a')='[';
v0_str(v0_str=='b')=']';
for i=1:nMeasurments
    v0_str=strrep(v0_str,sprintf('t[%d]^2',i-1),sprintf('t[%d]*t[%d]',i-1,i-1));
end

fprintf('case %d:\n\t\t\tLog_AddNote("%d data points received");\n\t\t\th0 = %s;\n\t\t\tv0 = %s;\n\t\t\tbreak;\n',nMeasurments,nMeasurments,h0_str,v0_str);
