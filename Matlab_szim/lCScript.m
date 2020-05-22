Ts = 1/60;
vt = 0.22;
maxvr = 1.82;
%------------------
xi = 1.0;
w0 = 5;
z1 = exp((-w0*xi+w0*sqrt(xi^2-1))*Ts);
z2 = exp((-w0*xi-w0*sqrt(xi^2-1))*Ts);
%------------------
%állapotváltozók: fi, h
%bemenet: vr
A = [1 0; Ts*vt 1];
B = [Ts; 0];
C = [0 1];
D = 0;
%------------------
plant = ss(A, B, C, D, Ts);
%step(plant); %uncontroled, unstable system
Mc = ctrb(A, B);
conrolable = rank(Mc) == 2;
phic = [z1 z2];
K = acker(A, B, phic);
controledSys = ss(A-B*K, B, C, D, Ts);
%step(controledSys) %controled, stable system
%------------------
x0 = [pi/2; 0];
%initial(controledSys,x0);
K2 = [(2-z1-z2)/Ts (1-z1-z2+z1*z2)/Ts^2/vt];
ka1 = 2-z1-z2;
ka2 = 1-z1-z2+z1*z2;
1000*[ka1 ka2]
