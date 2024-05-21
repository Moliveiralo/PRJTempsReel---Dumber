% DEMO IDENTIFICATION 
% Author: C. Poussot-Vassal [MOR Digital Systems / Onera]
% Date  : March 2022 (creation)
% 
% Description
% Demonstration script for data-driven dynamical model identification using 
% either
%  * the "ident" toolbox embedded in the Matlab Identification Toolbox, or 
%  * the Hankel driven 
% This script is a rather simple one and modifications may be applied by 
% users. Still, it provides a first shot for it, but does not prevent for 
% additional reading and thinking.
% 
% Note
% The script uses the "+insa" matlab package.
% 

clear all, close all, clc;
% >> Graphic settings
insa.initGraphics
load('signals')

% Identify using 'ident' on either (u,y) or (u,yn) couples
%tt  = t1(:); uu  = u1(:); yy  = y1n(:);
%tt  = t2(:); uu  = u2(:); yy  = y2n(:);
%tt  = t3(:); uu  = u3(:); yy  = y3n(:);
tt  = t4(:); uu  = u4(:); yy  = y4n(:);
dt  = tt(2)-tt(1);

% >> IDENTIFICATION VIA HANKEL
r       = [];
dt      = tt(2)-tt(1);
Hr      = insa.pencil(uu,yy,dt,r);
Hr      = stabsep(Hr);
Hr      = minreal(Hr);
Hr      = d2c(Hr,'zoh');
size(Hr)
%Hr = tf1;

tt2     = 0:.1:10;
uu2     = 1+.1*randn(length(tt2),1);
[yg,tg] = lsim(G,uu2,tt2);
[ys,ts] = lsim(Hr,uu2,tt2);

figure, 
subplot(121); hold on, axis tight
bodemag(G,Hr)
subplot(122); hold on, axis tight
plot(tg,yg,'-')
plot(ts,ys,'--')
xlabel('Time [s]'); ylabel('Amplitude [.]');
legend({'Original model','Identified model'},'Location','Best')

% >> IDENTIFICATION VIA IDENT
ident

