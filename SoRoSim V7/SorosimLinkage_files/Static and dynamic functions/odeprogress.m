%Runs parallel with the ode45
%Last modified by Anup Teejo Mathew, 02.03.2022
function status = odeprogress(t,y,flag,Linkage,Show)
%ODEWBAR Graphical waitbar printing ODE solver progress.
%   When the function odewbar is passed to an ODE solver as the 'OutputFcn'
%   property, i.e. options = odeset('OutputFcn',@odewbar), the solver calls 
%   ODEWBAR(T,Y,'') after every timestep. The ODEWBAR function shows a
%   waitbar with the progress of the integration every 0.2 seconds.
%   
%   At the start of integration, a solver calls ODEWBAR(TSPAN,Y0,'init') to
%   initialize the output function.  After each integration step to new time
%   point T with solution vector Y the solver calls STATUS = ODEWBAR(T,Y,'').
%	When the integration is complete, the solver calls ODEWBAR([],[],'done').
%
%   See also ODEPLOT, ODEPHAS2, ODEPHAS3, ODE45, ODE15S, ODESET.

%   José Pina, 22-11-2006
%	

persistent tlast

% regular call -> increment wbar
if nargin < 3 || isempty(flag)
	
	% update only if more than 0.2 sec elapsed
 	if cputime-tlast>0.2
 		tlast = cputime;
        if Show
            plotalong(Linkage,t(end),y(:,end)) 
        end
 	else
 		status = 0;
 		return
 	end

% initialization / end
else
  switch(flag)
  case 'init'               % odeprint(tspan,y0,'init')
	  
      if Show
        close all
        PlotParameters = Linkage.PlotParameters;
        fh=figure(1);
        fh.Units='normalized';
        FigScale = PlotParameters.VideoResolution;
        FigScale(FigScale<0.1)=0.5;
        FigScale(FigScale>1)=1;
        FigLocation = (1-FigScale)/2;
        fh.OuterPosition=[FigLocation FigLocation FigScale FigScale];
        
        set(gca,'CameraPosition',PlotParameters.CameraPosition,...
            'CameraTarget',PlotParameters.CameraTarget,...
            'CameraUpVector',PlotParameters.CameraUpVector,...
            'FontSize',18)
        
        if PlotParameters.Light
            camlight(PlotParameters.Az_light,PlotParameters.El_light)
        end
        
        axis equal
        grid on
        hold on
        xlabel('x (m)')
        ylabel('y (m)')
        zlabel('z (m)')
        
      end
      tlast = cputime;
    
  case 'done'
    if Show
        close all
    end
  end
end

status = 0;

