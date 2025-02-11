% Function to compute inertia matrix (M) and the distance from base to CM (cx) for default geometries.
% Last modified by Anup Teejo Mathew 27.11.2024
function [M,cx] = RigidBodyProperties(CS,L,Rho,varargin)

if L>0
    if CS=='C'
        
        r = varargin{1}; %radius as function of X1 (X1 varies from 0 to 1)
        
        %Integration
        [Xs,Ws,nGauss] = GaussQuadrature(10); %change 10 if needed
        V      = 0;
        M_area = 0;

        for ii=2:nGauss-1
            r_here = r(Xs(ii));
            A_here = pi*r_here^2;
            V      = V+Ws(ii)*L*A_here;
            M_area = M_area+Ws(ii)*L*A_here*Xs(ii)*L;
        end

        if V~=0
            cx   = M_area/V;
        else
            cx = L/2;
        end

        mass = V*Rho;

        Ix = 0;
        Iy = 0;

        for ii=2:nGauss-1
            r_here = r(Xs(ii));
            Iy     = Iy+Ws(ii)*L*((pi/4)*r_here^4+pi*r_here^2*(Xs(ii)*L-cx)^2);
            Ix     = Ix+Ws(ii)*L*((pi/2)*r_here^4);
        end
        Iz = Iy;

    elseif CS=='R'

        h = varargin{1}; %height as function of X1 (X1 varies from 0 to 1)
        w = varargin{2}; %width as function of X1 (X1 varies from 0 to 1)

        [Xs,Ws,nGauss] = GaussQuadrature(10);
        V      = 0;
        M_area = 0;

        for ii=2:nGauss-1
            h_here = h(Xs(ii));
            w_here = w(Xs(ii));
            A_here = h_here*w_here;
            V      = V + Ws(ii)*L*A_here;
            M_area = M_area+ Ws(ii)*L*A_here*Xs(ii)*L;
        end

        if V~=0
            cx   = M_area/V;
        else
            if L==0
                cx = 0;
            else
                Vh      = 0;
                Mh_area = 0;
                Vw      = 0;
                Mw_area = 0;
                for ii=2:nGauss-1
                    h_here = h(Xs(ii));
                    w_here = w(Xs(ii));
                    Vh      = Vh + Ws(ii)*L*h_here;
                    Mh_area = Mh_area+ Ws(ii)*L*h_here*Xs(ii)*L;
                    Vw      = Vw + Ws(ii)*L*w_here;
                    Mw_area = Mw_area+ Ws(ii)*L*w_here*Xs(ii)*L;
                end
                if Vh==0
                    cx   = Mw_area/Vw;
                else
                    cx   = Mh_area/Vh;
                end
            end
        end

        mass = V*Rho;

        Ix = 0;
        Iy = 0;
        Iz = 0;
        for ii=2:nGauss-1
            h_here = h(Xs(ii));
            w_here = w(Xs(ii));
            Iy     = Iy+Ws(ii)*L*((Xs(ii)*L-cx)^2*h_here*w_here+h_here*w_here^3/12);
            Iz     = Iz+Ws(ii)*L*((Xs(ii)*L-cx)^2*h_here*w_here+h_here^3*w_here/12);
            Ix     = Ix+Ws(ii)*L*(h_here*w_here^3/12+h_here^3*w_here/12);
        end

    elseif CS=='E'

        a = varargin{1}; %semi major axis length as function of X1 (X1 varies from 0 to 1)
        b = varargin{2}; %semi minor axis length as function of X1 (X1 varies from 0 to 1)

        [Xs,Ws,nGauss] = GaussQuadrature(10);
        V      = 0;
        M_area = 0;

        for ii=2:nGauss-1
            a_here = a(Xs(ii));
            b_here = b(Xs(ii));
            A_here = pi*a_here*b_here;
            V      = V + Ws(ii)*L*A_here;
            M_area = M_area+ Ws(ii)*L*A_here*Xs(ii)*L;
        end

        if V~=0
            cx   = M_area/V;
        else
            if L==0
                cx = 0;
            else
                Va      = 0;
                Ma_area = 0;
                Vb      = 0;
                Mb_area = 0;
                for ii=2:nGauss-1
                    a_here = a(Xs(ii));
                    b_here = b(Xs(ii));
                    Va      = Va + Ws(ii)*L*a_here;
                    Ma_area = Ma_area+ Ws(ii)*L*a_here*Xs(ii)*L;
                    Vb      = Vb + Ws(ii)*L*b_here;
                    Mb_area = Mb_area+ Ws(ii)*L*b_here*Xs(ii)*L;
                end
                if Va==0
                    cx   = Mb_area/Vb;
                else
                    cx   = Ma_area/Va;
                end
            end
        end

        mass = V*Rho;

        Ix = 0;
        Iy = 0;
        Iz = 0;
        for ii=2:nGauss-1
            a_here = a(Xs(ii));
            b_here = b(Xs(ii));
            Iy     = Iy+Ws(ii)*L*((Xs(ii)*L-cx)^2*pi*a_here*b_here+pi*a_here*b_here^3/4);
            Iz     = Iz+Ws(ii)*L*((Xs(ii)*L-cx)^2*pi*a_here*b_here+pi*a_here^3*b_here/4);
            Ix     = Ix+Ws(ii)*L*(pi*a_here*b_here^3/4+pi*a_here^3*b_here/4);
        end
    end
    Ix = Ix*Rho;
    Iy = Iy*Rho;
    Iz = Iz*Rho;

    M   = double([Ix 0 0 0 0 0;0 Iy 0 0 0 0;0 0 Iz 0 0 0;...
                   0 0 0 mass 0 0;0 0 0 0 mass 0;0 0 0 0 0 mass]);

else % L==0
    M = zeros(6,6);
    cx = 0;
end

end