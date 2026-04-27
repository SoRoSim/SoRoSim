function [Omega, dOmega_dq] = OmegaZanna6(xi1, xi2, xi3, phi_xi1, phi_xi2, phi_xi3, h)
% 6th-order Magnus/Zanna segment generator
%
% Inputs:
%   xi1,xi2,xi3       : 6x1
%   phi_xi1,phi_xi2,phi_xi3 : 6xnq
%   h                 : scalar
%
% Outputs:
%   Omega             : 6x1
%   dOmega_dq         : 6xnq

    % adjoint matrices of xi's
    Ad1 = dinamico_adjoint(xi1);
    Ad2 = dinamico_adjoint(xi2);
    Ad3 = dinamico_adjoint(xi3);

    % ---------- A1 ----------
    A1  = (5/18)*xi1 + (4/9)*xi2 + (5/18)*xi3;
    dA1 = (5/18)*phi_xi1 + (4/9)*phi_xi2 + (5/18)*phi_xi3;

    % ---------- basic brackets ----------
    B12 = Ad1*xi2;
    B13 = Ad1*xi3;
    B23 = Ad2*xi3;

    % dB12 = ad_xi1(phi_xi2) - ad_xi2(phi_xi1)
    dB12 = Ad1*phi_xi2 - Ad2*phi_xi1;
    dB13 = Ad1*phi_xi3 - Ad3*phi_xi1;
    dB23 = Ad2*phi_xi3 - Ad3*phi_xi2;

    % ---------- A2 ----------
    c2  = -sqrt(15)/108;
    A2  = c2*(2*B12 + B13 + 2*B23);
    dA2 = c2*(2*dB12 + dB13 + 2*dB23);

    % ---------- nested brackets ----------
    AdB12 = dinamico_adjoint(B12);
    AdB23 = dinamico_adjoint(B23);

    T1 = Ad1*B12;   % [xi1,[xi1,xi2]]
    T2 = Ad1*B23;   % [xi1,[xi2,xi3]]
    T3 = Ad2*B12;   % [xi2,[xi1,xi2]]
    T4 = Ad2*B23;   % [xi2,[xi2,xi3]]
    T5 = Ad3*B12;   % [xi3,[xi1,xi2]]
    T6 = Ad3*B23;   % [xi3,[xi2,xi3]]

    % d[ a , B ] = ad_a(dB) - ad_B(da)
    dT1 = Ad1*dB12 - AdB12*phi_xi1;
    dT2 = Ad1*dB23 - AdB23*phi_xi1;
    dT3 = Ad2*dB12 - AdB12*phi_xi2;
    dT4 = Ad2*dB23 - AdB23*phi_xi2;
    dT5 = Ad3*dB12 - AdB12*phi_xi3;
    dT6 = Ad3*dB23 - AdB23*phi_xi3;

    % ---------- A3 ----------
    c3  = 1/1296;
    A3  = c3*(-11*T1 + 29*T2 - 80*T3 + 80*T4 - 29*T5 + 11*T6);
    dA3 = c3*(-11*dT1 + 29*dT2 - 80*dT3 + 80*dT4 - 29*dT5 + 11*dT6);

    % ---------- final ----------
    h2 = h*h;
    h3 = h2*h;

    Omega     = h*A1 + h2*A2 + h3*A3;
    dOmega_dq = h*dA1 + h2*dA2 + h3*dA3;
end