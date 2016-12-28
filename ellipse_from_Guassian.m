function [x_points, y_points] = ellipse_from_Guassian(mu_x, mu_y, covar_Mat)
% ELLIPSE_FROM_GAUSSIAN Accepts parameters for a Gaussian distribution
% and draws an ellipse of constant probability. First it obtains the
% eigenvalues and eigenvectors of the covariance matrix.  The eigenvectors
% give the direction of the ellipse.  The square roots of the eigenvalues
% give the dimension of the ellipse. A. Maida
[v, d] = eig(covar_Mat);
if (covar_Mat(1,1) > covar_Mat(2,2))
    angle = atan2(v(2,1),v(1,1));
    sqrt_eig_1 = sqrt(d(1,1));
    sqrt_eig_2 = sqrt(d(2,2));
else
    angle = atan2(v(2,2),v(1,2));
    sqrt_eig_1 = sqrt(d(2,2));
    sqrt_eig_2 = sqrt(d(1,1));
end
for i = 0:1:35,
    step = (i/36)*2*pi;
    x_points(i+1) = mu_x + (sqrt_eig_1*cos(step)*cos(angle) - sqrt_eig_2*sin(step)*sin(angle));
    y_points(i+1) = mu_y + (sqrt_eig_1*cos(step)*sin(angle) + sqrt_eig_2*sin(step)*cos(angle));
end
x_points(37) = x_points(1);
y_points(37) = y_points(1);
