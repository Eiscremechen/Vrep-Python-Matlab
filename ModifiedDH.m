function [T] = ModifiedDH(alpha,a,d,theta)
    T_z_theta=  [cos(theta),-sin(theta),0,0;
                sin(theta),cos(theta),0,0;
                0,0,1,0;
                0,0,0,1];
    T_z_d=  [1,0,0,0;
            0,1,0,0;
            0,0,1,d;
            0,0,0,1];
    T_x_a= [1,0,0,a;
            0,1,0,0;
            0,0,1,0;
            0,0,0,1];
    T_x_alpha=  [1,0,0,0;
                 0,cos(alpha),-sin(alpha),0;
                 0,sin(alpha),cos(alpha),0;
                 0,0,0,1];
    digits(6);
    T=vpa(T_x_alpha*T_x_a*T_z_d*T_z_theta,4);
%     T=vpa(T_x_alpha*T_x_a*T_z_d,4);


end