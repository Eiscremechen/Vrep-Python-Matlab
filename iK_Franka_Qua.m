function q_i = iK_Franka_Qua(q_0, q_1, q_2, q_3, x, y, z)
% function q_i = iK_Franka (2, 0, 0, 0, 0.5, 0, 0.5)

        
    l_1=0.330;
    l_2=0.3160;
    l_3=0.0825;
    l_4=0.3840;
    l_5=0.088;

    radian1=pi/180;

    lim1_min = -2.7437; lim1_max = 2.7437; 
    lim2_min = -1.7837; lim2_max = 1.7837; 
    lim3_min = -2.9007; lim3_max = 2.9007; 
    lim4_min = -3.0421; lim4_max = -0.1518; 
    lim5_min = -2.8065; lim5_max = 2.8065; 
    lim6_min = 0.5445; lim6_max = 4.5169; 
    lim7_min = -3.0159; lim7_max = 3.0159;

    
        %            zq;    zL;     xL;     xq
    L(1) = Link([0,     l_1,    0,      0],'modified');
    L(1).qlim=[lim1_min,lim1_max];
    
    L(2) = Link([0,     0,      0,      -pi/2],'modified');
    L(2).qlim=[lim2_min,lim2_max];
    
    L(3) = Link([0,     l_2,    0,      pi/2],'modified');
    L(3).qlim=[lim3_min,lim3_max];
    
    L(4) = Link([0,     0,      l_3,    pi/2],'modified');
    L(4).qlim=[lim4_min,lim4_max];
    
    L(5) = Link([0,     l_4,    -l_3,   -pi/2],'modified');
    L(5).qlim=[lim5_min,lim5_max];
    
    L(6) = Link([0,     0,      0,      pi/2],'modified');
    L(6).qlim=[lim6_min,lim6_max];
    
    L(7) = Link([0,     0,      l_5,      pi/2],'modified');
    L(7).qlim=[lim7_min,lim7_max];


    Franka=SerialLink(L,'name','Franka');
    
    
    R_q= ...
        [1-2*(q_2^2+q_3^2),     2*(q_1*q_2-q_0*q_3),        2*(q_1*q_3+q_0*q_2),    0;
         2*(q_1*q_2+q_0*q_3),   1-2*(q_1^2+q_3^2),          2*(q_2*q_3-q_0*q_1),    0;
         2*(q_1*q_3-q_0*q_2),   2*(q_2*q_3+q_0*q_1),        1-2*(q_1^2+q_2^2),      0;
         0,                     0,                          0,                      1
        ];

    R_t=...
        [1,     0,      0,      x;
         0,     1,      0,      y;
         0,     0,      1,      z;
         0,     0,      0,      1
        
        ];

    T= R_t * R_q;
    
    for i = 1:3
        sum = (T(1,i)^2 + T(2,i)^2 + T(3,i)^2)^(0.5)
        T(1,i) = T(1,i) / sum
        T(2,i) = T(2,i) / sum
        T(3,i) = T(3,i) / sum
%         print (i, " Columns sum is ",sum )
    end



    q_i = Franka.ikine(T);
end