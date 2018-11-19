function [ ] = Kr3plot( KR3, Vtheta )
    
   
    syms theta1 theta2 theta3 theta4 theta5 theta6


    T0_1(theta1) = KR3.T0_n(:,:,1);
    T0_2(theta1,theta2) = KR3.T0_n(:,:,2);
    T0_3(theta1,theta2,theta3) = KR3.T0_n(:,:,3);
    T0_4(theta1,theta2,theta3,theta4) = KR3.T0_n(:,:,4);
    T0_5(theta1,theta2,theta3,theta4,theta5) = KR3.T0_n(:,:,5);
    T0_6(theta1,theta2,theta3,theta4,theta5,theta6) = KR3.T0_n(:,:,6);


    Mrt = eye(4);  
    Mrt(3,4) = -20;
    
 
    %-----Calculo----------------------

    X(1) = 0;
    Y(1)= 0;
    Z(1) = 0;

    
    M(:,:,1) = double(T0_1(Vtheta(1)+KR3.MasterPos(1)));
    M(:,:,2) = double(T0_2(Vtheta(1)+KR3.MasterPos(1),Vtheta(2)+KR3.MasterPos(2)));
    M(:,:,3) = double(T0_3(Vtheta(1)+KR3.MasterPos(1),Vtheta(2)+KR3.MasterPos(2),Vtheta(3)+KR3.MasterPos(3)));
    M(:,:,4) = double(T0_4(Vtheta(1)+KR3.MasterPos(1),Vtheta(2)+KR3.MasterPos(2),Vtheta(3)+KR3.MasterPos(3),Vtheta(4)+KR3.MasterPos(4)));
    M(:,:,5) = double(T0_5(Vtheta(1)+KR3.MasterPos(1),Vtheta(2)+KR3.MasterPos(2),Vtheta(3)+KR3.MasterPos(3),Vtheta(4)+KR3.MasterPos(4),Vtheta(5)+KR3.MasterPos(5)));
    M(:,:,6) = double(T0_6(Vtheta(1)+KR3.MasterPos(1),Vtheta(2)+KR3.MasterPos(2),Vtheta(3)+KR3.MasterPos(3),Vtheta(4)+KR3.MasterPos(4),Vtheta(5)+KR3.MasterPos(5),Vtheta(6)+KR3.MasterPos(6)));


    for i = 2:7

        X(i) = M(1,4,i-1);
        Y(i)= M(2,4,i-1);
        Z(i) = M(3,4,i-1);

    end
   
    plot3(X,Y,Z,'Linewidth', 5)
    hold on
    scatter3(double(M(1,4,6)),double(M(2,4,6)),double(M(3,4,6)),'*')

    for i=1:5
        [Xc, Yc, Zc] = cylinder(10);
        t = hgtransform;
        h = surface(Xc, Yc, 40*Zc);

        set(h, 'Parent', t);    
        surface(Xc, Yc, 15*Zc);
        set(t,'Matrix',double(M(:,:,i))*Mrt);

        hold on
    end






    grid
    
    T(:) = double(M(3,4,:));
    xlim([min(double(M(1,4,:)))-100,max(double(M(1,4,:)))+100])
    ylim([min(double(M(2,4,:)))-100,max(double(M(2,4,:)))+100])
    zlim([min([T,0])-100,max(double(M(3,4,:)))+100])
    
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    view([1,2,1])
    hold on
    grid on

    
    M(:,:,6)
end

