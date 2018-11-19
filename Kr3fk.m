function [ M ] = Kr3fk( KR3, Vtheta )
    
    syms theta1 theta2 theta3 theta4 theta5 theta6

    T0 = KR3.T0_n;
    
    
    T0_1(theta1) = T0(:,:,1);
    T0_2(theta1,theta2) = T0(:,:,2);
    T0_3(theta1,theta2,theta3) = T0(:,:,3);
    T0_4(theta1,theta2,theta3,theta4) = T0(:,:,4);
    T0_5(theta1,theta2,theta3,theta4,theta5) = T0(:,:,5);
    T0_6(theta1,theta2,theta3,theta4,theta5,theta6) = T0(:,:,6);
    
    M(:,:,1) = double(T0_1(Vtheta(1)+KR3.MasterPos(1)));
    M(:,:,2) = double(T0_2(Vtheta(1)+KR3.MasterPos(1),Vtheta(2)+KR3.MasterPos(2)));
    M(:,:,3) = double(T0_3(Vtheta(1)+KR3.MasterPos(1),Vtheta(2)+KR3.MasterPos(2),Vtheta(3)+KR3.MasterPos(3)));
    M(:,:,4) = double(T0_4(Vtheta(1)+KR3.MasterPos(1),Vtheta(2)+KR3.MasterPos(2),Vtheta(3)+KR3.MasterPos(3),Vtheta(4)+KR3.MasterPos(4)));
    M(:,:,5) = double(T0_5(Vtheta(1)+KR3.MasterPos(1),Vtheta(2)+KR3.MasterPos(2),Vtheta(3)+KR3.MasterPos(3),Vtheta(4)+KR3.MasterPos(4),Vtheta(5)+KR3.MasterPos(5)));
    M(:,:,6) = double(T0_6(Vtheta(1)+KR3.MasterPos(1),Vtheta(2)+KR3.MasterPos(2),Vtheta(3)+KR3.MasterPos(3),Vtheta(4)+KR3.MasterPos(4),Vtheta(5)+KR3.MasterPos(5),Vtheta(6)+KR3.MasterPos(6)));
    
    %{
    T0_(:,:,1) = double(T0_1(Vtheta(1)));
    T0_(:,:,2) = double(T0_2(Vtheta(1),Vtheta(2)));
    T0_(:,:,3) = double(T0_3(Vtheta(1),Vtheta(2),Vtheta(3)));
    T0_(:,:,4) = double(T0_4(Vtheta(1),Vtheta(2),Vtheta(3),Vtheta(4)));
    T0_(:,:,5) = double(T0_5(Vtheta(1),Vtheta(2),Vtheta(3),Vtheta(4),Vtheta(5)));
    T0_(:,:,6) = double(T0_6(Vtheta(1),Vtheta(2),Vtheta(3),Vtheta(4),Vtheta(5),Vtheta(6)));
    %}
    
end

