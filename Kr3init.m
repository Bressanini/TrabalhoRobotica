function [KR3] = Kr3init(KR3, DHtable, MasterPosition, MBase, MTool)

        %{
        DH_table(:,1) = theta;
        DH_table(:,2) = d;
        DH_table(:,3) = a;
        DH_table(:,4) = alfa;
        %}

        KR3.DHtable = DHtable;
        KR3.MasterPos = MasterPosition;
        KR3.Base = MBase;
        KR3.Tool = MTool;
        
        theta  = DHtable(:,1);
        d  = DHtable(:,2);
        a = DHtable(:,3);
        alfa = DHtable(:,4);
            
        syms theta1 theta2 theta3 theta4 theta5 theta6

        syms At(Theta,D,A,Alfa)

        At(Theta, D, A, Alfa) = [cos(Theta)      -sin(Theta)*cos(Alfa)    sin(Theta)*sin(Alfa)       A*cos(Theta);
                                sin(Theta)      cos(Theta)*cos(Alfa)     -cos(Theta)*sin(Alfa)      A*sin(Theta);
                                0               sin(Alfa)                cos(Alfa)                  D;
                                0               0                        0                          1];

        %ABase = eye(4);
        %ATool = eye(4);

        %theta1 = 0;theta2 = 0;theta3 = 0;theta4 = 0;theta5 = 0;theta6 = 0;
        %---------------Matrizes de transformção-------------- 
        AW = At(0,0,0,pi);             
        At1 = At(theta1 + theta(1),d(1),a(1),alfa(1));
        At2 = At(theta2 + theta(2),d(2),a(2),alfa(2));
        At3 = At(theta3 + theta(3),d(3),a(3),alfa(3)); 
        At4 = At(theta4 + theta(4),d(4),a(4),alfa(4));  
        At5 = At(theta5 + theta(5),d(5),a(5),alfa(5));  
        At6 = At(theta6 + theta(6),d(6),a(6),alfa(6));    
        %-----------------------------------------------------

        %----------------Transformações-----------------------
        T0_1(theta1) = AW*MBase*At1;
        T0_2(theta1,theta2) = AW*MBase*At1*At2;
        T0_3(theta1,theta2,theta3) = AW*MBase*At1*At2*At3;
        T0_4(theta1,theta2,theta3,theta4) = AW*MBase*At1*At2*At3*At4;
        T0_5(theta1,theta2,theta3,theta4,theta5) = AW*MBase*At1*At2*At3*At4*At5;
        T0_6(theta1,theta2,theta3,theta4,theta5,theta6) = AW*MBase*At1*At2*At3*At4*At5*At6*MTool;
        %------------------------------------------------------
        
        T(:,:,1) = T0_1;
        T(:,:,2) = T0_2;
        T(:,:,3) = T0_3;
        T(:,:,4) = T0_4;
        T(:,:,5) = T0_5;
        T(:,:,6) = T0_6;

        %---------------------------Jacobiana----------------------------------
        x(theta1,theta2,theta3,theta4,theta5,theta6) = T(1,4,6);
        y(theta1,theta2,theta3,theta4,theta5,theta6) = T(2,4,6);
        z(theta1,theta2,theta3,theta4,theta5,theta6) = T(3,4,6);


        Jac(theta1,theta2,theta3,theta4,theta5,theta6) = [  diff(x,theta1),diff(x,theta2),diff(x,theta3),diff(x,theta4),diff(x,theta5),diff(x,theta6);
                                                            diff(y,theta1),diff(y,theta2),diff(y,theta3),diff(y,theta4),diff(y,theta5),diff(y,theta6);
                                                            diff(z,theta1),diff(z,theta2),diff(z,theta3),diff(z,theta4),diff(z,theta5),diff(z,theta6);
                                                            0,             T(1,3,1)      ,T(1,3,2)      ,T(1,3,3)      ,T(1,3,4)      ,T(1,3,5);
                                                            0,             T(2,3,1)      ,T(2,3,2)      ,T(2,3,3)      ,T(2,3,4)      ,T(2,3,5);
                                                            1,             T(3,3,1)      ,T(3,3,2)      ,T(3,3,3)      ,T(3,3,4)      ,T(3,3,5)];

        %----------------------------------------------------------
        KR3.T0_n = T;
        KR3.Jacobiana = Jac;
        KR3.ThetaConfig = [0,0,0,0,0,0];
end

