function [ THConfig ] = Kr3ik( KR3, Pos, Ori)
    
    theta_atual = KR3.ThetaConfig;
    Efetuador = [KR3.Tool(1,4),KR3.Tool(2,4),KR3.Tool(3,4) - KR3.DHtable(6,2)];
    
    %Pos = [400, 100, 400];
    %Ori = [deg2rad(90),deg2rad(90),deg2rad(0)];

    %x_for = Pos(1);
    %y_for = Pos(2);
    %z_for = Pos(3);
    
    alfa = Ori(1);  %Rotação em x
    beta = Ori(2);    %Rotação em y
    gama = Ori(3);    %Rotação em z

    R0_6 = [cos(beta)*cos(gama),    sin(alfa)*sin(beta)*cos(gama)+cos(alfa)*sin(gama),  -cos(alfa)*sin(beta)*cos(gama)+sin(alfa)*sin(gama);
            -cos(beta)*sin(gama),   -sin(alfa)*sin(beta)*sin(gama)+cos(alfa)*cos(gama), cos(alfa)*sin(beta)*sin(gama)+sin(alfa)*sin(gama);
            sin(beta),              -sin(alfa)*cos(beta),                               cos(alfa)*cos(beta)];
    

    
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %------------------------------Posição---------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
   
    %---Definir posição do punho de acordo com a orientação e ferramenta---
    punho = Pos - Efetuador*R0_6;
    x = punho(1);
    y = punho(2);
    z = punho(3);
    %----------------------------------------------------------------------
    
    
    %----------------------Definindo Theta1:-------------------------------
    %---Caso as posições x e y do punho estiverem em 0, existe um ponto de
    %singularidade, onde a configuração de theta1 pode ser qualquer uma. O
    %valor de orientação será ajeitado no cálculo de theta(4-6)
    %----------------------------------------------------------------------
    
    if x~=0 || y~=0 
        theta1(1) = atan2(y,x);
        theta1(2) = atan2(y,x)+pi;    
    
    else
        theta1(1) = -theta_atual(1);
        theta1(2) = -theta_atual(1);
    end
    
    %----------------------------------------------------------------------
    %-----------------------Definindo Theta 2 e 3:-------------------------
    %----Mudanaça do plano de ação:----------------------------------------
    %y_ = 0;
    z_ = z - 345;

    r2 = 260;
    
    r3 = sqrt(260^2+20^2);
    
    %-----Para primeira opção de theta1 (Mesmo quadrante dacoordenada X,Y 
    % desejada:------------------------------------------------------------
    %x_ = x/cos(theta1(1))-20;

    x_ = sqrt(x^2+y^2) - 20;
    C3_ = (x_^2 +z_^2 - r2^2 - r3^2)/(2*r2*r3);

    theta3(1) = acos(C3_);
    theta2(1) = atan(z_/x_) - atan(r3*sin(theta3(1))/(r2+r3*cos(theta3(1))));

    theta3(2) = -acos(C3_);
    theta2(2) = atan(z_/x_) - atan(r3*sin(theta3(2))/(r2+r3*cos(theta3(2))));
    

    
    %-----Para segunda opção de theta1:------------------------------------
    %x_ = x/cos(theta1(2))-20;
    x_ = -sqrt(x^2+y^2) - 20;
    C3_ = (x_^2 +z_^2 - r2^2 - r3^2)/(2*r2*r3);

    theta3(3) = acos(C3_);
    theta2(3) = pi+atan(z_/x_) - atan(r3*sin(theta3(3))/(r2+r3*cos(theta3(3))));

    theta3(4) = -acos(C3_);
    theta2(4) = pi+atan(z_/x_) - atan(r3*sin(theta3(4))/(r2+r3*cos(theta3(4))));
    
    %----------------------------------------------------------------------
    %---------Selecionar valores reais-------------------------------------
    %--Caso alguma combinação de angulos for formadas por valores
    %complexos, deve-se anular este valor, pois não está dentro do espaço
    %de trabalho.----------------------------------------------------------
    %----------------------------------------------------------------------
    j=1;
    %i=3;
    
    for i=1:2
        
        if round(imag(theta1(1)),2) == 0 && round(imag(-theta2(i)),2) == 0 && round(imag(-(theta3(i)- atan(20/260))),2) == 0
            comb(:,j) = [theta1(1),-theta2(i),-(theta3(i)- atan(20/260))];
            j = j+1;
        end
        
    end
    
    for i=3:4
        
        if round(imag(theta1(2)),2) == 0 && round(imag(-theta2(i)),2) == 0 && round(imag(-(theta3(i)- atan(20/260))),2) == 0
            comb(:,j) = [theta1(2),-theta2(i),-(theta3(i)- atan(20/260))];
            j = j+1;
        end
        
    end
    
    
    %----------------Selecionar combinação ideal --------------------------
    %-- A combinação ideal será baseada no fato de escolher a menor soma
    %das distâncias angular das possíveis combinações de ângulos do ponto 
    %destino e o ponto atual.----------------------------------------------
    try
    
    ind=1;
    for i=1:length(comb(1,:))
        if [comb(1,i), comb(2,i), comb(3,i)] <= [deg2rad(170), deg2rad(50), deg2rad(155)] & [comb(1,i), comb(2,i), comb(3,i)] >= [deg2rad(-170), deg2rad(-170), deg2rad(-110)]
        CombPOS(1:3,ind) = comb(:,i);
        end
    end
    
    V(:) = abs(CombPOS(1,:)-theta_atual(1)) + abs(CombPOS(2,:)-theta_atual(2)) + abs(CombPOS(3,:)-theta_atual(3));
    
    for i=1:length(V)
        
        if V(i) == min(V)
            
            Theta123(:) = CombPOS(:,i);
        end
        
    end
    catch
        error('As coordenadas escolhidas estão fora do espaço de trabalho.');
        
    end
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %-----------------Orientação:------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    
    theta1 = -Theta123(1);
    theta2 = Theta123(2);
    theta3 = Theta123(3);
    
    d_(1) = -345;
    d_(2) = 0;
    d_(3) = 0;
    d_(4) = -260;
    d_(5) = 0;
    d_(6) = -75;

    a_(1) = 20;
    a_(2) = 260;
    a_(3) = 20;
    a_(4) = 0;
    a_(5) = 0;
    a_(6) = 0;

    alfa_(1) = deg2rad(90);
    alfa_(2) = deg2rad(0);
    alfa_(3) = deg2rad(90);
    alfa_(4) = deg2rad(-90);
    alfa_(5) = deg2rad(90);
    alfa_(6) = deg2rad(180);
    

    syms A(theta,d,a,alfa)

    A(theta, d, a, alfa) = [cos(theta)      -sin(theta)*cos(alfa)    sin(theta)*sin(alfa)       a*cos(theta);
                            sin(theta)      cos(theta)*cos(alfa)     -cos(theta)*sin(alfa)      a*sin(theta);
                            0               sin(alfa)                cos(alfa)                  d;
                            0               0                        0                          1];
  
    AW = A(0,0,0,pi);             
    At1 = A(theta1,d_(1),a_(1),alfa_(1));
    At2 = A(theta2-pi/2,d_(2),a_(2),alfa_(2));
    At3 = A(theta3,d_(3),a_(3),alfa_(3));                    


    T3 = AW*At1*At2*At3;
    
    T = Kr3fk(KR3, [theta1,theta2,theta3,0,0,0]);
    T3 = T(:,:,3);
    
    %alfa = -Ori(1);
    %beta = -Ori(2);
    %gama = Ori(3);
    
    %Efetuador = [0,0,75];
    
    %R0_6 = [cos(beta)*cos(gama),    sin(alfa)*sin(beta)*cos(gama)+cos(alfa)*sin(gama),  -cos(alfa)*sin(beta)*cos(gama)+sin(alfa)*sin(gama);
    %        -cos(beta)*sin(gama),   -sin(alfa)*sin(beta)*sin(gama)+cos(alfa)*cos(gama), cos(alfa)*sin(beta)*sin(gama)+sin(alfa)*sin(gama);
    %        sin(beta),              -sin(alfa)*cos(beta),                               cos(alfa)*cos(beta)];
 
    %-----------------------------
    alfa = Ori(1);
    beta = Ori(2);
    gama = Ori(3);
    
    Rotx = [1   0           0;
        0   cos(alfa)   -sin(alfa);
        0   sin(alfa)   cos(alfa)];
    
    Roty = [cos(beta)   0   sin(beta);
        0           1   0;
        -sin(beta)  0   cos(beta)];
       
    Rotz = [cos(gama)   -sin(gama)  0;
        sin(gama)   cos(gama)   0;
        0           0           1];
    
    R0_6 = Rotx*Roty*Rotz;
    
    %R0_3 = double(M(1:3,1:3,3));
    R0_3 = T3(1:3,1:3); 
    R4_6 = R0_3^-1*R0_6;
    
    %----------------------------
 
    
    %----------------------------------------
    
    t5(1) = atan2(sqrt(R4_6(1,3)^2 + R4_6(2,3)^2), -R4_6(3,3));
    t5(2) = atan2(-sqrt(R4_6(1,3)^2 + R4_6(2,3)^2),-R4_6(3,3));
    %{
    if t5(1) == 0
     
        t4(1) = 0;
        t6(1) = atan2(R4_6(1,2),R4_6(2,2))
        
        t4(2) = pi;
        t6(2) = atan2(-R4_6(1,2),-R4_6(2,2))
        
    else
   %}
    if t5(1)>0 && t5(1)<pi
        t4(1) = atan2(-R4_6(2,3),-R4_6(1,3));
        t6(1) = atan2(-R4_6(3,2),-R4_6(3,1));
        
        t4(2) = atan2(R4_6(2,3),R4_6(1,3));
        t6(2) = atan2(R4_6(3,2),R4_6(3,1));
    else
        t4(1) = atan2(R4_6(2,3),R4_6(1,3));
        t6(1) = atan2(R4_6(3,2),R4_6(3,1));
        
        t4(2) = atan2(-R4_6(2,3),-R4_6(1,3));
        t6(2) = atan2(-R4_6(3,2),-R4_6(3,1));

    end    
        
    %end
    
    %THConfig(4:6) = [t4(1),t5(1),t6(1)];
    %round(rad2deg(Theta123),6); 
    %if (abs(t4(1)-theta_atual(4)
    THConfig(1:3) = Theta123(:);
    
    degTH = rad2deg(THConfig);
    
    for i=1:2
        if degTH(1)<=170 && degTH(1)>=-170 && degTH(2)<=50 && degTH(2)>=-170 && degTH(3)<=155 && degTH(3)>=-110 && t4(i)<=120 && t4(i)>=-120 && t5(i)<=120 && t5(i)>=-120 && t6(i)<=350 && t6(i)>=-350
            combOR(1:3,i) = [t4(i),t5(i),t6(i)];
        else
        error('Os limites das juntas não permitem atingir o ponto e orientação desejado');
        end  
    end
    
    try
    if abs(combOR(:,1)-[theta_atual(4),theta_atual(5),theta_atual(6)])<abs(combOR(:,2)-[theta_atual(4),theta_atual(5),theta_atual(6)]) 
   
        Theta456 = [double(combOR(1,1)),double(combOR(2,1)),double(combOR(3,1))];
    else
        Theta456 = [double(combOR(1,2)),double(combOR(2,2)),double(combOR(3,2))];
    end
    catch
        Theta456 = [double(combOR(1,1)),double(combOR(2,1)),double(combOR(3,1))];
    end
    
    THConfig(4:6) = Theta456(:);
    THConfig(1) =  -THConfig(1)
    
    %-------------Retorno:-----------
    %i=2;
    %theta4 = t4(i);
    %theta5 = t5(i);
    %theta6 = t6(i);
   
    %R4_6;
    %T4_6teste = [cos(theta4)*cos(theta5)*cos(theta6) - sin(theta4)*sin(theta6), cos(theta6)*sin(theta4) + cos(theta4)*cos(theta5)*sin(theta6),      -cos(theta4)*sin(theta5);
                %cos(theta4)*sin(theta6) + cos(theta5)*cos(theta6)*sin(theta4),  cos(theta5)*sin(theta4)*sin(theta6) - cos(theta4)*cos(theta6),      -sin(theta4)*sin(theta5);
                %-cos(theta6)*sin(theta5),                                      -sin(theta5)*sin(theta6),                                            -cos(theta5)];
    %i=1        
    %TH = [round(rad2deg(Theta123(1)),6),round(rad2deg(Theta123(2)),6),round(rad2deg(Theta123(3)),6),rad2deg(t4(i)),rad2deg(t5(i)),rad2deg(t6(i))]; 


end

