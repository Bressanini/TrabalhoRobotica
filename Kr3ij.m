function [ velJuntas ] = Kr3ij( KR3, velEfetuador )
    
    %KR3 = kuka
    %velEfetuador = [0;-615;0;0;0;1];
    Jac = KR3.Jacobiana;

    thetaConfig = KR3.ThetaConfig + KR3.MasterPos;
    Jac_num = double(Jac(thetaConfig(1),thetaConfig(2),thetaConfig(3),thetaConfig(4),thetaConfig(5),thetaConfig(6)));
    
    JacInv = Jac_num^-1;
    
    try
    velJuntas = JacInv*velEfetuador;
    catch
        error('As juntas de orientação estão numa posição singular ou o efetuador está esticado e limitado em liberdade.')
    end
    %{
    JacPos = eye(6);
    JacOri = eye(6);
    JacPos(1:3,:) = Jac_num(1:3,:);
    JacOri(4:6,:) = Jac_num(4:6,:);
    
    JacPosInv = JacPos^-1;
    JacOriInv = JacOri^-1;
    
    
    LinJuntas = JacPosInv*velEfetuador;
    RotJuntas = JacOriInv*velEfetuador;
    
    velJuntas(1:3) = LinJuntas(1:3);
    velJuntas(4:6) = RotJuntas(4:6);
    
    %}
    
end

