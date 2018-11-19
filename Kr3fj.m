function [ velEfetuador ] = Kr3fj( KR3, velJuntas )
    
    
    Jac = KR3.Jacobiana;
    thetaConfig = KR3.ThetaConfig + KR3.MasterPos;
    
    velEfetuador = double(Jac(thetaConfig(1),thetaConfig(2),thetaConfig(3),thetaConfig(4),thetaConfig(5),thetaConfig(6)))*velJuntas;
    
end

