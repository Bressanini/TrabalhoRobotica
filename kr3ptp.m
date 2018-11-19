function [ ] = Kr3ptp(KR3,X, Y, Z, A, B, C )

    
    KR3.ThetaConfig = [0,0,0,0,0,0];
     
    Pos = [X,Y,Z];
    Ori = [A,B,C];
    res = 60;
    CD = Kr3ik(KR3,Pos,Ori)

    
    range = [(CD(1)-KR3.ThetaConfig(1)),CD(2)-KR3.ThetaConfig(2),CD(3)-KR3.ThetaConfig(3),CD(4)-KR3.ThetaConfig(4),CD(5)-KR3.ThetaConfig(5),CD(6)-KR3.ThetaConfig(6)]/res;
    
    for i=1:res
        cla
        Kr3plot(KR3, [i*range(1),i*range(2),i*range(3),i*range(4),i*range(5),i*range(6)] )
        pause(0.00001)
    end
    
    
    
   

end

