classdef KukaKR3
   
    
    properties
        
        DHtable
        T0_n
        Jacobiana
        MasterPos
        
        Base
        Tool
        
        ThetaConfig
        
    end
    
    methods
        function r = soma(obj,t)
            r = t+1;            
        end
    end
    
end

