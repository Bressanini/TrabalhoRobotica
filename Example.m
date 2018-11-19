%% Utilização da toolbox BBKuka
% Esta toolbox tem o objetivo de abordar e calcular as propriedades de um 
%Manipulador robótico de juntas rotativas com 6 graus de liberdade. O
%modelo específico é o manipulador Kuka KR3 Agilus.

%% Criando e incializando o kuka KR3
% Kuka com café, na sua inicialização, deve apresentar: Um objeto KukaKR3; 
% uma tabela 6x4 com os parâmetros de Denavit-Hatenberg na ordem theta, d, 
% a, alfa; e um vetor com a Mastering Position de acordo com a tabela DH
% fornecida.
        
    theta_ = [0 -pi/2 0 0 0 0];
    d_ = [-345, 0, 0, -260, 0, -75];
    a_ = [20, 260, 20, 0, 0, 0];
    alfa_ = [pi/2, 0, pi/2, -pi/2, pi/2, pi];
    MPosition = [0,pi/2,-pi/2,0,0,0];
    
    DH_table(:,1) = theta_;
    DH_table(:,2) = d_;
    DH_table(:,3) = a_;
    DH_table(:,4) = alfa_;
    
    MBase = eye(4);  %Base é o Mundo
    MTool = [1, 0,  0,  0;
            0,  1,  0,  0;
            0,  0,  1,  146.9;
            0,  0,  0,  1]; %Configuração de ferramenta da garra Festo HGPLE-14-30-3,1-DC-VCSC-G96
    %MTool = eye(4);    
    kuka = Kr3init(KukaKR3, DH_table, MPosition,MBase,MTool);
   
        
%% Cinemática direta
% A partir do objeto KukaKR3 criado, é possível calcular a cinemática
% direta do manipulador com a função Kr3fk. Para isto, deve-se fornecer o
% KukaKR3 e os ângulos das juntas.

    VTheta = [0,0,0,0,0,0];
    
    CinDireta = Kr3fk(kuka, VTheta);
    CinDiretaFlange = CinDireta(:,:,6)
    
%% Cinemática Inversa
%A Cinemática inversa pode ser obtida a partir do fornecimento de um
%KukaKR3, um vetor com as 3 coordenadas e um vetor com as rotações da
%ferramenta ao longo de X, Y e Z.

    Pos = [300,300,0];
    Ori = [0, pi/2, 0];
    
    CD = Kr3ik(kuka,Pos,Ori);
    
    Kr3fk(kuka, [CD(1),CD(2),CD(3),CD(4),CD(5),CD(6)] )
    
%% Jacobiana Direta
%Por meio da posição do robô e da velocidade de cada junta num vetor vertical, a função Kr3fj
%fornece a velocidade em x,y,z nos primeiros 3 valores e as velocidades
%angulares ao longo de x,y,z nos 3 últimos valores;
    
    
    kuka.ThetaConfig = [0,0,0,0,0,0];
    velJuntas = [0;1;0;0;0;0]; %rad/s
    Kr3fj(kuka, velJuntas)
    
    
%% Jacobiana Inversa    
%Por meio da função Kr3ij, a qual recebe o objeto KukaKR3 e a velocidade do
%efetuador, a velocidade das juntas pode ser calculada.

    kuka.ThetaConfig = [0,0,0,0,0.5,0];
    VelEfetuador = [0;-615;0;0;0;1];
    Kr3ij(kuka,VelEfetuador)
    Kr3fj(kuka,Kr3ij(kuka,VelEfetuador))


%% Plot do manipulador
%A função Kr3plot realiza a reprodução gráfica do manipulador por meio da
%posição das juntas e as características de KukaKR3.


    Pos = [370,0,300];
    Ori = [0, pi/2, 0];
    
    CD = Kr3ik(kuka,Pos,Ori);
    cla
    Kr3plot(kuka, [CD(1),CD(2),CD(3),CD(4),CD(5),CD(6)])
    
    
   
    kr3ptp(kuka,-300,300,0,deg2rad(0),deg2rad(180),deg2rad(0))

    


