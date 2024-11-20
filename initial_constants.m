function constants=initial_constants()
    
     % Constants    
    g=9.81;
    m=1500;
    Iz=3000;
    Cf=38000;
    Cr=66000;    
    rho=1.225;
    lf=2;
    lr=3;
    Ts=0.02;
    mju=0.02;
    
    outputs=4;
    inputs=2;
    
    % Choose your trajectory (1,2,3)
    trajectory=1;

    delay=0;
    if trajectory==1
        hz=10;
        time_length=60;
        Q=[1 200 50 50];
        S=[1 0 0 0;0 200 0 0;0 0 50 0;0 0 0 50];
        R=[100 1];
    elseif trajectory==2
        hz=10;
        time_length=140;
        Q=[100 100 1 1];
        S=[100 100 1 1];
        R=[100 1];
    else
        hz=10;
        first_section=14;
        other_sections=14;
        time_length=first_section+other_sections*10;
        delay=zeros(1,12);
        for dly = 2:length(delay)
            delay(dly)=first_section+(dly-2)*other_sections;
        end
        Q=[1 1000 50 50];
        %Q=[100 0 0 0;0 100 0 0;0 0 50 0;0 0 0 50];
        S=[100 0 0 0;0 100000 0 0;0 0 5000 0;0 0 0 5000];
        %S=[100 0 0 0;0 100 0 0;0 0 50 0;0 0 0 50];
        R=[100 1];   
    
    end
    
    
    keySet = {'g', 'm', 'Iz', 'Cf', 'Cr', 'rho', 'lf', 'lr', 'Ts', 'mju', 'outputs', 'inputs', 'hz', 'trajectory', 'Q', 'S', 'R','time_length','delay'};
    constants_list={g m Iz Cf Cr rho lf lr Ts mju outputs inputs hz trajectory Q S R time_length delay};
    constants = containers.Map(keySet,constants_list);

end