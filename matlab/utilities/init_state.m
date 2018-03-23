function X = init_state(sel_traj)
%% Initialization
    X = zeros(13,1);

    %% Defining initial quaternions
    X(7) = 0;
    X(8) = 0;
    X(9) = 0;
    X(10) = 1;
    
    %% Defining initial position based on trajectory
switch sel_traj
    case 'experimental_try'
        X(1) = 1.9;
        X(2) = -1.5;
        X(3) = 1;
    case 'Circle'
        X(1) = 1;
        X(2) = 0;
        X(3) = 1;    
    case 'Leminiscate'        
        X(1) = 1;
        X(2) = 0;
        X(3) = 1;
    case 'Eight Curve'
        X(1) = 0;
        X(2) = 0;
        X(3) = 1;  
    case 'Ellipse'
        X(1) = 0;
        X(2) = 1;
        X(3) = 1;          
    case 'Cornoid'
        X(1) = 1;
        X(2) = 0;
        X(3) = 1;                  
    case 'Astroid'
        X(1) = 4;
        X(2) = 0;
        X(3) = 1;   
    case 'Vertical Circle'
        X(1) = 1;
        X(2) = 0;
        X(3) = 3;  
    case 'Bicorn'
        X(1) = 0;
        X(2) = 1;
        X(3) = 1;
    case 'Butterfly Curve'
        X(1) = 0;
        X(2) = 0.7183;
        X(3) = 1;
    case 'TakeOff'
        X(1) = 0;
        X(2) = 0;
        X(3) = 2;         
    case 'Spiral Circle'
        X(1) = 1;
        X(2) = 0;
        X(3) = 0.2;   
    case 'Circle_MultiDimensional'
        X(1) = 1;
        X(2) = 0;
        X(3) = 3;           
    case 'Back&Forth'
        X(1) = 0;
        X(2) = 0;
        X(3) = 1;                   
    otherwise
        display('Invalid initial state');
 
end
