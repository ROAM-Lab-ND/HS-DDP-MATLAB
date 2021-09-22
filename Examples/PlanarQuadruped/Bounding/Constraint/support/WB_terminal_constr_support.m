function WB_terminal_constr_support(WBMC2D)       
    q = sym('q', [7,1], 'real');
    qd = sym('qd', [7,1], 'real');
    x = [q; qd];
    
    ground = -0.404;
    Pos_ffoot = WBMC2D.getPosition(q, 3, [0,-WBMC2D.kneeLinkLength]');
    h_FTD = Pos_ffoot(2) - ground;
    hx_FTD = jacobian(h_FTD, x);
    hxx_FTD = hessian(h_FTD, x);
    
    Pos_bfoot = WBMC2D.getPosition(q, 5, [0,-WBMC2D.kneeLinkLength]');
    h_BTD = Pos_bfoot(2) - ground;
    hx_BTD = jacobian(h_BTD, x);
    hxx_BTD = hessian(h_BTD, x);
    
    folder = '/home/wensinglab/HL/Code/MHPC-project/MATLAB_v2/Examples/Bounding/Constraint/';
    fileName = 'Front_TouchDown_Constraint';
    matlabFunction(h_FTD, hx_FTD, hxx_FTD, 'file', strcat(folder, fileName), 'vars', {x});
    fileName = 'Back_TouchDown_Constraint';
    matlabFunction(h_BTD, hx_BTD, hxx_BTD, 'file', strcat(folder, fileName), 'vars', {x});    
    fprintf('Terminal constrait functions generated successfully!\n');
end