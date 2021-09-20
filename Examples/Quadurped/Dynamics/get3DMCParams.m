function mc3D = get3DMCParams()
mc3D = struct(...
        'bodyCoM',  zeros(3,1),...
        'hipLinkCoM', [0, 0.016, -0.02]',...
        'kneeLinkCoM',[0, 0, -0.061]',...
        'abadLinkCoM', [0, 0.036, 0]',...
        ...
        'bodyMass', 3.3,...
        'hipLinkMass' , 0.634,...
        'kneeLinkMass' , 0.064,...
        'abadLinkMass' , 0.54,...
        ... % link rotational inertia w.r.t. its CoM expressed in its own frame
        'bodyRotInertia' , [11253, 0, 0; 0, 36203, 0; 0, 0, 42673]*1e-6,...          % trunk rotational inertia
        'hipRotInertia' , [1983, 245, 13; 245, 2103, 1.5; 13, 1.5, 408]*1e-6,...     % hip rotational inertia
        'kneeRotInertia' , [245, 0, 0; 0, 248, 0; 0, 0, 6]*1e-6,...                  % knee rotational inertia
        'abadRotInertia' , [381, 58, 0.45; 58, 560, 0.95; 0.45, 0.95, 444]*1e-6,...  % abad rotational inertia
        ...
        'bodyLength' , 0.19 * 2,...
        'bodyWidth' , 0.049 * 2,...
        'bodyHeight' , 0.05 * 2,...
        ...
        'hipLinkLength' , 0.209,...
        'kneeLinkLength' , 0.195,...
        'abadLinkLength', 0.062,...
        ...
        'linkwidth', 0.03);
    mc3D.abadLoc{1} = [mc3D.bodyLength, mc3D.bodyWidth, 0]'/2;
    mc3D.abadLoc{2} = [mc3D.bodyLength, -mc3D.bodyWidth, 0]'/2;
    mc3D.abadLoc{3} = [-mc3D.bodyLength, mc3D.bodyWidth, 0]'/2;
    mc3D.abadLoc{4} = [-mc3D.bodyLength, -mc3D.bodyWidth, 0]'/2;
    
    mc3D.hipLoc = {[0, mc3D.abadLinkLength, 0]'};
    mc3D.kneeLoc = [0, 0, -mc3D.hipLinkLength]';
end