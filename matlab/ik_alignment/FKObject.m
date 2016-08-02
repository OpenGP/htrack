%==========================================================================
%                                                                            
%   Dynamic 2D/3D Registration
%   by Sofien Bouaziz and Andrea Tagliasacchi and Mark Pauly
%
%   Copyright (C) 2013  LGG, EPFL.
%                                                                            
%==========================================================================
classdef FKObject < handle
    
properties
    n_segs   = 3;    % number of segments
    n_joints = 4;    % origin is not really a joint as it's always in ZERO
    n_pars   = 7;    % degree of freedom
    joint_thetas     % rotation angles
    joint_vs         % joint rotation vectors        
    joint_ps         % posed (w.r.t. thetas) joint positions    
    joint_offs       % offsets to move from joint to other    
    
    %--- Registration specific stuff
    points = nan;
    normals = nan;
    kdtree = nan;
    color = nan;
end

methods
    function this = FKObject()
        % TX TY YZ R0 R1 R2 R3
        this.joint_thetas = zeros(this.n_pars,1);
        this.joint_offs   = zeros(this.n_pars,3);
        this.joint_ps     = zeros(this.n_pars,3);
        this.joint_vs     = zeros(this.n_pars,3);
        
        %%% JOINT OFFSETS (straight finger)
        this.joint_offs(:,1)   = 1;
        this.joint_offs(1:3,:) = 0; 
        
        %%% JOINT VECTORS
        this.joint_vs(:,3) = 1; % all axis rotation point +z;
        this.joint_vs(1:3,:) = eye(3); % euclidean basis for translation

        %%% INSTANTIATE
        this.pose([0,0],[0,0,0]);
    end
       
    function pose(this, tran, rot)
        thetas(1:2)   = tran;
        thetas(4:6)   = rot;
        thetas([3,7]) = 0; % last joint does nothing
        this.joint_thetas = thetas; % cache parameters
        frame = makehgtform('translate',this.joint_thetas(1:3)); % root translation
               
        for i=4:this.n_pars
            h_ps = frame*[0,0,0,1]';
            this.joint_ps(i,:) = h_ps(1:end-1)';
            frame = frame * makehgtform('axisrotate', this.joint_vs(i,:), this.joint_thetas(i));
            frame = frame * makehgtform('translate',  this.joint_offs(i,:));
        end
    end
    
    function [effectors, parents] = sample(this, num_samples)
        effectors = zeros(0,3);
        parents   = zeros(0,1);
        
        for i=4:this.n_pars-1
            p1 = this.joint_ps(i+0,:);
            p2 = this.joint_ps(i+1,:);
            c_eff = [ linspace(p1(1),p2(1),num_samples+1); 
                      linspace(p1(2),p2(2),num_samples+1);
                      linspace(p1(3),p2(3),num_samples+1)  ];
            c_eff(:,1) = []; % origin is not a sample
            effectors = [ effectors; c_eff' ]; %#ok<AGROW>
            parents   = [ parents; i*ones(num_samples,1) ]; %#ok<AGROW>
        end
    end
    
    function h = plot(this,varargin)
        h = [];
        hold on;
        X = this.joint_ps(4:end,1);
        Y = this.joint_ps(4:end,2);
        h(end+1) = line(X,Y,varargin{:});
        h(end+1) = plot(X,Y,'.k','markersize',10);
    end
end

end