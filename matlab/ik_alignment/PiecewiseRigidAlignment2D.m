%==========================================================================
%                       Dynamic 2D/3D Registration
%          Sofien Bouaziz and Andrea Tagliasacchi and Mark Pauly
%
%                     Copyright (C) 2013  LGG, EPFL
%==========================================================================
function PiecewiseRigidAlignment2D()
clc; close all; clear;
clearvars -global *;
global S T lambda noise_sigma;
lambda = 1; %< solver regularizer
noise_sigma = 0.025; %< variance of target noise
samples_per_branch = 30; %< samples on source geometry
noise_jlength = 1;

build_gui();
init_callback();

function init_callback(h,e) %#ok<INUSD>
    rng(0); %< reset random generator

    %--- Define source geometry
    S = FKObject();
    S.joint_offs(4:end-1,1) = noise_jlength * S.joint_offs(4:end-1,1);
    S.pose( [0,0], [+.1, +.2, -.5] );
    % S.plot('color','red');

    %--- Define target geometry (with normals!!)
    target_samples_per_branch = 30;
    T = FKObject();
    T.pose( [0,0], [-.7, -.5, +.6] );
    T.points = T.sample(target_samples_per_branch); %< point cloud of target geometry
    T.points = T.points + noise_sigma*randn(size(T.points)); %< add noise?
    T.kdtree = KDTreeSearcher(T.points); %< lookup for ICP
    T.normals = compute_normals(T.points(:,1:2),10);

    %--- Define colors
    S.color=[ 0 0 .9];
    T.color=[.8 .8 0];
        
    display_source()
    clear_correspondences()
    
    %--- Target
    global h_T_lines h_T_cloud;
    if ~isempty(h_T_lines), delete(h_T_lines); end
    if ~isempty(h_T_cloud), delete(h_T_cloud); end
    % h_T_lines = T.plot('color',T.color,'linewidth',5);
    h_T_cloud = scatter2(T.points,20,T.color,'fill');
    % h_T_normals = quiver2(cloud,normals,.3);
end

function display_correspondences()
    global h_corresp;
    pS = S.sample(samples_per_branch);
    iT = T.kdtree.knnsearch(pS); %< ICP like search
    if ~isempty(h_corresp), delete(h_corresp); end;
    h_corresp = edge2(pS,T.points(iT,:),'color','red');
end

function clear_correspondences()
    global h_corresp;
    if ~isempty(h_corresp), delete(h_corresp); h_corresp = []; end;
end

function display_source()
    global h_src;
    global h_src_p;
    if ~isempty(h_src), delete(h_src); end;
    if ~isempty(h_src_p), delete(h_src_p); end;
    h_src = S.plot('color',S.color,'linewidth',5);
    h_src_p = plot(S.joint_ps(:,1),S.joint_ps(:,2),'.','markersize',50,'color',S.color);
    % h_src  = scatter(S(:,1),S(:,2),20,iS,'fill'); %< color coded samples
end

function step_callback(h,e) %#ok<INUSD>
    %--- Sample the Source
    [pS, iS] = S.sample(samples_per_branch);
    
    %--- Create effectors matrix (origin is not an effector)
    iT = T.kdtree.knnsearch(pS); %< ICP like search
    E = T.points(iT,:)-pS; %< effector vector 
    NN = T.normals(iT,:); %< normal matrix
    
    %--- Solve IK & apply
    thetas = solve(S,pS,iS,E,NN);
    S.pose( thetas(1:2), thetas(4:4+2) );
    
    %--- Visualize the new source
    display_source();
    clear_correspondences();
end

function corresp_callback(h,e) %#ok<INUSD>
    display_correspondences();
end

function run_callback(h,e) %#ok<INUSD>
    global STOP;
    STOP = 0;
    for i=1:100
        title(sprintf('iteration=%d',i),'BackgroundColor',[.7 .9 .7],'FontSize',30);
        step_callback();
        pause(.5);
        if STOP==1, break; end;
    end
end

function stop_callback(h,e) %#ok<INUSD>
    global STOP;
    STOP = 1;
end

function new_thetas = solve(S,pS,iS,E,NN)
    N = S.n_pars; %< #parameters
    K = size(E,1); %< #effectors
    LHS = zeros(N,N);
    RHS = zeros(N,1);

    %--- Build the Jacobian matrix
    for k=1:K
        e = E(k,:);
        J = zeros(N,3);
        
        %--- indexes of translation-type dependencies (DISABLED)
        % tra_deps = min(iS(k),3):-1:1; 
        % for n=tra_deps
        %     J(n,:) = f2.joint_vs(n,:);
        % end
        
        %--- indexes of rotation-type dependencies
        rot_deps = iS(k):-1:4; 
        for n=rot_deps
            J(n,:) = cross( S.joint_vs(n,:), (pS(k,:)-S.joint_ps(n,:)) );
        end
        
        %--- accumulate sides
        n = NN(k,:)';
        ntJ = n(1)*J(:,1) + n(2)*J(:,2) + n(3)*J(:,3); %<transposed err.
        nte = n(1)*e(1) + n(2)*e(2) + n(3)*e(3);
        LHS = LHS + ntJ*ntJ';
        RHS = RHS + ntJ*nte;
    end
    
    %--- Solve for IK (Tikhonov regularized)
    lambda_sq = lambda^2;
    LHS = LHS + lambda_sq*eye(N,N);
    d_theta = (LHS \ RHS)';
    
    %--- Compute the new thetas
    new_thetas = S.joint_thetas + d_theta;
end

function weight_callback(h,e) %#ok<INUSD>
    lambda = 10^get(h,'value');
    title(sprintf('lambda=%f',lambda),'BackgroundColor',[.7 .9 .7],'FontSize',30);
end

function noise_callback(h,e)
    noise_sigma = get(h,'value');
    title(sprintf('noise variance=%f',noise_sigma),'BackgroundColor',[.7 .9 .7],'FontSize',30);
    init_callback(h,e);
end

function length_callback(h,e)
    noise_jlength = get(h,'value');
    title(sprintf('length modulation=%f',noise_jlength),'BackgroundColor',[.7 .9 .7],'FontSize',30);
    init_callback(h,e);
end

function build_gui()
    figure(1); clf; 
    cla; %< sets full screen axis!!
    xlim([-.1 3]);
    ylim([-2.5  .5]);
    axis square;
    set(gcf, 'Position', [100, 100, 1024, 768]);
    
    % movegui('east')
    set(gcf,'color','white');
    axis off;
    
    %--- delete the old menu
    set(gcf,'Toolbar','none')
    set(gcf,'Menubar','none');
    
    %--- setup callbacks
    uicontrol('style', 'pushb',  'Position', [ 30 20 60 40],  'string', 'Init', 'callback', @init_callback);
    uicontrol('style', 'pushb',  'Position', [105 20 60 40],  'string', 'Corresp.', 'callback', @corresp_callback);
    uicontrol('style', 'pushb',  'Position', [165 20 60 40],  'string', 'Step', 'callback', @step_callback);
    uicontrol('style', 'pushb',  'Position', [245 20 60 40],  'string', 'Run', 'callback', @run_callback);
    uicontrol('style', 'pushb',  'Position', [305 20 60 40],  'string', 'Stop', 'callback', @stop_callback);
    uicontrol('Style',  'text',  'Position', [390 38 120 20], 'string', 'damping');
    uicontrol('Style', 'slider', 'Position', [390 -2 120 40], 'Min', -2, 'Max', 2, 'Value', log10(lambda), 'callback', @weight_callback);
    uicontrol('Style',  'text',  'Position', [520 38 120 20], 'string', 'noise');
    uicontrol('Style', 'slider', 'Position', [520 -2 120 40], 'Min', 0, 'Max', .15, 'Value', noise_sigma, 'callback', @noise_callback);
    uicontrol('Style',  'text',  'Position', [650 38 120 20], 'string', 'bone lengths');
    uicontrol('Style', 'slider', 'Position', [650 -2 120 40], 'Min', 0, 'Max', 1, 'Value', noise_jlength, 'callback', @length_callback);
end

% Compute normals for a point cloud "points" using NN nearest neighbors
function normals = compute_normals(points,NN)
    points_tree = KDTreeSearcher(points); % kd-tree data structure
    neighs = points_tree.knnsearch(points,'k',NN); % local neighbors query
    neighs = num2cell(neighs,2); % cell contains neighbors
    demeaned = cellfun(@(X)( points(X,:)-repmat(mean(points(X,:)),[numel(X),1]) ), neighs, 'UniformOutput',false);
    covs = cellfun(@(X)( X'*X ), demeaned, 'UniformOutput',false); %covariance matrixes
    [Us,~,~] = cellfun(@(X)( svd(X) ), covs, 'UniformOutput',false); % eigenvectors
    normals = cellfun(@(X)(X(:,end)'), Us, 'UniformOutput', false); % normals
    normals = cell2mat(normals); % convert cell back to array
    normals(:,3) = 0; %< only care about 2D
end

function h = edge2( p1, p2, varargin )
    % the transpose are for when A,B are matrixes
    % where every row is a point
    A = [p1(:,1), p2(:,1)]';
    B = [p1(:,2), p2(:,2)]';
    h = line( A, B, varargin{:} );
end

function h = quiver2( P, V, varargin ) %#ok<DEFNU>
    h = quiver( P(:,1), P(:,2), V(:,1), V(:,2), varargin{:} );
end

function h = scatter2( P, varargin )
    h = scatter( P(:,1), P(:,2), varargin{:} );
end

end %--- FUNCTION