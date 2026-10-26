% Chapter 3, Problem 3-3: Kirk Example 3.10-1.
% No Control System Toolbox is required. Run this script directly.
% Convention: u(k) = F_k*x(k), so the minus sign is included in F_k.
% MATLAB index k+1 represents mathematical time k.
clear; clc; close all;
format long g
outDir = fileparts(mfilename('fullpath'));
if isempty(outDir), outDir = pwd; end
if ~isfolder(outDir), mkdir(outDir); end

A = [0.9974 0.0539; -0.1078 1.1591];
B = [0.0013; 0.0539];
Q = diag([0.25, 0.05]);
R = 0.05;
H = zeros(2);
x0 = [2; 1];
Ns = [200 300 400]; % N=400 explicitly supplies data for k >= 300.
assert(rank([B A*B]) == 2, 'The system must be controllable.');

% Infinite-horizon solution from the Riccati fixed-point iteration.
Pss = H;
for iteration = 1:10000
    Fss = -((R + B'*Pss*B) \ (B'*Pss*A));
    Ac = A + B*Fss;
    Pnew = Q + Fss'*R*Fss + Ac'*Pss*Ac;
    Pnew = (Pnew + Pnew')/2;
    delta = norm(Pnew-Pss, 'fro');
    Pss = Pnew;
    if delta < 1e-13, break; end
end
assert(iteration < 10000, 'Riccati fixed-point iteration did not converge.');
Fss = -((R + B'*Pss*B) \ (B'*Pss*A));
Ac = A+B*Fss;
areResidual = norm(Pss-(Q+Fss'*R*Fss+Ac'*Pss*Ac), 'fro');
assert(areResidual < 1e-10);
assert(max(abs(eig(Ac))) < 1);
fprintf('P_inf =\n'); disp(Pss);
fprintf('F_inf =\n'); disp(Fss);
fprintf('ARE residual = %.3e\n', areResidual);

sol = cell(numel(Ns),1);
gainAll = table(); trajectoryAll = table();
for j = 1:numel(Ns)
    N = Ns(j);
    s = solveFinite(A,B,Q,R,H,x0,N);
    sol{j} = s;
    k = (0:N-1)';
    F = reshape(s.F,2,N)';
    gains = table(repmat(N,N,1), k, N-k, F(:,1), F(:,2), ...
        'VariableNames',{'N','k','remaining_stages','f1','f2'});
    gainAll = [gainAll; gains]; %#ok<AGROW>
    states = table(repmat(N,N,1), k, s.x(1,1:N)', s.x(2,1:N)', ...
        s.u, s.stageCost, s.V(1:N), s.cumulativeCost(1:N), ...
        'VariableNames',{'N','k','x1','x2','u','stage_cost', ...
        'optimal_remaining_cost','cost_before_k'});
    trajectoryAll = [trajectoryAll; states]; %#ok<AGROW>
    fprintf('N=%d: J*=%.12f; cost identity error=%.3e\n', ...
        N, s.V(1), abs(sum(s.stageCost)-s.V(1)));
end

% Equal remaining horizon must give identical gains for time-invariant data.
for j = 2:numel(Ns)
    offset = Ns(j)-Ns(1);
    difference = sol{j}.F(:,:,offset+1:end)-sol{1}.F;
    assert(max(abs(difference(:))) < 1e-12);
end

% An independent, directly condensed quadratic program for a short horizon.
Ntest = 6;
st = solveFinite(A,B,Q,R,H,x0,Ntest);
G = zeros(2*Ntest,2); D = zeros(2*Ntest,Ntest);
for k = 0:Ntest-1
    rows = 2*k+(1:2);
    G(rows,:) = A^k;
    for j = 0:k-1
        D(rows,j+1) = A^(k-1-j)*B;
    end
end
Qbar = kron(eye(Ntest),Q);
uDirect = -(D'*Qbar*D+R*eye(Ntest)) \ (D'*Qbar*G*x0);
directError = norm(uDirect-st.u,inf);
assert(directError < 1e-10, 'Direct QP and Riccati controls disagree.');
fprintf('Independent short-horizon QP error = %.3e\n',directError);

% Additional terminal-cost experiment: H=P_inf gives constant gain at all k.
terminalSol = solveFinite(A,B,Q,R,Pss,x0,Ns(1));
terminalF = reshape(terminalSol.F,2,Ns(1))';
assert(max(abs(terminalF-Fss),[],'all') < 1e-10);

firstFive = trajectoryAll(trajectoryAll.N==200 & trajectoryAll.k<5,:);
disp(firstFive);
writetable(firstFive,fullfile(outDir,'first_five_N200.csv'));
writetable(gainAll,fullfile(outDir,'feedback_gains_all.csv'));
writetable(trajectoryAll,fullfile(outDir,'optimal_trajectory_all.csv'));

% Selected tail data. The same row of F appears at equal N-k.
tail = gainAll(gainAll.remaining_stages<=100,:);
writetable(tail,fullfile(outDir,'terminal_region_gains.csv'));
rows = gainAll.N==400 & gainAll.k>=300;
writetable(gainAll(rows,:),fullfile(outDir,'gains_N400_k300_to399.csv'));
probe = [0 100 130 150 170 177 180 190 195 198 199 ...
         250 277 299 300 330 350 370 377 380 390 395 398 399];
selected = gainAll(ismember(gainAll.k,probe),:);
writetable(selected,fullfile(outDir,'selected_gain_values.csv'));

% Also record N=0,...,4 as short-horizon problems, to distinguish N from k.
shortN = (0:4)'; shortJ = zeros(5,1); sequence = strings(5,1);
for j = 1:5
    ss = solveFinite(A,B,Q,R,H,x0,shortN(j));
    shortJ(j) = ss.V(1);
    sequence(j) = string(mat2str(ss.u',10));
end
writetable(table(shortN,sequence,shortJ, ...
    'VariableNames',{'horizon_N','optimal_control_sequence','minimum_cost'}), ...
    fullfile(outDir,'alternative_short_horizons_N0_to4.csv'));

% Export figures without opening an interactive MATLAB window.
colors = lines(numel(Ns));
fig = figure('Visible','off','Color','w','Position',[100 100 1200 800]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
for component = 1:2
    nexttile; hold on;
    for j = 1:numel(Ns)
        plot(0:Ns(j)-1,reshape(sol{j}.F(1,component,:),[],1), ...
            'LineWidth',1.8,'Color',colors(j,:),'DisplayName',sprintf('N = %d',Ns(j)));
    end
    yline(Fss(component),'k--','LineWidth',1.1,'DisplayName','Infinite horizon');
    grid on; box on; xlabel('Time index k');
    ylabel(sprintf('Feedback gain f_%d',component));
    title(sprintf('Component f_%d: terminal variation follows the horizon end',component));
    legend('Location','best'); xlim([0 max(Ns)-1]);
end
sgtitle('u_k = F_k x_k, H = 0: finite-horizon gain comparison');
exportgraphics(fig,fullfile(outDir,'feedback_gains.png'),'Resolution',180);
close(fig);

fig = figure('Visible','off','Color','w','Position',[100 100 1200 800]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
styles = {'-','--',':'};
for component = 1:2
    nexttile; hold on;
    for j = 1:numel(Ns)
        r = (1:120)';
        k = Ns(j)-r;
        vals = reshape(sol{j}.F(1,component,k+1),[],1);
        plot(r,vals,'LineWidth',2.2,'LineStyle',styles{j}, ...
            'Color',colors(j,:),'DisplayName',sprintf('N = %d',Ns(j)));
    end
    yline(Fss(component),'k--','DisplayName','Infinite horizon');
    grid on; box on; xlabel('Remaining stages r = N - k');
    ylabel(sprintf('Feedback gain f_%d',component));
    title('Equal remaining stages give the same finite-horizon gain');
    xlim([1 120]); legend('Location','best');
end
sgtitle('The curves coincide when indexed by remaining horizon');
exportgraphics(fig,fullfile(outDir,'remaining_horizon.png'),'Resolution',180);
close(fig);

fig = figure('Visible','off','Color','w','Position',[100 100 1200 760]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
s = sol{1}; N=Ns(1);
nexttile;
plot(0:N,s.x','LineWidth',1.8); grid on; box on;
xlabel('Time index k'); ylabel('State'); legend('x_1','x_2','Location','best');
title('Optimal state trajectory, x_0 = [2; 1]');
nexttile;
plot(0:N-1,s.u,'LineWidth',1.8); grid on; box on;
xlabel('Time index k'); ylabel('Optimal control u_k');
title(sprintf('N = 200, minimum total cost = %.9f',s.V(1)));
exportgraphics(fig,fullfile(outDir,'optimal_trajectory_N200.png'),'Resolution',180);
close(fig);

fig = figure('Visible','off','Color','w','Position',[100 100 1200 760]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
for component = 1:2
    nexttile;
    plot(0:N-1,reshape(sol{1}.F(1,component,:),[],1),'LineWidth',1.8);
    hold on;
    plot(0:N-1,terminalF(:,component),'--','LineWidth',1.8);
    xlabel('Time index k'); ylabel(sprintf('Feedback gain f_%d',component));
    grid on; box on; legend('H = 0','H = P_{infinity}','Location','best');
end
sgtitle('An exact infinite-horizon terminal penalty removes the end effect');
exportgraphics(fig,fullfile(outDir,'terminal_penalty_comparison.png'),'Resolution',180);
close(fig);

save(fullfile(outDir,'chapter3_q3_results.mat'), ...
    'A','B','Q','R','H','x0','Ns','sol','Pss','Fss','terminalSol', ...
    'areResidual','directError');
fprintf('All assertions passed. Output directory: %s\n',outDir);

function s = solveFinite(A,B,Q,R,H,x0,N)
    n = size(A,1);
    P = zeros(n,n,N+1); F = zeros(1,n,N);
    P(:,:,N+1) = H;
    for k = N-1:-1:0
        Pnext = P(:,:,k+2);
        S = R + B'*Pnext*B;
        assert(S >= R-1e-12);
        F(:,:,k+1) = -(S \ (B'*Pnext*A));
        Ac = A+B*F(:,:,k+1);
        Pk = Q + F(:,:,k+1)'*R*F(:,:,k+1) + Ac'*Pnext*Ac;
        P(:,:,k+1) = (Pk+Pk')/2;
        assert(min(eig(P(:,:,k+1))) > -1e-10);
    end
    x = zeros(n,N+1); u = zeros(N,1); stageCost = zeros(N,1);
    x(:,1) = x0;
    for k = 0:N-1
        u(k+1) = F(:,:,k+1)*x(:,k+1);
        stageCost(k+1) = 0.5*(x(:,k+1)'*Q*x(:,k+1) + R*u(k+1)^2);
        x(:,k+2) = A*x(:,k+1)+B*u(k+1);
    end
    V = zeros(N+1,1);
    for k = 0:N
        V(k+1) = 0.5*x(:,k+1)'*P(:,:,k+1)*x(:,k+1);
    end
    terminalCost = 0.5*x(:,N+1)'*H*x(:,N+1);
    total = sum(stageCost)+terminalCost;
    assert(abs(total-V(1)) < 1e-9*max(1,abs(total)));
    if N>0
        assert(max(abs(V(1:N)-stageCost-V(2:N+1))) < 1e-9);
        if norm(H,'fro')==0, assert(abs(u(N))<1e-14); end
    end
    s = struct('N',N,'P',P,'F',F,'x',x,'u',u, ...
        'stageCost',stageCost,'V',V,'cumulativeCost',[0;cumsum(stageCost)]);
end
