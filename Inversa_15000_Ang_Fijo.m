
%% Pruebas
clc, clear all; close all;

L1=0.265; L2=0.444; L3=0.110; L4=0.470; L5=0.080; L6=0.101;
pos_dada = [0.4 0.5 0.6]; % Pos. Final Pedida
ori_dada = [-120 35 -75]; % Ori. Final Pedida

q1 = atan2(pos_dada(2),pos_dada(1));

% T_06 = transl(pos_dada)*rpy2tr(ori_dada);
P_01 = [0 0 L1]; P_06 = pos_dada;
P_65 = [-cos(q1)*L6 -sin(q1)*L6 -L5]; %P_65 = [-L5 0 -L6]; 
P_05 = P_06+P_65; %P_05 = (T_06*[P_65 1]')';
%P_05(:,4) = [];
P_15 = P_05-P_01;
a = P_15(1); b = P_15(2); c = P_15(3); 

sigma = atan2(L4,L3);
alpha = acos((-a^2-b^2-c^2+(L2^2+L3^2+L4^2))/(2*L2*sqrt(L3^2+L4^2)));
q3 = pi-sigma-alpha;

beta = atan2(c,sqrt(a^2+b^2));
gamma = acos((L2^2-L3^2-L4^2+a^2+b^2+c^2)/(2*L2*sqrt(a^2+b^2+c^2)));
q2 = pi/2-beta-gamma;

q5 = -(q2+q3);

L(1) = Link('revolute','alpha', 0,    'a', 0,   'd',L1,   'offset', 0,   'modified', 'qlim',[-pi pi]);
L(2) = Link('revolute','alpha', -pi/2,    'a', 0,   'd',0,   'offset', -pi/2,   'modified', 'qlim',[-pi pi]);
L(3) = Link('revolute','alpha', 0,    'a', L2,   'd',0,   'offset', 0,   'modified', 'qlim',[-pi pi]);
L(4) = Link('revolute','alpha', -pi/2,    'a', L3,   'd',L4,   'offset', 0,   'modified', 'qlim',[-pi pi]);
L(5) = Link('revolute','alpha', pi/2,    'a', 0,   'd',0,   'offset', 0,   'modified', 'qlim',[-pi pi]);
L(6) = Link('revolute','alpha', -pi/2,    'a', L5,   'd',L6,   'offset', 0,   'modified', 'qlim',[-pi pi]);

plot_options = {'workspace',[-10 10 -10 10 -10 10],'scale',0.05, 'view',[130 20]};
Robot_ABB = SerialLink(L,'name','ABB CRB 15000','plotopt',plot_options);

figure(1);
hold on;
trplot(eye(4), 'width',2,'arrow');
axis([-1 1 -1 1 -1 1.5]);
Robot_ABB.teach([q1 q2 q3 0 q5 0]);
arrow3([0,0,0],[0 0 L1],'5',0);

%plot3(P_06(1),P_06(2),P_06(3));
arrow3([0 0 0],P_06,'2',0);
arrow3([0 0 0],P_05,'2',0);
arrow3([0 0 L1],[P_15(1) P_15(2) P_15(3)+L1],'2',0);

%% Robot de 6 GDL RRRRRR - ABB CRB 15000
clear all; close all; clc;
% q1=0; q2=0; q3=0; q4=0; q5=0; q6=0;

L1=0.265; L2=0.444; L3=0.110; L4=0.470; L5=0.080; L6=0.101;

L(1) = Link('revolute','alpha', 0,    'a', 0,   'd',L1,   'offset', 0,   'modified', 'qlim',[-pi pi]);
L(2) = Link('revolute','alpha', -pi/2,    'a', 0,   'd',0,   'offset', -pi/2,   'modified', 'qlim',[-pi pi]);
L(3) = Link('revolute','alpha', 0,    'a', L2,   'd',0,   'offset', 0,   'modified', 'qlim',[-pi pi]);
L(4) = Link('revolute','alpha', -pi/2,    'a', L3,   'd',L4,   'offset', 0,   'modified', 'qlim',[-pi pi]);
L(5) = Link('revolute','alpha', pi/2,    'a', 0,   'd',0,   'offset', 0,   'modified', 'qlim',[-pi pi]);
L(6) = Link('revolute','alpha', -pi/2,    'a', L5,   'd',L6,   'offset', 0,   'modified', 'qlim',[-pi pi]);

plot_options = {'workspace',[-10 10 -10 10 -10 10],'scale',0.05, 'view',[130 20]};
Robot_ABB = SerialLink(L,'name','ABB CRB 15000','plotopt',plot_options);

pos_dada = [0.6 0.6 0.8]; % Pos. Final Pedida
ori_dada = [-120 45 -90]; % Ori. Final Pedida
min_pos_q1 = 10;

for q1=-180:180
    TCP = Robot_ABB.fkine([q1*pi/180 0 0 0 0 0]);
    pos = transl(TCP)';
    c_pos = abs(sum(pos_dada-pos));
    if c_pos<min_pos_q1
        min_pos_q1 = c_pos;
        sol_q1 = q1;
    end
end
q1 = sol_q1*pi/180;

min_pos_q2_q3 = 10;
for q2=-180:180
    for q3=-225:85
    TCP = Robot_ABB.fkine([q1 q2*pi/180 q3*pi/180 0 0 0]);
    pos = transl(TCP)';
    d_pos = pos_dada-pos;
    norm_d_pos = norm(d_pos);
    if norm_d_pos<min_pos_q2_q3
        min_pos_q2_q3 = norm_d_pos;
        sol_q2 = q2;
        sol_q3 = q3;
    end
    end
end
q2 = sol_q2*pi/180;
q3 = sol_q3*pi/180;

% for q1=-180:180
%     for q2=-180:180
%         for q3=-225:85
%             for q4=-180:180
%                 for q5=-180:180
%                     for q6=-180:180
%                         
%                     end
%                 end
%             end
%         end
%     end
% end

% TCP = Robot_ABB.fkine([0 0 0 0 0 0]);
% RPY = tr2rpy(TCP)*180/pi;
% pos = transl(TCP)';


hold on;
trplot(eye(4), 'width',2,'arrow');
axis([-1 1 -1 1 -0 1.5]);
disp(q3)
Robot_ABB.teach([q1 q2 q3 0 0 0]);
arrow3([0,0,0],[0 0 L1],'5',0);