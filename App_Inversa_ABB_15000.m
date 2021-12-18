classdef App_Inversa_ABB_15000 < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        ABB_CRB_15000    matlab.ui.Figure
        yaw              matlab.ui.control.NumericEditField
        YLabel           matlab.ui.control.Label
        pitch            matlab.ui.control.NumericEditField
        PLabel           matlab.ui.control.Label
        roll             matlab.ui.control.NumericEditField
        RLabel           matlab.ui.control.Label
        Label_3          matlab.ui.control.Label
        Label_2          matlab.ui.control.Label
        Label            matlab.ui.control.Label
        CalcularButton   matlab.ui.control.Button
        ParacomenzarutilizacualquiersliderparaconfigurarunanguloLabel  matlab.ui.control.Label
        mmLabel_3        matlab.ui.control.Label
        mmLabel_2        matlab.ui.control.Label
        mmLabel          matlab.ui.control.Label
        posZ             matlab.ui.control.NumericEditField
        ZEditFieldLabel  matlab.ui.control.Label
        posY             matlab.ui.control.NumericEditField
        YEditFieldLabel  matlab.ui.control.Label
        posX             matlab.ui.control.NumericEditField
        XEditFieldLabel  matlab.ui.control.Label
        PosicinYOrientacinEfectorFinalLabel  matlab.ui.control.Label
        info_q6          matlab.ui.control.NumericEditField
        q1Label_6        matlab.ui.control.Label
        info_q5          matlab.ui.control.NumericEditField
        q1Label_5        matlab.ui.control.Label
        info_q4          matlab.ui.control.NumericEditField
        q1Label_4        matlab.ui.control.Label
        info_q3          matlab.ui.control.NumericEditField
        q1Label_3        matlab.ui.control.Label
        info_q2          matlab.ui.control.NumericEditField
        q1Label_2        matlab.ui.control.Label
        info_q1          matlab.ui.control.NumericEditField
        q1Label          matlab.ui.control.Label
        q6               matlab.ui.control.Slider
        q2               matlab.ui.control.Slider
        q5               matlab.ui.control.Slider
        q4               matlab.ui.control.Slider
        q3               matlab.ui.control.Slider
        q1               matlab.ui.control.Slider
        grafica          matlab.ui.control.UIAxes
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Callback function
        function graficarButtonPushed(app, event)
         
        end

        % Value changing function: q1
        function q1ValueChanging(app, event)
            close all; clc;
            plot(app.grafica,0,0);
            axis(app.grafica,'on');
            grid(app.grafica,'on')
            hold(app.grafica,'on');
            title(app.grafica,'Robot ABB CRB-15000');

            q_1 = event.Value*pi/180;
            q_2 = app.q3.Value*pi/180;
            q_3 = app.q2.Value*pi/180;
            q_4 = app.q4.Value*pi/180;
            q_5 = app.q5.Value*pi/180;
            q_6 = app.q6.Value*pi/180;
            Q = [q_1 q_2 q_3 q_4 q_5 q_6];

           L1=0.265; L2=0.444; L3=0.110; L4=0.470; L5=0.080; L6=0.037;

           L(1) = Link('revolute','alpha',0,'a', 0,'d',L1,'offset',0,'modified','qlim',[-pi pi]);
           L(2) = Link('revolute','alpha',-pi/2,'a', 0,'d',0,'offset',-pi/2,'modified','qlim',[-pi pi]);
           L(3) = Link('revolute','alpha',0,'a', L2,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(4) = Link('revolute','alpha',-pi/2,'a', L3,'d',L4,'offset',0,'modified', 'qlim',[-pi pi]);
           L(5) = Link('revolute','alpha',pi/2,'a', 0,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(6) = Link('revolute','alpha',-pi/2,'a', L5,'d',L6,'offset', 0,'modified','qlim',[-pi pi]);

           pos_x = [0];
           pos_y = [0];
           pos_z = [0];

           MTH_BC = L(1).A(Q(1));
           [R,translation] = tr2rt(MTH_BC);
           pos_x(1) = [translation(1)];
           pos_y(1) = [translation(2)];
           pos_z(1) = [translation(3)];

           for i=2:6
               MTH_BC = MTH_BC*L(i).A(Q(i));
               [R,translation] = tr2rt(MTH_BC);
               pos_x(i+1) = [translation(1)];
               pos_y(i+1) = [translation(2)];
               pos_z(i+1) = [translation(3)];
               plot3(app.grafica,pos_x(i+1),pos_y(i+1),pos_z(i+1),'-o' , 'Color' , 'b' , 'MarkerSize' , 15,'MarkerFaceColor','b')
           end

            plot3(app.grafica,pos_x,pos_y,pos_z,'LineWidth',5,'Color',[.6 0 0]);

           tool = MTH_BC*[0.15 0 0 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','r');

           tool = MTH_BC*[0 0 0.2 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','c');

           view(app.grafica,140,30);
           axis(app.grafica,[-1 1 -1 1 -0.67 1.3]);
           hold(app.grafica,'off');

           app.info_q1.Value = round(q_1*180/pi,2);
           app.info_q2.Value = round(q_2*180/pi,2);
           app.info_q3.Value = round(q_3*180/pi,2);
           app.info_q4.Value = round(q_4*180/pi,2);
           app.info_q5.Value = round(q_5*180/pi,2);
           app.info_q6.Value = round(q_6*180/pi,2);
           app.posX.Value = round(translation(1)*1000,2);
           app.posY.Value = round(translation(2)*1000,2);
           app.posZ.Value = round(translation(3)*1000,2);

           T_01 = L(1).A(q_1);
           T_12 = L(2).A(q_2);
           T_23 = L(3).A(q_3);
           T_34 = L(4).A(q_4);
           T_45 = L(5).A(q_5);
           T_56 = L(6).A(q_6);
           T_06 = T_01*T_12*T_23*T_34*T_45*T_56;
           RPY = tr2rpy(T_06);           
           app.roll.Value = RPY(1)*180/pi;
           app.pitch.Value = RPY(2)*180/pi;
           app.yaw.Value = RPY(3)*180/pi;
         
        end

        % Value changing function: q3
        function q3ValueChanging(app, event)
            close all; clc;
            plot(app.grafica,0,0);
            axis(app.grafica,'on');
            grid(app.grafica,'on')
            hold(app.grafica,'on');
            title(app.grafica,'Robot ABB CRB-15000');

            q_1 = app.q1.Value*pi/180;
            q_2 = app.q2.Value*pi/180;
            q_3 = event.Value*pi/180;
            q_4 = app.q4.Value*pi/180;
            q_5 = app.q5.Value*pi/180;
            q_6 = app.q6.Value*pi/180;
            Q = [q_1 q_2 q_3 q_4 q_5 q_6];

           L1=0.265; L2=0.444; L3=0.110; L4=0.470; L5=0.080; L6=0.037;

           L(1) = Link('revolute','alpha',0,'a', 0,'d',L1,'offset',0,'modified','qlim',[-pi pi]);
           L(2) = Link('revolute','alpha',-pi/2,'a', 0,'d',0,'offset',-pi/2,'modified','qlim',[-pi pi]);
           L(3) = Link('revolute','alpha',0,'a', L2,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(4) = Link('revolute','alpha',-pi/2,'a', L3,'d',L4,'offset',0,'modified', 'qlim',[-pi pi]);
           L(5) = Link('revolute','alpha',pi/2,'a', 0,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(6) = Link('revolute','alpha',-pi/2,'a', L5,'d',L6,'offset', 0,'modified','qlim',[-pi pi]);

           pos_x = [0];
           pos_y = [0];
           pos_z = [0];

           MTH_BC = L(1).A(Q(1));
           [R,translation] = tr2rt(MTH_BC);
           pos_x(1) = [translation(1)];
           pos_y(1) = [translation(2)];
           pos_z(1) = [translation(3)];

           for i=2:6
               MTH_BC = MTH_BC*L(i).A(Q(i));
               [R,translation] = tr2rt(MTH_BC);
               pos_x(i+1) = [translation(1)];
               pos_y(i+1) = [translation(2)];
               pos_z(i+1) = [translation(3)];
               plot3(app.grafica,pos_x(i+1),pos_y(i+1),pos_z(i+1),'-o' , 'Color' , 'b' , 'MarkerSize' , 15,'MarkerFaceColor','b')
           end

           plot3(app.grafica,pos_x,pos_y,pos_z,'LineWidth',5,'Color',[.6 0 0]);

           tool = MTH_BC*[0.15 0 0 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','r');

           tool = MTH_BC*[0 0 0.2 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','c');

           view(app.grafica,140,30);
           axis(app.grafica,[-1 1 -1 1 -0.67 1.3]);
           hold(app.grafica,'off');

           app.info_q1.Value = round(q_1*180/pi,2);
           app.info_q2.Value = round(q_2*180/pi,2);
           app.info_q3.Value = round(q_3*180/pi,2);
           app.info_q4.Value = round(q_4*180/pi,2);
           app.info_q5.Value = round(q_5*180/pi,2);
           app.info_q6.Value = round(q_6*180/pi,2);
           app.posX.Value = round(translation(1)*1000,2);
           app.posY.Value = round(translation(2)*1000,2);
           app.posZ.Value = round(translation(3)*1000,2);

           T_01 = L(1).A(q_1);
           T_12 = L(2).A(q_2);
           T_23 = L(3).A(q_3);
           T_34 = L(4).A(q_4);
           T_45 = L(5).A(q_5);
           T_56 = L(6).A(q_6);
           T_06 = T_01*T_12*T_23*T_34*T_45*T_56;
           RPY = tr2rpy(T_06);           
           app.roll.Value = RPY(1)*180/pi;
           app.pitch.Value = RPY(2)*180/pi;
           app.yaw.Value = RPY(3)*180/pi;
            
        end

        % Value changed function: q3
        function q3ValueChanged(app, event)
            value = app.q3.Value;
            
        end

        % Value changed function: q1
        function q1ValueChanged(app, event)
            value = app.q1.Value;
            
        end

        % Value changed function: info_q1
        function info_q1ValueChanged(app, event)
            value = app.info_q1.Value;
            
        end

        % Value changed function: info_q2
        function info_q2ValueChanged(app, event)
            value = app.info_q2.Value;
            
        end

        % Value changed function: info_q3
        function info_q3ValueChanged(app, event)
            value = app.info_q3.Value;
            
        end

        % Value changed function: info_q4
        function info_q4ValueChanged(app, event)
            value = app.info_q4.Value;
            
        end

        % Value changed function: info_q5
        function info_q5ValueChanged(app, event)
            value = app.info_q5.Value;
            
        end

        % Value changed function: info_q6
        function info_q6ValueChanged(app, event)
            value = app.info_q6.Value;
            
        end

        % Value changed function: q4
        function q4ValueChanged(app, event)
            value = app.q4.Value;
            
        end

        % Value changed function: q5
        function q5ValueChanged(app, event)
            value = app.q5.Value;
            
        end

        % Value changed function: q6
        function q6ValueChanged(app, event)
            value = app.q6.Value;
            
        end

        % Value changed function: posX
        function posXValueChanged(app, event)
            value = app.posX.Value;
            
        end

        % Value changed function: posY
        function posYValueChanged(app, event)
            value = app.posY.Value;
            
        end

        % Value changed function: posZ
        function posZValueChanged(app, event)
            value = app.posZ.Value;
            
        end

        % Value changing function: q2
        function q2ValueChanging(app, event)
            close all; clc;
            plot(app.grafica,0,0);
            axis(app.grafica,'on');
            grid(app.grafica,'on')
            hold(app.grafica,'on');
            title(app.grafica,'Robot ABB CRB-15000');

            q_1 = app.q1.Value*pi/180;
            q_2 = event.Value*pi/180;
            q_3 = app.q3.Value*pi/180;
            q_4 = app.q4.Value*pi/180;
            q_5 = app.q5.Value*pi/180;
            q_6 = app.q6.Value*pi/180;
            Q = [q_1 q_2 q_3 q_4 q_5 q_6];

           L1=0.265; L2=0.444; L3=0.110; L4=0.470; L5=0.080; L6=0.037;

           L(1) = Link('revolute','alpha',0,'a', 0,'d',L1,'offset',0,'modified','qlim',[-pi pi]);
           L(2) = Link('revolute','alpha',-pi/2,'a', 0,'d',0,'offset',-pi/2,'modified','qlim',[-pi pi]);
           L(3) = Link('revolute','alpha',0,'a', L2,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(4) = Link('revolute','alpha',-pi/2,'a', L3,'d',L4,'offset',0,'modified', 'qlim',[-pi pi]);
           L(5) = Link('revolute','alpha',pi/2,'a', 0,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(6) = Link('revolute','alpha',-pi/2,'a', L5,'d',L6,'offset', 0,'modified','qlim',[-pi pi]);

           pos_x = [0];
           pos_y = [0];
           pos_z = [0];

           MTH_BC = L(1).A(Q(1));
           [R,translation] = tr2rt(MTH_BC);
           pos_x(1) = [translation(1)];
           pos_y(1) = [translation(2)];
           pos_z(1) = [translation(3)];

           for i=2:6
               MTH_BC = MTH_BC*L(i).A(Q(i));
               [R,translation] = tr2rt(MTH_BC);
               pos_x(i+1) = [translation(1)];
               pos_y(i+1) = [translation(2)];
               pos_z(i+1) = [translation(3)];
               plot3(app.grafica,pos_x(i+1),pos_y(i+1),pos_z(i+1),'-o' , 'Color' , 'b' , 'MarkerSize' , 15,'MarkerFaceColor','b')
           end

           plot3(app.grafica,pos_x,pos_y,pos_z,'LineWidth',5,'Color',[.6 0 0]);

           tool = MTH_BC*[0.15 0 0 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','r');

           tool = MTH_BC*[0 0 0.2 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','c');

           view(app.grafica,140,30);
           axis(app.grafica,[-1 1 -1 1 -0.67 1.3]);
           hold(app.grafica,'off');

           app.info_q1.Value = round(q_1*180/pi,2);
           app.info_q2.Value = round(q_2*180/pi,2);
           app.info_q3.Value = round(q_3*180/pi,2);
           app.info_q4.Value = round(q_4*180/pi,2);
           app.info_q5.Value = round(q_5*180/pi,2);
           app.info_q6.Value = round(q_6*180/pi,2);
           app.posX.Value = round(translation(1)*1000,2);
           app.posY.Value = round(translation(2)*1000,2);
           app.posZ.Value = round(translation(3)*1000,2);

           T_01 = L(1).A(q_1);
           T_12 = L(2).A(q_2);
           T_23 = L(3).A(q_3);
           T_34 = L(4).A(q_4);
           T_45 = L(5).A(q_5);
           T_56 = L(6).A(q_6);
           T_06 = T_01*T_12*T_23*T_34*T_45*T_56;
           RPY = tr2rpy(T_06);           
           app.roll.Value = RPY(1)*180/pi;
           app.pitch.Value = RPY(2)*180/pi;
           app.yaw.Value = RPY(3)*180/pi;
            
        end

        % Value changing function: q4
        function q4ValueChanging(app, event)
            close all; clc;
            plot(app.grafica,0,0);
            axis(app.grafica,'on');
            grid(app.grafica,'on')
            hold(app.grafica,'on');
            title(app.grafica,'Robot ABB CRB-15000');

            q_1 = app.q1.Value*pi/180;
            q_2 = app.q2.Value*pi/180;
            q_3 = app.q3.Value*pi/180;
            q_4 = event.Value*pi/180;
            q_5 = app.q5.Value*pi/180;
            q_6 = app.q6.Value*pi/180;
            Q = [q_1 q_2 q_3 q_4 q_5 q_6];

           L1=0.265; L2=0.444; L3=0.110; L4=0.470; L5=0.080; L6=0.037;

           L(1) = Link('revolute','alpha',0,'a', 0,'d',L1,'offset',0,'modified','qlim',[-pi pi]);
           L(2) = Link('revolute','alpha',-pi/2,'a', 0,'d',0,'offset',-pi/2,'modified','qlim',[-pi pi]);
           L(3) = Link('revolute','alpha',0,'a', L2,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(4) = Link('revolute','alpha',-pi/2,'a', L3,'d',L4,'offset',0,'modified', 'qlim',[-pi pi]);
           L(5) = Link('revolute','alpha',pi/2,'a', 0,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(6) = Link('revolute','alpha',-pi/2,'a', L5,'d',L6,'offset', 0,'modified','qlim',[-pi pi]);

           pos_x = [0];
           pos_y = [0];
           pos_z = [0];

           MTH_BC = L(1).A(Q(1));
           [R,translation] = tr2rt(MTH_BC);
           pos_x(1) = [translation(1)];
           pos_y(1) = [translation(2)];
           pos_z(1) = [translation(3)];

           for i=2:6
               MTH_BC = MTH_BC*L(i).A(Q(i));
               [R,translation] = tr2rt(MTH_BC);
               pos_x(i+1) = [translation(1)];
               pos_y(i+1) = [translation(2)];
               pos_z(i+1) = [translation(3)];
               plot3(app.grafica,pos_x(i+1),pos_y(i+1),pos_z(i+1),'-o' , 'Color' , 'b' , 'MarkerSize' , 15,'MarkerFaceColor','b')
           end

           plot3(app.grafica,pos_x,pos_y,pos_z,'LineWidth',5,'Color',[.6 0 0]);
    
           tool = MTH_BC*[0.15 0 0 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','r');

           tool = MTH_BC*[0 0 0.2 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','c');

           view(app.grafica,140,30);
           axis(app.grafica,[-1 1 -1 1 -0.67 1.3]);
           hold(app.grafica,'off');

           app.info_q1.Value = round(q_1*180/pi,2);
           app.info_q2.Value = round(q_2*180/pi,2);
           app.info_q3.Value = round(q_3*180/pi,2);
           app.info_q4.Value = round(q_4*180/pi,2);
           app.info_q5.Value = round(q_5*180/pi,2);
           app.info_q6.Value = round(q_6*180/pi,2);

           app.posX.Value = round(translation(1)*1000,2);
           app.posY.Value = round(translation(2)*1000,2);
           app.posZ.Value = round(translation(3)*1000,2);

           T_01 = L(1).A(q_1);
           T_12 = L(2).A(q_2);
           T_23 = L(3).A(q_3);
           T_34 = L(4).A(q_4);
           T_45 = L(5).A(q_5);
           T_56 = L(6).A(q_6);
           T_06 = T_01*T_12*T_23*T_34*T_45*T_56;
           RPY = tr2rpy(T_06);           
           app.roll.Value = RPY(1)*180/pi;
           app.pitch.Value = RPY(2)*180/pi;
           app.yaw.Value = RPY(3)*180/pi;
            
        end

        % Value changing function: q5
        function q5ValueChanging(app, event)
            close all; clc;
            plot(app.grafica,0,0);
            axis(app.grafica,'on');
            grid(app.grafica,'on')
            hold(app.grafica,'on');
            title(app.grafica,'Robot ABB CRB-15000');

            q_1 = app.q1.Value*pi/180;
            q_2 = app.q2.Value*pi/180;
            q_3 = app.q3.Value*pi/180;
            q_4 = app.q4.Value*pi/180;
            q_5 = event.Value*pi/180;
            q_6 = app.q6.Value*pi/180;
            Q = [q_1 q_2 q_3 q_4 q_5 q_6];

           L1=0.265; L2=0.444; L3=0.110; L4=0.470; L5=0.080; L6=0.037;

           L(1) = Link('revolute','alpha',0,'a', 0,'d',L1,'offset',0,'modified','qlim',[-pi pi]);
           L(2) = Link('revolute','alpha',-pi/2,'a', 0,'d',0,'offset',-pi/2,'modified','qlim',[-pi pi]);
           L(3) = Link('revolute','alpha',0,'a', L2,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(4) = Link('revolute','alpha',-pi/2,'a', L3,'d',L4,'offset',0,'modified', 'qlim',[-pi pi]);
           L(5) = Link('revolute','alpha',pi/2,'a', 0,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(6) = Link('revolute','alpha',-pi/2,'a', L5,'d',L6,'offset', 0,'modified','qlim',[-pi pi]);

           pos_x = [0];
           pos_y = [0];
           pos_z = [0];

           MTH_BC = L(1).A(Q(1));
           [R,translation] = tr2rt(MTH_BC);
           pos_x(1) = [translation(1)];
           pos_y(1) = [translation(2)];
           pos_z(1) = [translation(3)];

           for i=2:6
               MTH_BC = MTH_BC*L(i).A(Q(i));
               [R,translation] = tr2rt(MTH_BC);
               pos_x(i+1) = [translation(1)];
               pos_y(i+1) = [translation(2)];
               pos_z(i+1) = [translation(3)];
               plot3(app.grafica,pos_x(i+1),pos_y(i+1),pos_z(i+1),'-o' , 'Color' , 'b' , 'MarkerSize' , 15,'MarkerFaceColor','b')
           end

           plot3(app.grafica,pos_x,pos_y,pos_z,'LineWidth',5,'Color',[.6 0 0]);

           tool = MTH_BC*[0.15 0 0 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','r');

           tool = MTH_BC*[0 0 0.2 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','c');

           view(app.grafica,140,30);
           axis(app.grafica,[-1 1 -1 1 -0.67 1.3]);
           hold(app.grafica,'off');


           app.info_q1.Value = round(q_1*180/pi,2);
           app.info_q2.Value = round(q_2*180/pi,2);
           app.info_q3.Value = round(q_3*180/pi,2);
           app.info_q4.Value = round(q_4*180/pi,2);
           app.info_q5.Value = round(q_5*180/pi,2);
           app.info_q6.Value = round(q_6*180/pi,2);
           app.posX.Value = round(translation(1)*1000,2);
           app.posY.Value = round(translation(2)*1000,2);
           app.posZ.Value = round(translation(3)*1000,2);

           T_01 = L(1).A(q_1);
           T_12 = L(2).A(q_2);
           T_23 = L(3).A(q_3);
           T_34 = L(4).A(q_4);
           T_45 = L(5).A(q_5);
           T_56 = L(6).A(q_6);
           T_06 = T_01*T_12*T_23*T_34*T_45*T_56;
           RPY = tr2rpy(T_06);           
           app.roll.Value = RPY(1)*180/pi;
           app.pitch.Value = RPY(2)*180/pi;
           app.yaw.Value = RPY(3)*180/pi;
        end

        % Value changing function: q6
        function q6ValueChanging(app, event)
            close all; clc;
            plot(app.grafica,0,0);
            axis(app.grafica,'on');
            grid(app.grafica,'on')
            hold(app.grafica,'on');
            title(app.grafica,'Robot ABB CRB-15000');

            q_1 = app.q1.Value*pi/180;
            q_2 = app.q2.Value*pi/180;
            q_3 = app.q3.Value*pi/180;
            q_4 = app.q4.Value*pi/180;
            q_5 = app.q5.Value*pi/180;
            q_6 = event.Value*pi/180;
            Q = [q_1 q_2 q_3 q_4 q_5 q_6];

           L1=0.265; L2=0.444; L3=0.110; L4=0.470; L5=0.080; L6=0.037;

           L(1) = Link('revolute','alpha',0,'a', 0,'d',L1,'offset',0,'modified','qlim',[-pi pi]);
           L(2) = Link('revolute','alpha',-pi/2,'a', 0,'d',0,'offset',-pi/2,'modified','qlim',[-pi pi]);
           L(3) = Link('revolute','alpha',0,'a', L2,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(4) = Link('revolute','alpha',-pi/2,'a', L3,'d',L4,'offset',0,'modified', 'qlim',[-pi pi]);
           L(5) = Link('revolute','alpha',pi/2,'a', 0,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(6) = Link('revolute','alpha',-pi/2,'a', L5,'d',L6,'offset', 0,'modified','qlim',[-pi pi]);

           pos_x = [0];
           pos_y = [0];
           pos_z = [0];

           MTH_BC = L(1).A(Q(1));
           [R,translation] = tr2rt(MTH_BC);
           pos_x(1) = [translation(1)];
           pos_y(1) = [translation(2)];
           pos_z(1) = [translation(3)];

           for i=2:6
               MTH_BC = MTH_BC*L(i).A(Q(i));
               [R,translation] = tr2rt(MTH_BC);
               pos_x(i+1) = [translation(1)];
               pos_y(i+1) = [translation(2)];
               pos_z(i+1) = [translation(3)];
               plot3(app.grafica,pos_x(i+1),pos_y(i+1),pos_z(i+1),'-o' , 'Color' , 'b' , 'MarkerSize' , 15,'MarkerFaceColor','b')
           end

           plot3(app.grafica,pos_x,pos_y,pos_z,'LineWidth',5,'Color',[.6 0 0]);

           tool = MTH_BC*[0.15 0 0 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','r');

           tool = MTH_BC*[0 0 0.2 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','c');

           view(app.grafica,140,30);
           axis(app.grafica,[-1 1 -1 1 -0.67 1.3]);
           hold(app.grafica,'off');

           app.info_q1.Value = round(q_1*180/pi,2);
           app.info_q2.Value = round(q_2*180/pi,2);
           app.info_q3.Value = round(q_3*180/pi,2);
           app.info_q4.Value = round(q_4*180/pi,2);
           app.info_q5.Value = round(q_5*180/pi,2);
           app.info_q6.Value = round(q_6*180/pi,2);
           app.posX.Value = round(translation(1)*1000,2);
           app.posY.Value = round(translation(2)*1000,2);
           app.posZ.Value = round(translation(3)*1000,2);

           T_01 = L(1).A(q_1);
           T_12 = L(2).A(q_2);
           T_23 = L(3).A(q_3);
           T_34 = L(4).A(q_4);
           T_45 = L(5).A(q_5);
           T_56 = L(6).A(q_6);
           T_06 = T_01*T_12*T_23*T_34*T_45*T_56;
           RPY = tr2rpy(T_06);           
           app.roll.Value = RPY(1)*180/pi;
           app.pitch.Value = RPY(2)*180/pi;
           app.yaw.Value = RPY(3)*180/pi;
        end

        % Button pushed function: CalcularButton
        function CalcularButtonPushed(app, event)
            x = app.posX.Value/1000;
            y = app.posY.Value/1000;
            z = app.posZ.Value/1000;
            R = app.roll.Value;
            P = app.pitch.Value;
            Y = app.yaw.Value;

L1=0.265; L2=0.444; L3=0.110; L4=0.470; L5=0.080; L6=0.101;
pos_dada = [x y z]; % Pos. Final Pedida
ori_dada = [R P Y]*pi/180; % Ori. Final Pedida

Q1 = atan2(pos_dada(2),pos_dada(1));

T_06 = transl(pos_dada)*rpy2tr(ori_dada);
m = (T_06*[0 0 1 1]')';
m(:,4) = [];
P = m-pos_dada;
phi_1 = atan2(P(3),sqrt(P(1)^2+P(2)^2));
f = atan2(L5,L6);
h= phi_1+f;
P_65 = [cos(Q1)*(cos(h)*sqrt(L5^2+L6^2)) sin(Q1)*(cos(h)*sqrt(L5^2+L6^2)) sin(h)*sqrt(L5^2+L6^2)]; 
P_05 = pos_dada-P_65;
P_01 = [0 0 L1]; P_06 = pos_dada;
P_15 = P_05-P_01;
a = P_15(1); b = P_15(2); c = P_15(3); 

sigma = atan2(L4,L3);
alpha = acos((-a^2-b^2-c^2+(L2^2+L3^2+L4^2))/(2*L2*sqrt(L3^2+L4^2)));
Q3 = pi-sigma-alpha;

beta = atan2(c,sqrt(a^2+b^2));
gamma = acos((L2^2-L3^2-L4^2+a^2+b^2+c^2)/(2*L2*sqrt(a^2+b^2+c^2)));
Q2 = pi/2-beta-gamma;

P_56_a = [cos(Q1)*L6 sin(Q1)*L6 L5];
ang_0 = atan2(P_65(3),sqrt(P_65(1)^2+P_65(2)^2));
ang_1 = atan2(P_56_a(3),sqrt(P_56_a(1)^2+P_56_a(2)^2));

Q5 = -Q2-Q3+ang_1-ang_0;

L(1) = Link('revolute','alpha', 0,    'a', 0,   'd',L1,   'offset', 0,   'modified', 'qlim',[-pi pi]);
L(2) = Link('revolute','alpha', -pi/2,    'a', 0,   'd',0,   'offset', -pi/2,   'modified', 'qlim',[-pi pi]);
L(3) = Link('revolute','alpha', 0,    'a', L2,   'd',0,   'offset', 0,   'modified', 'qlim',[-pi pi]);
L(4) = Link('revolute','alpha', -pi/2,    'a', L3,   'd',L4,   'offset', 0,   'modified', 'qlim',[-pi pi]);
L(5) = Link('revolute','alpha', pi/2,    'a', 0,   'd',0,   'offset', 0,   'modified', 'qlim',[-pi pi]);
L(6) = Link('revolute','alpha', -pi/2,    'a', L5,   'd',L6,   'offset', 0,   'modified', 'qlim',[-pi pi]);

T_01 = L(1).A(Q1);
T_12 = L(2).A(Q2);
T_23 = L(3).A(Q3);
T_34 = L(4).A(0);
T_45 = L(5).A(Q5);
Q6 = 0;
T_56 = L(6).A(Q6);
T_06 = T_01*T_12*T_23*T_34*T_45*T_56;
RPY = tr2rpy(T_06);
Q6 = -RPY(3)+ori_dada(3);

             app.info_q1.Value = Q1*180/pi;
             app.info_q2.Value = Q2*180/pi;
             app.info_q3.Value = Q3*180/pi;
             app.info_q4.Value = 0;
             app.info_q5.Value = Q5*180/pi;
             app.info_q6.Value = Q6*180/pi;
             
             app.q1.Value = Q1*180/pi;
             app.q2.Value = Q2*180/pi;
             app.q3.Value = Q3*180/pi;
             app.q4.Value = 0;
             app.q5.Value = Q5*180/pi;
             app.q6.Value = Q6*180/pi;

             %%% GRÁFICA
                         close all; clc;
            plot(app.grafica,0,0);
            axis(app.grafica,'on');
            grid(app.grafica,'on')
            hold(app.grafica,'on');
            title(app.grafica,'Robot ABB CRB-15000');

            q_1 = app.info_q1.Value*pi/180;
            q_2 = app.info_q2.Value*pi/180;
            q_3 = app.info_q3.Value*pi/180;
            q_4 = app.info_q4.Value*pi/180;
            q_5 = app.info_q5.Value*pi/180;
            q_6 = app.info_q6.Value*pi/180;
            Q = [q_1 q_2 q_3 q_4 q_5 q_6];
            disp(Q)

           L1=0.265; L2=0.444; L3=0.110; L4=0.470; L5=0.080; L6=0.037;

           L(1) = Link('revolute','alpha',0,'a', 0,'d',L1,'offset',0,'modified','qlim',[-pi pi]);
           L(2) = Link('revolute','alpha',-pi/2,'a', 0,'d',0,'offset',-pi/2,'modified','qlim',[-pi pi]);
           L(3) = Link('revolute','alpha',0,'a', L2,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(4) = Link('revolute','alpha',-pi/2,'a', L3,'d',L4,'offset',0,'modified', 'qlim',[-pi pi]);
           L(5) = Link('revolute','alpha',pi/2,'a', 0,'d',0,'offset',0,'modified','qlim',[-pi pi]);
           L(6) = Link('revolute','alpha',-pi/2,'a', L5,'d',L6,'offset', 0,'modified','qlim',[-pi pi]);

           pos_x = [0];
           pos_y = [0];
           pos_z = [0];

           MTH_BC = L(1).A(Q(1));
           [R,translation] = tr2rt(MTH_BC);
           pos_x(1) = [translation(1)];
           pos_y(1) = [translation(2)];
           pos_z(1) = [translation(3)];

           for i=2:6
               MTH_BC = MTH_BC*L(i).A(Q(i));
               [R,translation] = tr2rt(MTH_BC);
               pos_x(i+1) = [translation(1)];
               pos_y(i+1) = [translation(2)];
               pos_z(i+1) = [translation(3)];
               plot3(app.grafica,pos_x(i+1),pos_y(i+1),pos_z(i+1),'-o' , 'Color' , 'b' , 'MarkerSize' , 15,'MarkerFaceColor','b')
           end

           plot3(app.grafica,pos_x,pos_y,pos_z,'LineWidth',5,'Color',[.6 0 0]);

           tool = MTH_BC*[0.15 0 0 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','r');

           tool = MTH_BC*[0 0 0.2 1]';
           pos_tool_x = [pos_x(7) tool(1)];
           pos_tool_y = [pos_y(7) tool(2)];
           pos_tool_z = [pos_z(7) tool(3)];
           plot3(app.grafica,pos_tool_x,pos_tool_y,pos_tool_z,'LineWidth',2,'Color','c');

           view(app.grafica,140,30);
           axis(app.grafica,[-1 1 -1 1 -0.67 1.3]);
           hold(app.grafica,'off');


        end

        % Value changed function: roll
        function rollValueChanged(app, event)
            value = app.roll.Value;
            
        end

        % Value changed function: pitch
        function pitchValueChanged(app, event)
            value = app.pitch.Value;
            
        end

        % Value changed function: yaw
        function yawValueChanged(app, event)
            value = app.yaw.Value;
            
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create ABB_CRB_15000 and hide until all components are created
            app.ABB_CRB_15000 = uifigure('Visible', 'off');
            app.ABB_CRB_15000.Position = [100 100 961 596];
            app.ABB_CRB_15000.Name = 'MATLAB App';

            % Create grafica
            app.grafica = uiaxes(app.ABB_CRB_15000);
            title(app.grafica, 'Title')
            xlabel(app.grafica, 'X')
            ylabel(app.grafica, 'Y')
            zlabel(app.grafica, 'Z')
            app.grafica.Position = [366 49 564 515];

            % Create q1
            app.q1 = uislider(app.ABB_CRB_15000);
            app.q1.Limits = [-180 180];
            app.q1.ValueChangedFcn = createCallbackFcn(app, @q1ValueChanged, true);
            app.q1.ValueChangingFcn = createCallbackFcn(app, @q1ValueChanging, true);
            app.q1.Position = [156 395 150 3];

            % Create q3
            app.q3 = uislider(app.ABB_CRB_15000);
            app.q3.Limits = [-225 85];
            app.q3.ValueChangedFcn = createCallbackFcn(app, @q3ValueChanged, true);
            app.q3.ValueChangingFcn = createCallbackFcn(app, @q3ValueChanging, true);
            app.q3.Position = [157 295 150 3];

            % Create q4
            app.q4 = uislider(app.ABB_CRB_15000);
            app.q4.Limits = [-180 180];
            app.q4.ValueChangedFcn = createCallbackFcn(app, @q4ValueChanged, true);
            app.q4.ValueChangingFcn = createCallbackFcn(app, @q4ValueChanging, true);
            app.q4.Position = [157 246 150 3];

            % Create q5
            app.q5 = uislider(app.ABB_CRB_15000);
            app.q5.Limits = [-180 180];
            app.q5.ValueChangedFcn = createCallbackFcn(app, @q5ValueChanged, true);
            app.q5.ValueChangingFcn = createCallbackFcn(app, @q5ValueChanging, true);
            app.q5.Position = [157 196 150 3];

            % Create q2
            app.q2 = uislider(app.ABB_CRB_15000);
            app.q2.Limits = [-180 180];
            app.q2.ValueChangingFcn = createCallbackFcn(app, @q2ValueChanging, true);
            app.q2.Position = [156 345 150 3];

            % Create q6
            app.q6 = uislider(app.ABB_CRB_15000);
            app.q6.Limits = [-180 180];
            app.q6.ValueChangedFcn = createCallbackFcn(app, @q6ValueChanged, true);
            app.q6.ValueChangingFcn = createCallbackFcn(app, @q6ValueChanging, true);
            app.q6.Position = [157 149 150 3];

            % Create q1Label
            app.q1Label = uilabel(app.ABB_CRB_15000);
            app.q1Label.HorizontalAlignment = 'right';
            app.q1Label.Position = [45 386 25 22];
            app.q1Label.Text = 'q1';

            % Create info_q1
            app.info_q1 = uieditfield(app.ABB_CRB_15000, 'numeric');
            app.info_q1.ValueChangedFcn = createCallbackFcn(app, @info_q1ValueChanged, true);
            app.info_q1.Position = [80 385 43 22];

            % Create q1Label_2
            app.q1Label_2 = uilabel(app.ABB_CRB_15000);
            app.q1Label_2.HorizontalAlignment = 'right';
            app.q1Label_2.Position = [45 336 25 22];
            app.q1Label_2.Text = 'q2';

            % Create info_q2
            app.info_q2 = uieditfield(app.ABB_CRB_15000, 'numeric');
            app.info_q2.ValueChangedFcn = createCallbackFcn(app, @info_q2ValueChanged, true);
            app.info_q2.Position = [80 335 43 22];

            % Create q1Label_3
            app.q1Label_3 = uilabel(app.ABB_CRB_15000);
            app.q1Label_3.HorizontalAlignment = 'right';
            app.q1Label_3.Position = [45 286 25 22];
            app.q1Label_3.Text = 'q3';

            % Create info_q3
            app.info_q3 = uieditfield(app.ABB_CRB_15000, 'numeric');
            app.info_q3.ValueChangedFcn = createCallbackFcn(app, @info_q3ValueChanged, true);
            app.info_q3.Position = [80 285 43 22];

            % Create q1Label_4
            app.q1Label_4 = uilabel(app.ABB_CRB_15000);
            app.q1Label_4.HorizontalAlignment = 'right';
            app.q1Label_4.Position = [45 237 25 22];
            app.q1Label_4.Text = 'q4';

            % Create info_q4
            app.info_q4 = uieditfield(app.ABB_CRB_15000, 'numeric');
            app.info_q4.ValueChangedFcn = createCallbackFcn(app, @info_q4ValueChanged, true);
            app.info_q4.Position = [80 236 43 22];

            % Create q1Label_5
            app.q1Label_5 = uilabel(app.ABB_CRB_15000);
            app.q1Label_5.HorizontalAlignment = 'right';
            app.q1Label_5.Position = [45 187 25 22];
            app.q1Label_5.Text = 'q5';

            % Create info_q5
            app.info_q5 = uieditfield(app.ABB_CRB_15000, 'numeric');
            app.info_q5.ValueChangedFcn = createCallbackFcn(app, @info_q5ValueChanged, true);
            app.info_q5.Position = [80 186 43 22];

            % Create q1Label_6
            app.q1Label_6 = uilabel(app.ABB_CRB_15000);
            app.q1Label_6.HorizontalAlignment = 'right';
            app.q1Label_6.Position = [45 139 25 22];
            app.q1Label_6.Text = 'q6';

            % Create info_q6
            app.info_q6 = uieditfield(app.ABB_CRB_15000, 'numeric');
            app.info_q6.ValueChangedFcn = createCallbackFcn(app, @info_q6ValueChanged, true);
            app.info_q6.Position = [80 138 43 22];

            % Create PosicinYOrientacinEfectorFinalLabel
            app.PosicinYOrientacinEfectorFinalLabel = uilabel(app.ABB_CRB_15000);
            app.PosicinYOrientacinEfectorFinalLabel.FontSize = 14;
            app.PosicinYOrientacinEfectorFinalLabel.FontWeight = 'bold';
            app.PosicinYOrientacinEfectorFinalLabel.Position = [77 542 249 22];
            app.PosicinYOrientacinEfectorFinalLabel.Text = 'Posición Y Orientación Efector Final';

            % Create XEditFieldLabel
            app.XEditFieldLabel = uilabel(app.ABB_CRB_15000);
            app.XEditFieldLabel.HorizontalAlignment = 'right';
            app.XEditFieldLabel.FontWeight = 'bold';
            app.XEditFieldLabel.Position = [20 497 25 22];
            app.XEditFieldLabel.Text = 'X';

            % Create posX
            app.posX = uieditfield(app.ABB_CRB_15000, 'numeric');
            app.posX.ValueChangedFcn = createCallbackFcn(app, @posXValueChanged, true);
            app.posX.FontWeight = 'bold';
            app.posX.Position = [60 497 100 22];

            % Create YEditFieldLabel
            app.YEditFieldLabel = uilabel(app.ABB_CRB_15000);
            app.YEditFieldLabel.HorizontalAlignment = 'right';
            app.YEditFieldLabel.FontWeight = 'bold';
            app.YEditFieldLabel.Position = [20 463 25 22];
            app.YEditFieldLabel.Text = 'Y';

            % Create posY
            app.posY = uieditfield(app.ABB_CRB_15000, 'numeric');
            app.posY.ValueChangedFcn = createCallbackFcn(app, @posYValueChanged, true);
            app.posY.FontWeight = 'bold';
            app.posY.Position = [60 463 100 22];

            % Create ZEditFieldLabel
            app.ZEditFieldLabel = uilabel(app.ABB_CRB_15000);
            app.ZEditFieldLabel.HorizontalAlignment = 'right';
            app.ZEditFieldLabel.FontWeight = 'bold';
            app.ZEditFieldLabel.Position = [20 429 25 22];
            app.ZEditFieldLabel.Text = 'Z';

            % Create posZ
            app.posZ = uieditfield(app.ABB_CRB_15000, 'numeric');
            app.posZ.ValueChangedFcn = createCallbackFcn(app, @posZValueChanged, true);
            app.posZ.FontWeight = 'bold';
            app.posZ.Position = [60 429 100 22];

            % Create mmLabel
            app.mmLabel = uilabel(app.ABB_CRB_15000);
            app.mmLabel.FontWeight = 'bold';
            app.mmLabel.Position = [166 497 27 22];
            app.mmLabel.Text = 'mm';

            % Create mmLabel_2
            app.mmLabel_2 = uilabel(app.ABB_CRB_15000);
            app.mmLabel_2.FontWeight = 'bold';
            app.mmLabel_2.Position = [166 463 27 22];
            app.mmLabel_2.Text = 'mm';

            % Create mmLabel_3
            app.mmLabel_3 = uilabel(app.ABB_CRB_15000);
            app.mmLabel_3.FontWeight = 'bold';
            app.mmLabel_3.Position = [166 429 27 22];
            app.mmLabel_3.Text = 'mm';

            % Create ParacomenzarutilizacualquiersliderparaconfigurarunanguloLabel
            app.ParacomenzarutilizacualquiersliderparaconfigurarunanguloLabel = uilabel(app.ABB_CRB_15000);
            app.ParacomenzarutilizacualquiersliderparaconfigurarunanguloLabel.HorizontalAlignment = 'center';
            app.ParacomenzarutilizacualquiersliderparaconfigurarunanguloLabel.FontWeight = 'bold';
            app.ParacomenzarutilizacualquiersliderparaconfigurarunanguloLabel.Position = [31 49 344 56];
            app.ParacomenzarutilizacualquiersliderparaconfigurarunanguloLabel.Text = {'Para comenzar, utiliza cualquier slider para configurar un'; 'angulo de rotación ''q'' y se graficará automáticamente, o'; 'ingresa los valores de posición y orientación y presiona el'; 'botón calcular.'};

            % Create CalcularButton
            app.CalcularButton = uibutton(app.ABB_CRB_15000, 'push');
            app.CalcularButton.ButtonPushedFcn = createCallbackFcn(app, @CalcularButtonPushed, true);
            app.CalcularButton.Position = [152 16 100 22];
            app.CalcularButton.Text = 'Calcular';

            % Create Label
            app.Label = uilabel(app.ABB_CRB_15000);
            app.Label.FontWeight = 'bold';
            app.Label.Position = [350 497 25 22];
            app.Label.Text = '°';

            % Create Label_2
            app.Label_2 = uilabel(app.ABB_CRB_15000);
            app.Label_2.FontWeight = 'bold';
            app.Label_2.Position = [350 463 25 22];
            app.Label_2.Text = '°';

            % Create Label_3
            app.Label_3 = uilabel(app.ABB_CRB_15000);
            app.Label_3.FontWeight = 'bold';
            app.Label_3.Position = [350 429 25 22];
            app.Label_3.Text = '°';

            % Create RLabel
            app.RLabel = uilabel(app.ABB_CRB_15000);
            app.RLabel.HorizontalAlignment = 'right';
            app.RLabel.FontWeight = 'bold';
            app.RLabel.Position = [204 497 25 22];
            app.RLabel.Text = 'R';

            % Create roll
            app.roll = uieditfield(app.ABB_CRB_15000, 'numeric');
            app.roll.ValueChangedFcn = createCallbackFcn(app, @rollValueChanged, true);
            app.roll.FontWeight = 'bold';
            app.roll.Position = [244 497 100 22];

            % Create PLabel
            app.PLabel = uilabel(app.ABB_CRB_15000);
            app.PLabel.HorizontalAlignment = 'right';
            app.PLabel.FontWeight = 'bold';
            app.PLabel.Position = [204 463 25 22];
            app.PLabel.Text = 'P';

            % Create pitch
            app.pitch = uieditfield(app.ABB_CRB_15000, 'numeric');
            app.pitch.ValueChangedFcn = createCallbackFcn(app, @pitchValueChanged, true);
            app.pitch.FontWeight = 'bold';
            app.pitch.Position = [244 463 100 22];

            % Create YLabel
            app.YLabel = uilabel(app.ABB_CRB_15000);
            app.YLabel.HorizontalAlignment = 'right';
            app.YLabel.FontWeight = 'bold';
            app.YLabel.Position = [204 429 25 22];
            app.YLabel.Text = 'Y';

            % Create yaw
            app.yaw = uieditfield(app.ABB_CRB_15000, 'numeric');
            app.yaw.ValueChangedFcn = createCallbackFcn(app, @yawValueChanged, true);
            app.yaw.FontWeight = 'bold';
            app.yaw.Position = [244 429 100 22];

            % Show the figure after all components are created
            app.ABB_CRB_15000.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = App_Inversa_ABB_15000

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.ABB_CRB_15000)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.ABB_CRB_15000)
        end
    end
end