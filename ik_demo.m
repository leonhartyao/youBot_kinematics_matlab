function ik_demo()
	clc;
	clear all;
	close all;
	
% 	figure('Toolbar', 'figure');
	ax = createStage([-6 10 -10 10 -2 12], [-24 41]);
    set(gcf,'units','normalized','outerposition',[0 0 1 1])
%     set(ax, 'CameraUpVector', [1 0 0], 'CameraPosition', [12, 15, -3], 'CameraTarget', [1 0 5]);
%     t_goal = triade(T_unity(), [], 4, 0.1);
%     createObject(transform(T_shift(-1, 0, 0), geoGeneric([0, -10 -5; 0 -10 15; 0 10 15; 0 10 -5], [1 2 3 4])), 'FaceColor', [0.9 0.9 0.9]);
    t_goal = createObject(geoEllipsoid(1),'FaceColor', [0.2 0.2 0.2]);
	camlight();
	linkColors = {[.4 .4 .4], [1 1 1], [.5 1 .5], [.5 .5 1], [.5 1 1], [1 .5 .5]};
	
    % global varibles
    global max_angles min_angles d1 a1 a2 a3 d5
    
    max_angles = [169, 90, 151, 102.5, 167.5].*pi/180;
    min_angles = [-169, -65 -146, -102.5, -167.5].*pi/180;
    
	d1 = 14.7;
    a1 = 3.3;
    a2 = 15.5;
    a3 = 13.5;
    d5 = 21.75;
	robot = youBot(d1, a1, a2, a3, d5);
	robot.colorLinks(linkColors{1}, linkColors{2}, linkColors{3}, linkColors{4}, linkColors{5}, linkColors{6});
	robot.setTransparency(0.5);
    
	% controls for
	hArmConfig1 = uicontrol('Style', 'checkbox', 'String', 'Arm_face_front', 'Position', [5, 120, 215, 20], 'Callback', @updateFromSliders, 'Value', 1);
    hArmConfig3 = uicontrol('Style', 'checkbox', 'String', 'Arm_bended_up', 'Position', [5, 100, 215, 20], 'Callback', @updateFromSliders, 'Value', 1);
	[hXSlider, hXText] = createSlider('X', 0, 80, -54.05, 54.05, 25, @updateFromSliders, [1 0 0]);
	[hYSlider, hYText] = createSlider('Y', 0, 60, -54.05, 54.05, 0, @updateFromSliders, [0 1 0]);
	[hZSlider, hZText] = createSlider('Z', 0, 40, -20.5, 65.45, 40, @updateFromSliders, [0 0 1]);
	[hPhiSlider, hPhiText] = createSlider('Pitch', 0, 20, -180, 180, 0, @updateFromSliders, [0 0 0]);
	[hGammaSlider, hGammaText] = createSlider('Roll', 0, 0, -150, 150, 0, @updateFromSliders, [0 0 0]);
	    
    updateFromSliders();	        

	function [hSlider, hText, hCaption] = createSlider(text, x, y, minimum, maximum, initialValue, callback, color)		
		hCaption = uicontrol('Style', 'text', 'Position', [x y 40 20], 'String', text, 'ForegroundColor', brighten(color, -0.5));
		hSlider = uicontrol('Style', 'slider', 'Position', [x + 40, y, 250, 20], 'Min', minimum, 'Max', maximum, 'Value', initialValue, 'Callback', callback);
		hText = uicontrol('Style', 'text', 'Position', [x + 290, y, 70, 20]);
	end

	function updateFromSliders(varargin)		
        px = get(hXSlider, 'Value');
        py = get(hYSlider, 'Value');
        pz = get(hZSlider, 'Value');
        phi = get(hPhiSlider, 'Value') * pi / 180;
        gamma = get(hGammaSlider, 'Value') * pi / 180;
        % show values
        set(hXText, 'String', sprintf('%0.2f cm', px));
        set(hYText, 'String', sprintf('%0.2f cm', py));
        set(hZText, 'String', sprintf('%0.1f cm', pz));
        set(hPhiText, 'String', sprintf('%0.0f°', phi * 180 / pi));
        set(hGammaText, 'String', sprintf('%0.0f°', gamma * 180 / pi));
    
        if get(hArmConfig1, 'Value') == 1
            arm_face_front = true;
        else arm_face_front = false;
        end
        
        if get(hArmConfig3, 'Value') == 1
            arm_bended_up = true;
        else arm_bended_up = false;
        end
        
        % calculate inverse kinematics
        [jointarray, feasible]= ik_Youbot(px,py,pz,phi,gamma,arm_face_front,arm_bended_up);
        
        robot.setJoins(jointarray(1), jointarray(2), jointarray(3), jointarray(4), jointarray(5));
        % solution valid check
%         if ~feasible
%             return
%         else
%             limit_check = checkLimit(jointarray);
%             if ~limit_check
%                 disp('[WARNING] ik solution out of joint limits!');
%                 return
%             else
%                 robot.setJoins(jointarray(1), jointarray(2), jointarray(3), jointarray(4), jointarray(5));
%             end
%         end
    end

    function [theta,feasible] = ik_Youbot(gx,gy,gz,gphi,ggamma,config1,config3)
        T_B_T = T_shift(gx, gy, gz) * T_rot('xz', gphi, ggamma);  
		t_goal.place(T_B_T);
        
        % first and fifth joint
        if config1
            j1 = atan2(gy,gx); 
            j5 = ggamma;
        else
            j1 = atan2(gy,gx)+pi;  
            j5 = ggamma+pi;
        end
        j1 = angle_normalize(j1);
        j5 = angle_normalize(j5);
        
        % third joint
        if config1
            x4 = sqrt(gx^2+gy^2) - a1 - d5*cos(gphi);
        else
            x4 = sqrt(gx^2+gy^2) + a1 - d5*cos(gphi);
        end
        y4 = gz - d1 - d5*sin(gphi);
        
        j3_cos = (x4^2 + y4^2 - a2^2 - a3^2)/(2*a2*a3);
        if j3_cos > 0.9999999
            j3 = 0;
        elseif j3_cos < -0.9999999
            j3 = pi;
        else
            j3 = acos(j3_cos);
        end
        if config3
            j3 = -j3;
        end  
        
        % second joint
        beta_cos = (x4^2 + y4^2 + a2^2 - a3^2)/(2*a2*sqrt(x4^2 + y4^2));
        if beta_cos > 0.9999999
            beta = 0;
        elseif beta_cos < -0.9999999
            beta = pi;
        else
            beta = acos(beta_cos);
        end
        j2 = atan2(y4, x4)-sign(j3)*beta-pi/2;
        
        % fourth joint determines the pitch of the gripper
        j4 = gphi-pi/2-j2-j3;                     
        
        theta = angle_normalize([j1, j2, j3, j4, j5]);
        
        if ~config1
            theta = -theta;
        end
        
        % check if the goal position can be reached at all
        if sqrt(x4^2 + y4^2)>(a2+a3) || sqrt(x4^2 + y4^2)<abs(a2-a3)
            disp('[WARNING] ik solution is not feasible!');
            feasible = false;
        else
            feasible = true;
        end 
    end
    
    function solution_valid = checkLimit(theta)
%         global max_angles min_angles
        
        solution_valid = true;
        if numel(theta)~=5
            solution_valid = false;
            return
        end
        for i=1:5
            if theta(i)<min_angles(i) ||theta(i)>max_angles(i)
                solution_valid = false;
                return
            end
        end
    end

    function out = angle_normalize(input)
        for i = 1:numel(input)
            if input(i) > pi, input = input-2*pi; end
            if input(i) < -pi, input = input+2*pi; end
        end
        out = input;
    end

end
