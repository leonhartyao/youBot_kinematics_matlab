function main
	clear all; close all;
   
	ax = createStage([-1 6 -3 3 0 13], [-38 29],'kinematische Struktur des youBots');
    k=100;
    
    % DH-Parameter
    d1 = 0.147*k;
    a1 = 0.033*k;
    alpha1 = pi/2;
    a2 = 0.155*k;
    a3 = 0.135*k;
    alpha4 = pi/2;
    d5 = 0.2175*k;
    
    rad = 2;
	height = 4;
    linkColors = {[.3 .3 1], [1 .3 .3], [.3 1 .3], [1 .5 0], [1 0 1]};
    for i = 1:5
		joins(i) = createObject(geoCylinder(rad, height), 'FaceColor', linkColors{i}, 'FaceAlpha', 0.6); 
        joins(i).setWireframeColor([0 0.2 0]);
    end
    for i = 1:7 % incl. end effector
		links(i) = line('XData', [], 'YData', [], 'ZData', [], 'Color', [0 0 0], 'LineWidth', 2);
    end
    
    set(links(1), 'XData', [0 0], 'YData', [0 0], 'ZData', [0 0]);
%   set(links(1), 'XData', [0 0], 'YData', [0 0], 'ZData', [0 (d1 - height - rad)/2]);
	joins(1).place(T_shift(0, 0, -height/2));	
	set(links(2), 'XData', [0 a1], 'YData', [0 0], 'ZData', [0, d1]);
    joins(2).place(T_shift(a1, 0, d1) * T_rot('X', alpha1) * T_shift(0, 0, -height/2));
	set(links(3), 'XData', [a1 a1+a2], 'YData', [0 0], 'ZData', [d1 d1]);
	joins(3).place(T_shift(a1+a2, 0, d1) * T_rot('X', alpha1) * T_shift(0, 0, -height/2));
	set(links(4), 'XData', [a1+a2 a1+a2+a3], 'YData', [0 0], 'ZData', [d1 d1]);
	joins(4).place(T_shift(a1+a2+a3, 0, d1) * T_rot('X', alpha1) * T_shift(0, 0, -height/2));
% 	set(links(5), 'XData', [0 0], 'YData', [a1+a2+a3 a1+a2+a3], 'ZData', [d1 d1]);
	joins(5).place(T_shift(a1+a2+a3+10, 0, d1) * T_rot('Y', alpha4) * T_shift(0, 0, -height/2));
	set(links(6), 'XData', [a1+a2+a3 a1+a2+a3+d5], 'YData', [0 0], 'ZData', [d1 d1]);
	set(links(7), 'XData', [[a1+a2+a3+d5+2 a1+a2+a3+d5 a1+a2+a3+d5 a1+a2+a3+d5+2]], 'YData', [0 0 0 0], 'ZData', [d1+1 d1+1 d1-1 d1-1]);

    T_accu = T_unity();	
	org0 = triade(T_accu, 0.3*[1 1 1], 5, 0.15);
    
    % Transformation according to DH-Parameters
    T_arr = {T_dh(0, d1, a1, alpha1), T_dh(0, 0, a2, 0), T_dh(0, 0, a3, 0), T_dh(pi/2, 0, 0, alpha4), T_dh(0, d5, 0, 0)};
    org_sizes = [5 5 7 5 4];
    for i = 1:5
		T_accu = T_accu * T_arr{i};
		orgs(i) = triade(T_accu, linkColors{i}, org_sizes(i), 0.15);
		orgs(i).hide();
    end
    
    btn = uicontrol('style', 'pushbutton', 'String', 'Next Step', 'Callback', @(varargin)uiresume, 'Position', [4 4 120 32], 'Interruptible', 'off');
	dhParams = [0, d1, a1, alpha1; 0, 0, a2, 0; 0, 0, a3, 0; pi/2 0, 0, alpha4; 0, d5, 0, 0];
	zoom(1.5);
	rotate3d();
	
	uiwait;
    % show coordinate of each joint
    for i=1:5
		orgs(i).show();
		uiwait;
    end
    
    for i=1:5, orgs(i).setTransparency(0.3); end
	uiwait;
    
    dhOrg = T_unity();
	dhTriade = triade(dhOrg, [], 7, 0.15);
	uiwait;
    
    org0.setTransparency(1);	
	for iJoint = 1:size(dhParams, 1)
		% theta_i
		dhTriade.highlight('Z');
		animate(linspace(0, 2 * pi, 20), @(theta)dhTriade.place(dhOrg * T_rot('Z', theta)));					
		dhTriade.resetHighlight();
		uiwait;

		if dhParams(iJoint, 1) ~= 0
			dhTriade.highlight('Z');
			animate(linspace(0, dhParams(iJoint, 1), 15), @(theta)dhTriade.place(dhOrg * T_rot('Z', theta)));
			dhOrg = dhOrg * T_rot('Z', dhParams(iJoint, 1));
			dhTriade.resetHighlight();
			uiwait;
		end

		% d_i
		dhTriade.highlight('Z');
		if dhParams(iJoint, 2) == 0,
			animate([linspace(0, -1, 3) linspace(-1, 1, 6) linspace(1, 0, 3)], @(d)dhTriade.place(dhOrg * T_shift(0, 0, d)));
		else
			animate(linspace(0, dhParams(iJoint, 2), 20), @(d)dhTriade.place(dhOrg * T_shift(0, 0, d)));
			dhOrg = dhOrg * T_shift(0, 0, dhParams(iJoint, 2));
		end
		dhTriade.resetHighlight();
		uiwait;

		% a_i
		dhTriade.highlight('X');
		if dhParams(iJoint, 3) == 0,
			animate([linspace(0, -.5, 3) linspace(-.5, .5, 6) linspace(.5, 0, 3)], @(a)dhTriade.place(dhOrg * T_shift(a, 0, 0)));
		else
			animate(linspace(0, dhParams(iJoint, 3), 20), @(a)dhTriade.place(dhOrg * T_shift(a, 0, 0)));
			dhOrg = dhOrg * T_shift(dhParams(iJoint, 3), 0, 0);
		end
		dhTriade.resetHighlight();
		uiwait;

		% alpha_i
		dhTriade.highlight('X');						
		if dhParams(iJoint, 4) == 0,
			animate([linspace(0, -pi/4, 3) linspace(-pi/4, pi/4, 6) linspace(pi/4, 0, 3)], @(alpha)dhTriade.place(dhOrg * T_rot('X', alpha)));
		else
			animate(linspace(0, dhParams(iJoint, 4), 20), @(alpha)dhTriade.place(dhOrg * T_rot('X', alpha)));
			dhOrg = dhOrg * T_rot('X', dhParams(iJoint, 4));
		end
		dhTriade.resetHighlight();
		uiwait;
		orgs(iJoint).setTransparency(1);
	end

	dhTriade.hide();
	set(btn, 'Enable', 'off');
    
end