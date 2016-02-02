function fk_demo()
	clc;
	clear all;
	close all;
	
	ax = createStage([-60 60 -60 60 -30 70], [-38 29]);
    set(gcf,'units','normalized','outerposition',[0 0 1 1])
%     set(ax, 'CameraUpVector', [1 0 0], 'CameraPosition', [12, 15, -3], 'CameraTarget', [1 0 5]);
	ball1 = createObject(geoEllipsoid(0.5), 'FaceColor', [1 1 0]);
% 	createObject(transform(T_shift(-1, 0, 0), geoGeneric([0, -10 -5; 0 -10 15; 0 10 15; 0 10 -5], [1 2 3 4])), 'FaceColor', [0.9 0.9 0.9]);
	camlight();
	linkColors = {[.4 .4 .4], [1 1 1], [.5 1 .5], [.5 .5 1], [.5 1 1], [1 .5 .5]};
	
    d1 = 14.7;
    a1 = 3.3;
    a2 = 15.5;
    a3 = 13.5;
    d5 = 21.75;
	robot = youBot(d1, a1, a2, a3, d5);
	robot.colorLinks(linkColors{1}, linkColors{2}, linkColors{3}, linkColors{4}, linkColors{5}, linkColors{6});
	robot.setTransparency(0.5);
    
	% controls for
	hOriginCheck = uicontrol('Style', 'checkbox', 'String', 'Show origins', 'Position', [5, 100, 215, 20], 'Callback', @setOriginsVisibility, 'Value', 1);
	[hTheta1Slider, hTheta1Text] = createSlider('Theta1', 5, 80, -169, 169, 0, @updateFromSliders, linkColors{2});
	[hTheta2Slider, hTheta2Text] = createSlider('Theta2', 5, 60, 0, 155, 0, @updateFromSliders, linkColors{3});
	[hTheta3Slider, hTheta3Text] = createSlider('Theta3', 5, 40, -146, 150, 0, @updateFromSliders, linkColors{4});
	[hTheta4Slider, hTheta4Text] = createSlider('Theta4', 5, 20, -102.5, 102.5, 0, @updateFromSliders, linkColors{5});
	[hTheta5Slider, hTheta5Text] = createSlider('Theta5', 5, 0, -167.5, 167.5, 0, @updateFromSliders, linkColors{6});
	    
    updateFromSliders();	    
    
	%zoom(1.5); rotate3d;	

	function setOriginsVisibility(varargin)
		if get(hOriginCheck, 'Value'), robot.showOrigins(); else robot.hideOrigins(); end
    end
	function [hSlider, hText, hCaption] = createSlider(text, x, y, minimum, maximum, initialValue, callback, color)		
		hCaption = uicontrol('Style', 'text', 'Position', [x y 50 20], 'String', text, 'ForegroundColor', brighten(color, -0.5));
		hSlider = uicontrol('Style', 'slider', 'Position', [x + 50, y, 250, 20], 'Min', minimum, 'Max', maximum, 'Value', initialValue, 'Callback', callback);
		hText = uicontrol('Style', 'text', 'Position', [x + 300, y, 45, 20]);
	end

	function updateFromSliders(varargin)		
		thetas = arrayfun(@(hs)get(hs, 'Value'), [hTheta1Slider, hTheta2Slider, hTheta3Slider, hTheta4Slider, hTheta5Slider]);
		arrayfun(@(th, v)set(th, 'String', sprintf('%.1f°', v)), [hTheta1Text, hTheta2Text, hTheta3Text, hTheta4Text, hTheta5Text], thetas);

		thetas = thetas * (pi / 180);   % convert to rad
		robot.setJoins(thetas(1), thetas(2), thetas(3), thetas(4), thetas(5));
		tcp = robot.getTcp();
		ball1.place(T_shift(tcp));        
	end
end