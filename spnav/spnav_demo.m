function spnav_demo()
	
    if ~spnav('open')
        clear spnav;
        if ~spnav('open')    
            error('Could not open Space Navigator device. Make sure that the device is attached and spacenavd is running!');
        end
    end
	
	f = figure('toolbar', 'figure', 'NumberTitle', 'off', 'Name', 'Space Navigator Demo', 'units','normalized','outerposition',[0 0 1 1], 'CloseRequestFcn', @closeHandler);
	labelSettings = {'FontName', 'Verdana', 'FontWeight', 'bold'};
	axAll = axes('Parent', f, 'OuterPosition', [0 0 0.5 1], 'DataAspectRatio', [1 1 1], 'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on');    	
	xlabel('North (X) / m', labelSettings{:}, 'Color', [1 0 0]);
    ylabel('East (Y) / m', labelSettings{:}, 'Color', [0 0.6 0]);
    zlabel('Down (Z) / m', labelSettings{:}, 'Color', [0 0 1]);    	
	title('Component Display')
	camlight();
	
	dirLength = 10;
	line('Parent', axAll, 'XData', [-dirLength dirLength NaN 0 0 NaN 0 0], 'YData', [0 0 NaN -dirLength dirLength NaN 0 0], 'ZData', [0 0 NaN 0 0 NaN -dirLength dirLength], ...
		 'Color', [0 0 0], 'LineWidth', 2, 'LineStyle', '-.');

	xArrow = createObject(geoGeneric([], []), 'FaceColor', [1 0 0]);
	yArrow = createObject(geoGeneric([], []), 'FaceColor', [0 1 0]);
	zArrow = createObject(geoGeneric([], []), 'FaceColor', [0 0 1]);
	rxArrow = createObject(geoGeneric([], []), 'FaceColor', [1 0 0]);
	ryArrow = createObject(geoGeneric([], []), 'FaceColor', [0 1 0]);
	rzArrow = createObject(geoGeneric([], []), 'FaceColor', [0 0 1]);
	
	view([30 12]);
	 
	axVec = axes('Parent', f, 'OuterPosition', [0.5 0 0.5 1], 'DataAspectRatio', [1 1 1], 'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on', ...
				 'XLim', dirLength * [-1 1], 'YLim', dirLength * [-1 1], 'ZLim', dirLength * [-1 1]);
	xlabel('North (X) / m', labelSettings{:}, 'Color', [1 0 0]);
    ylabel('East (Y) / m', labelSettings{:}, 'Color', [0 0.6 0]);
    zlabel('Down (Z) / m', labelSettings{:}, 'Color', [0 0 1]);    		
	title('Vector Display')
	camlight();
	
	transArrow = createObject(geoGeneric([], []), 'FaceColor', [1 1 0]);
	hTransLine = line('Parent', axVec, 'XData', [], 'YData', [], 'ZData', [], 'Color', [0 0 0], 'LineWidth', 2, 'LineStyle', '-.');
	rotArrow = createObject(geoGeneric([], []), 'FaceColor', [1 0.5 0]);
	rotArrowTurn = createObject(geoGeneric([], []), 'FaceColor', [1 0.5 0]);
	hRotLine = line('Parent', axVec, 'XData', [], 'YData', [], 'ZData', [], 'Color', [0 0 0], 'LineWidth', 2, 'LineStyle', '-.');
	
	view([30 12]);
	rotate3d();
	
	run = true;
	
	while run		
		res = spnav();
		x = res.trans(1);
		y = res.trans(3);
		z = res.trans(2);
		rx = res.rot(1);
		ry = res.rot(3);
		rz = res.rot(2);
		fprintf('x/y/z = %5.2f/%5.2f/%5.2f, rx/ry/rz = %5.2f/%5.2f/%5.2f', x, y, z, rx, ry, rz);
		for i = 1:numel(res.buttons)
			if res.buttons(i)
				if res.pressEvents(i) > 0
					fprintf(', button %d pressed', i);
				else fprintf(', button %d down', i);
				end
			elseif res.releaseEvents(i) > 0
				fprintf(', button %d released', i);
			end				
		end
		fprintf('\n');
        if res.pressEvents(1) > 0
            closeHandler(f); 
            break;
        end

		minTrans = 0.1;
		if abs(x) >= minTrans
			xArrow.setGeometry(geoArrow([sign(x) 0 0], abs(x) * dirLength, 0.3, 1, 0.7));
			xArrow.show();
		else xArrow.hide();
		end
		if abs(y) >= minTrans
			yArrow.setGeometry(geoArrow([0 sign(y) 0], abs(y) * dirLength, 0.3, 1, 0.7));
			yArrow.show();
		else yArrow.hide();
		end
		if abs(z) >= minTrans
			zArrow.setGeometry(geoArrow([0 0 sign(z)], abs(z) * dirLength, 0.3, 1, 0.7));
			zArrow.show();
		else zArrow.hide();
		end
		
		minRot = 0.1;
		if abs(rx) >= minRot
			rxArrow.setGeometry(transform(T_rot('YX', pi/2, pi) * T_scale(1, sign(rx), 1) * T_shift(0, 0, 0.7 * dirLength), ...
								geoArrowTurn(3, 0.2, pi * abs(rx), 0.5, 0.7)));
			rxArrow.show();
		else rxArrow.hide();
		end
		if abs(ry) >= minRot
			ryArrow.setGeometry(transform(T_rot('XY', -pi/2, -pi/2) * T_scale(1, sign(ry), 1) * T_shift(0, 0, 0.7 * dirLength), ...
								geoArrowTurn(3, 0.2, pi * abs(ry), 0.5, 0.7)));
			ryArrow.show();
		else ryArrow.hide();
		end
		if abs(rz) >= minRot
			rzArrow.setGeometry(transform(T_scale(1, sign(rz), 1) * T_shift(0, 0, 0.7 * dirLength), ...
								geoArrowTurn(3, 0.2, 1.9 * pi * abs(rz), 0.5, 0.7)));
			rzArrow.show();
		else rzArrow.hide();
		end
		
		trans = [x; y; z];
		transNorm = norm(trans);
		if transNorm >= minTrans
			normedTrans = trans / transNorm;
			set(hTransLine, 'XData', [0, normedTrans(1) * dirLength], 'YData', [0, normedTrans(2) * dirLength], 'ZData', [0, normedTrans(3) * dirLength]);
			transArrow.setGeometry(geoArrow(normedTrans, transNorm * dirLength, 0.3, 1, 0.7));
			transArrow.show();
		else
			transArrow.hide();
			set(hTransLine, 'XData', [], 'YData', [], 'ZData', []);
		end
		
		rot = [rx; ry; rz];
		rotNorm = norm(rot);
		if rotNorm >= minRot
			normedRot = rot / rotNorm;
			set(hRotLine, 'XData', [0, normedRot(1) * dirLength], 'YData', [0, normedRot(2) * dirLength], 'ZData', [0, normedRot(3) * dirLength]);
			rotArrow.setGeometry(geoArrow(normedRot, rotNorm * dirLength, 0.2, 0.7, 0.5));
			rotArrow.show();
			rotArrowAxis = cross([0; 0; 1], normedRot);
			rotArrowAngle = asin(norm(rotArrowAxis));
			if rz < 0
				rotArrowAngle = pi - rotArrowAngle;
			end
			rotArrowTurn.setGeometry(transform(T_rot('K', [rotArrowAxis; rotArrowAngle]) * T_shift(0, 0, 0.5 * dirLength), ...
									 geoArrowTurn(2, 0.15, 1.8 / sqrt(3)  * pi * rotNorm, 0.4, 0.6)));
			rotArrowTurn.show();
		else
			rotArrow.hide();
			rotArrowTurn.hide();
			set(hRotLine, 'XData', [], 'YData', [], 'ZData', []);
		end
		
		
		pause(0.1);
	end
	
	function closeHandler(varargin)
		run = false;
		delete(f);	
		if ~spnav('close')
			warning('Closing Space Navigator device failed');
		end
	end
	
end