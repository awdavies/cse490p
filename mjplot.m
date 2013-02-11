%MJPLOT   Plot MuJoCo model
%
%  MJPLOT plots the current model
%
%  MJPLOT(DATA) plots a trajectory (sequence of models) with poses
%				given by the elements of the struct-array DATA. See
%				mjsim.m for more information.
%
%  The plot is interactive:
%
%    Left mouse drag     rotate camera around look-at
%    Right mouse drag    translate in vertical viewplane
%    Scroll              zoom in/out
%
%    Left doubleclick    center view on selected object
%    Right doubleclick   select object for manipulation and print info
%
%	 Left+right drag	 draw rope to selected geom	 
%	 Arrow key			 select geoms:
%						 left	- previous geom
%						 right	- next geom
%						 up		- select
%						 down	- deselct
%
%	 return				 start / stop simulation, save DATA to workspace
%    backspace			 reset to initial pose
%    A                   auto scale and center once
%    L					 auto center mode (following)
%    backquote\tilde	 transparency
%
%  The following keys enable/disable elements of the plot:
%
%    G    geoms (cycles through flat/phong/off)
%    I    inertias
%    T    tendons
%    S    sites
%    C    contact points
%    F    contact forces
%    D    ground plane
%	 +    GUI panel

% This file is part of the MuJoCo software.
% (C) 2012 Emo Todorov. All rights reserved.


function fig = mjplot(DATA)

% make sure model is loaded
if ~mj('ismodel'),
	error 'MuJoCo model must be loaded first';
end
model		= mj('getmodel');
model_id	= [model.nbuffer model.ngeom];

% given trajectory or current pose
if nargin == 0
	DATA = getData;
end

% number of poses
nposes	= length(DATA);

% turn focus to the relevant figure
fig = findobj(0,'Tag','mjplot');
if  isempty(fig)
	fig = initFigure(model, nposes);
else
	set(0,'currentfigure',fig)
end

% get figure data
figdata = getappdata(fig);

% init figure if model or npose mismatch
if isempty(figdata) ||  any(figdata.model_id ~= model_id) || length(figdata.handles) ~= nposes
	fig		= initFigure(model, nposes);
	figdata	= getappdata(fig);
end

% loop over poses and update handles
for pose = 1:nposes
	updateHandles(model, DATA(pose), figdata.handles(pose), figdata.enable);
end

% update the rope if it's active
if ~isempty(getappdata(fig,'rope'))
	newrope = false;
	ropeUpdate(fig, newrope);
end

% update camera if spatial-following is enabled
cam		= get(figdata.main_ax, 'userdata');
if cam.follow_CoM
	cam.lookat = mj('spatial')';
	set(figdata.main_ax, 'userdata', cam);
	camUpdate(figdata.main_ax, '', 0);
end

% set color of the status bar text
running		= figdata.running_sim;
info		= findobj(fig,'tag','info_text');
if running
	set(info,'visible','on')
else
	set(info,'visible','off')
end

drawnow;

if nargout == 0
	clear fig;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   update graphics objects
function updateHandles(model, data, handles, enable)

% === constants ===
forceScaling		= 1; % scaling from force to length
tendonColorScaling	= 50;% scaling from tension to color


% === geoms ===
if strcmp(enable.geom,'on')
	xpos	= data.geom_xpos;
	xmat	= data.geom_xmat;
	
	for i=1:model.ngeom,
		geom	= handles.geoms(i);
		if isnan(geom)
			continue;
		end	

		% vertices in local coordinates
		vert		= get(geom,'userdata');

		% extract position and orientation, compute vertices
		pos		= xpos(i,:);
		rot		= reshape(xmat(i,:), [3, 3]);
		vert	= vert * rot + ones(size(vert,1),1) * pos;

		% update geom, skip world geoms for pose > 1
		set(geom,'vertices', vert, 'visible', 'on')
		setappdata(geom, 'type', 'geom')
		setappdata(geom, 'id', i)
		setappdata(geom, 'pos', pos)
	end
end

% === inertias ===
if strcmp(enable.inertia,'on')
	xpos	= data.xpos;
	xmat	= data.xmat;
	for i=2:model.nbody,
		inertia	= handles.inertias(i);
		if isnan(inertia)
			continue;
		end	

		% vertices in local coordinates
		vert		= get(inertia,'userdata');

		% extract position and orientation, compute vertices
		pos		= xpos(i,:);
		rot		= reshape(xmat(i,:), [3, 3]);
		vert	= vert * rot + ones(size(vert,1),1) * pos;

		% update geom, skip world geoms for pose > 1
		set(inertia,'vertices', vert, 'visible', 'on')
		setappdata(inertia,'type','inertia')
		setappdata(inertia, 'body', i)
		setappdata(inertia, 'pos', pos)
	end
end

% === sites ===
if strcmp(enable.site,'on')
	xpos	= data.site_xpos;
	for i=1:model.nsite
		site	= handles.sites(i);
		pos		= xpos(i,:);
		vert	= get(handles.sites(i),'userdata');
		vert	= vert + ones(size(vert,1),1)*pos;
		set(site,'vertices', vert, 'visible', 'on')
		setappdata(site, 'type', 'site')
		setappdata(site, 'id', i)
		setappdata(site, 'pos', pos)		
	end
end

% === tendons ===
if strcmp(enable.tendon,'on')
	ten_wrapadr	= data.ten_wrapadr;
	ten_wrapnum	= data.ten_wrapnum;
	wrap_obj	= data.wrap_obj;		% TODO use wrap_obj to NOT draw tendon inside the w/o
	wrap_xpos	= data.wrap_xpos;	
	act			= data.act;

	for i=1:model.ntendon
		pnt = [wrap_xpos(ten_wrapadr(i) +(1 : ten_wrapnum(i)), :); nan(1,3)];

		% set tendon color using tension in the tendon
		actuator = get(handles.tendons(i),'userdata');	
		if isnan(actuator)
			tension		= 0;
		else
			tension		= min(1, max(0, -act(actuator)*tendonColorScaling));
		end
		color	= [tension 0.3 (1-tension)];

		% plot tendon
		if size(pnt,1)>1,
			set(handles.tendons(i),'xdata',pnt(:,1), 'ydata',pnt(:,2), 'zdata',pnt(:,3), ...
				'edgecolor', color, ...
				'tag', 'tendon', 'visible', 'on');
		end
	end
end

if isfield(data,'contact')
	contact	= data.contact;

	% === contacts ===
	if strcmp(enable.contact,'on')
		for i=1: model.ncmax 
			if i <= length(contact)
				con = handles.contacts(i);
				% spheres at the contact points
				pos		= contact(i).pos';		
				vert	= get(con,'userdata');
				vert	= vert + ones(size(vert,1),1)*pos;
				set(con,'vertices', vert,'tag','contact', 'visible', 'on')	
				setappdata(con, 'type', 'contact')
				setappdata(con, 'id', i)
				setappdata(con, 'pos', pos)
				setappdata(con, 'dim', contact(i).dim)
				setappdata(con, 'geom1', contact(i).obj1)
				setappdata(con, 'geom2', contact(i).obj2)
			else
				% make inactive contacts invisible
				set(handles.contacts(i),'visible', 'off', 'tag','disabled_contact')
			end
		end
	end

	% === contact forcess ===
	if strcmp(enable.force,'on')
		for i=1: model.ncmax 
			if i <= length(contact)
				pos = contact(i).pos;
				% contact force lines
				forcet = contact(i).force;
				force = contact(i).frame(1:3) * forcet(1) + ...
						contact(i).frame(4:6) * forcet(2) + ...
						contact(i).frame(7:9) * forcet(3);
				set(handles.contactforces(i),...
					'xdata', [pos(1) pos(1)+force(1)*forceScaling], ...
					'ydata', [pos(2) pos(2)+force(2)*forceScaling], ...
					'zdata', [pos(3) pos(3)+force(3)*forceScaling], 'visible', 'on');
			else
				% make inactive forces invisible
				set(handles.contactforces(i),'visible', 'off')		
			end
		end
	end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   make figure with camera controls, initialize graphics objects
function fig = initFigure(model, nposes)

% find the figure
fig = findobj(0,'Tag','mjplot');

if ~isempty(fig)	% reuse existing figure (soft-reset)
	% clear main axis
	main_ax = getappdata(fig,'main_ax');
 	delete(get(main_ax,'children'));
	
else				% new figure and children
	clf('reset');
	fig = gcf;
	monitors	= get(0,'mon');
	fx0			= min(monitors(:,1));
	figpos		= [fx0+500, 300, 700, 500];
	set(fig,'position', figpos);	
	
	set(fig, 'toolbar', 'none', 'menubar', 'none', ...
		'name', ['MuJoCo: ', model.model_name], ...
		'Tag', 'mjplot', 'NumberTitle','off', ...
		'renderer', 'opengl', 'color', [0 0 0], ...
		'KeyPressFcn', @keyPress, ...
		'WindowButtonDownFcn', @mouseDown, ...
		'WindowScrollWheelFcn', @mouseWheel, ...
		'WindowButtonUpFcn', @mouseUp,...
		'WindowKeyReleaseFcn', @keyRelease);	

	% make main axis
	main_ax = axes('outerposition', [0 0 1 1],'projection', 'perspective');
	axis equal off;

	% info axis
	info_ax = axes('xtick',[],'ytick',[],'position', [0 0 1 .1], 'color',.7*[1 1 1],'visible','off');	

	% text object
	str	= help_text();
	text(.01,.5,str,'tag','info_text','color','w',...
		  'fontname','courier','fontsize',8,'VerticalAlignment','middle')		

	% GUI panel 
	GUI_ax = uipanel('BorderType','etchedin','Units','normalized',...
				'Title','',...
				'TitlePos','centertop','position', [0.75 0 0.25 1], 'visible','off');
	setappdata(fig, 'GUI_ax', GUI_ax)			
	
	% put graphics globals in figure
	setappdata(fig, 'main_ax', main_ax);
	setappdata(fig, 'info_ax', info_ax);
	setappdata(fig, 'selected_geom', 0)
	setappdata(fig, 'rope', [])
	setappdata(fig, 'alternate', false)
	setappdata(fig, 'transparency', false)
	setappdata(fig, 'running_sim', false);

	% default enable flags
	enable.ground	= 'on';
	enable.geom		= 'on';
	enable.inertia	= 'off';	
	enable.site		= 'on';
	enable.tendon	= 'on';
	enable.contact	= 'off';
	enable.force	= 'off';
	setappdata(fig, 'enable', enable);
end

% make sure figure is active
set(0,'currentfigure',fig);

% reset name and model_id
set(fig,'name', ['MuJoCo: ', model.model_name]);	
setappdata(fig, 'model_id',[model.nbuffer model.ngeom]);

% make sure main axis is active
set(fig,'CurrentAxes',main_ax);

% make object handles
handles = makeObjects(model);

% duplicate the pose-relevant handles (not including world and camera)
if nposes > 1
	for pose = 2:nposes
		handles(pose).sites				= copyobj(handles(1).sites, main_ax);
		handles(pose).tendons			= copyobj(handles(1).tendons, main_ax);
		handles(pose).contacts			= copyobj(handles(1).contacts, main_ax);
		handles(pose).contactforces		= copyobj(handles(1).contactforces, main_ax);
		handles(pose).inertias			= nan(model.nbody,1);
		% ignore first body (=world)
		handles(pose).inertias(2:end)	= copyobj(handles(1).inertias(2:end), main_ax);
		handles(pose).geoms				= nan(model.ngeom,1);
		for i = 1:model.ngeom
			g	= handles(1).geoms(i);
			if ~isnan(g) && getappdata(g,'parent') ~= 0 % don't duplicate world geoms
				handles(pose).geoms(i) = copyobj(g, main_ax);
			end
		end
	end
end

% put handles in figure
setappdata(fig, 'handles', handles);

% get spatial parameters
[center, sizes] = mj('spatial');
extent			= max([1E-10; sizes]);
scale			= 10^ceil(log10(extent)-.5)/100;

% reset main axis scaling (disable?)
% set(main_ax,'xlim',[-7 7]*scale,'ylim',[-7 7]*scale,'zlim',[-7 7]*scale);

% light
hl = light('position', [0 0 1], 'style', 'local');

% init camera and light, assign to axes, update
cam = struct('lookat', center', 'angle', 45, ...
	'azimuth', -90, 'elevation', 30, 'distance', 2*extent, ...
	'mouselast', [0 0], 'buttonlast', 'none', 'light', hl, ...
	'follow_CoM', false);
set(main_ax, 'userdata', cam);
camUpdate(main_ax, '');	


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  update camera parameters
function camUpdate(ax, mode, data)

% get camera params
cam		= get(ax, 'userdata');
az		= cam.azimuth/180*pi;

% rotation scaling 
sclRot = 0.3;

% translation scaling
position	= get(gcf, 'position');
szy			= position(1,4);
sclMove		= 2*cam.distance*tan(cam.angle/180*pi/2.0)/szy;

% update parameters
switch mode
	case 'rotate'
		cam.azimuth	  = mod(cam.azimuth - data(1)*sclRot, 360);
		cam.elevation = min(90, max(-90, cam.elevation - data(2)*sclRot));	
	case 'altrotate'
		cam.azimuth	  = mod(cam.azimuth + data(1)*sclRot, 360);
		cam.elevation = min(90, max(-90, cam.elevation + data(2)*sclRot));			
	case 'move1'
		cam.lookat(1) = cam.lookat(1) + sclMove*data(1)*sin(az);
		cam.lookat(2) = cam.lookat(2) - sclMove*data(1)*cos(az);
		cam.lookat(3) = cam.lookat(3) - sclMove*data(2);
	case 'move2'
		cam.lookat(1) = cam.lookat(1) + sclMove*(data(2)*cos(az) + data(1)*sin(az));
		cam.lookat(2) = cam.lookat(2) + sclMove*(data(2)*sin(az) - data(1)*cos(az));
	case {'zoom','altzoom'}
		cam.distance  = max(0.001, cam.distance - log(1+cam.distance/3) * data(1));
end

% get camera position
az			= cam.azimuth/180*pi;
el			= cam.elevation/180*pi;
vec			= [cos(el)*cos(az), cos(el)*sin(az), sin(el)];
campos		= cam.lookat + cam.distance*vec;

% in 'alt' mode move the lookat rather than camera
if strcmp(mode,'altrotate') || strcmp(mode,'altzoom')
	cam.lookat	= cam.lookat - campos + get(ax,'cameraposition');
	campos		= get(ax,'cameraposition');		
end

% refresh light
set(cam.light, 'position', campos);

% update axis
set(ax, 'userdata', cam);
set(ax, 'cameraviewangle', cam.angle, ...
		'cameratarget', cam.lookat, ...
		'cameraposition', campos);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  process key and mouse callbacks

function keyPress(fig, evt)

% get axis and camera
ax		= getappdata(fig,'main_ax');
cam		= get(ax, 'userdata');
enable	= getappdata(fig,'enable');

% make toggle()
on_off_str	= {'off', 'on'}; 
toggle		= @(state) on_off_str{~strcmp(state,'on')+1};

% process key
switch evt.Key	
	case 'equal'					% --- GUI toggling
		 GUI	=  getappdata(fig, 'GUI_ax');
		 set(GUI,'visible',toggle(get(GUI,'visible')));
	
	case {'return','space'}			% --- start/stop simulation
		running = getappdata(fig,'running_sim');
		running = ~running;
		setappdata(fig,'running_sim',running);
		if running
			if isempty(which('mjsim'))
				disp('''mjsim'' not found.')
				setappdata(fig,'running_sim',false);
			else
				DATA = mjsim;
				assignin('base','DATA',DATA);		
			end
		else
			% set color of the status bar text
			info		= findobj(fig,'tag','info_text');
			set(info,'visible','off')		
		end
									% --- geom selection
	case {'leftarrow','rightarrow','uparrow','downarrow'}
		select_geom(fig, evt.Key);
		
	case ']'						% --- zoom
		camUpdate(ax, 'zoom', -1);
	case '['
		camUpdate(ax, 'zoom', 1);
		
	case 'l'						% --- follow CoM
		cam.follow_CoM = ~cam.follow_CoM;
		set(ax, 'userdata', cam);		
		if cam.follow_CoM
			cam.lookat = mj('spatial')';
			set(ax, 'userdata', cam);
			camUpdate(ax, '', 0);
		end
		
	case 'a'						% --- auto scale
		[center, sizes] = mj('spatial');
		extent = max([1E-10; sizes]);
		cam.lookat = center';
		cam.distance = 2*extent;
		set(ax, 'userdata', cam);
		camUpdate(ax, '', 0);
									% --- visibility		
	case 'g'	%  cycle off-flat-phong
		geoms = findobj(fig, 'tag', 'geom');
		if strcmp(enable.geom,'off')
			enable.geom = toggle(enable.geom);
			set(geoms, 'visible', 'on', 'facelighting', 'flat');
		elseif strcmp(get(geoms(1),'facelighting'),'flat')
			set(geoms, 'visible', enable.geom, 'facelighting', 'phong');				
		else
			enable.geom = toggle(enable.geom);
			set(geoms, 'visible', 'off', 'facelighting', 'flat');								
		end
	case 'i'	%  inertia
		enable.inertia = toggle(enable.inertia);
		set(findobj(fig, 'tag', 'inertia'), 'visible', enable.inertia);		
	case 's'	%  site
		enable.site = toggle(enable.site);
		set(findobj(fig, 'tag', 'site'), 'visible', enable.site);		
	case 't'	%  tendon
		enable.tendon = toggle(enable.tendon);
		set(findobj(fig, 'tag', 'tendon'), 'visible', enable.tendon);		
	case 'c'	%  contact
		enable.contact = toggle(enable.contact);
		set(findobj(fig, 'tag', 'contact'), 'visible', enable.contact);		
	case 'f'	%  force
		enable.force = toggle(enable.force);
		set(findobj(fig, 'tag', 'force'), 'visible', enable.force);
	case 'd'	%  ground
		enable.ground = toggle(enable.ground);
		set(findobj(fig, 'tag', 'ground'), 'visible', enable.ground);
		
	case 'slash'						% --- alternate mode 
		d								%     (using 'slash' because of matlab limitations)
		setappdata(fig,'alternate',true)
		
	case 'backquote'				% --- transparency
		 geoms		= findobj(fig, 'tag', 'geom');
		 if getappdata(fig, 'transparency')
			 set(geoms,'facealpha',1)
			 setappdata(fig, 'transparency', false)
		 else
			set(geoms,'facealpha',.5)
			 setappdata(fig, 'transparency', true)			 
		 end		
end

setappdata(fig,'enable',enable)
setappdata(fig,'lastkey',evt.Key)

function keyRelease(fig, ~)
setappdata(fig,'alternate',false)

function i = select_geom(fig, key)

% get geoms and i
handles = getappdata(fig,'handles');
geoms	= handles.geoms;
ngeoms	= length(geoms);
i		= getappdata(fig,'selected_geom'); 

% values for selected geom i:
% 0: none, i>0 selected, i<0 previously-selected

if i > 0 % a geom is already selected, darken it
	set(geoms(i),'AmbientStrength', 0.3);
end

% process selection command
if strcmp(key,'downarrow')
	i = min(0,-i);				% deselect or select none
else
	switch key
		case 'uparrow'
			i = max(1,abs(i));		% reselect or select the first geom
		case 'leftarrow'
			i = mod(i,ngeoms)+1;		% select the next geom
		case 'rightarrow'
			i = mod(i-2,ngeoms)+1;	% select the previous geom
		otherwise 
			if isnumeric(key)			% select this geom
				i = key;
			end
	end	
	while isnan(geoms(i)) % skip plane geoms
		switch key
			case 'leftarrow'
				i = mod(i,ngeoms)+1;		% skip down
			otherwise 
				i = mod(i-2,ngeoms)+1;	% skip up
		end
	end
end

% brighten the selected geom
if i > 0
	set(geoms(i),'AmbientStrength', 0.9);
end
setappdata(fig,'selected_geom',i);


function mouseDown(fig, ~)
% get button type
% left:'normal', right:'alternate', both:'extend', doubleclick:'open'
button = get(fig, 'SelectionType'); 

% save last point 
ax				= getappdata(fig,'main_ax');
cam				= get(ax, 'userdata');
cam.mouselast	= get(fig,'CurrentPoint');

% save last click type
if ~strcmp(button, 'open')
	cam.buttonlast = button;
end
set(ax, 'userdata', cam);

% handle two-button click
if strcmp(button, 'extend')
	newrope		= true;
	ropeUpdate(fig, newrope);
	cursor      = findobj(fig, 'tag', 'cursor');
	set(cursor,'visible','on');
end

% set motion callback
set(fig, 'WindowButtonMotionFcn', @mouseMove);

% handle double-click
if strcmp(button, 'open')
	info	= getappdata(gco);
	if isfield(info,'id')
		if strcmp(cam.buttonlast, 'normal'),	% left double-click: center camera on object
			cam.lookat = info.pos;
			set(ax, 'userdata', cam);
			camUpdate(ax, '', 0);
		else									% right double-click: select geom and print info
			if strcmp(get(gco,'tag'), 'geom')
				select_geom(fig, info.id);
			end
			fprintf('%s: %d   position: [%1.1g %1.1g %1.1g]\n', info.type, info.id, info.pos);
		end			
	end
	set(fig, 'WindowButtonMotionFcn', '');
end


function mouseUp(fig, ~)

% clear motion callback
set(fig, 'WindowButtonMotionFcn', '');

ax			= getappdata(fig,'main_ax');
children    = get(ax, 'children');
cursor      = findobj(children, 'tag', 'cursor');
set(cursor,'visible','off');

setappdata(fig,'rope',[]);
setappdata(fig,'alternate',false);


function mouseWheel(fig, evt)

alternate	= getappdata(fig,'alternate');

% zoom
ax			= getappdata(fig,'main_ax');

if ~alternate
	camUpdate(ax, 'zoom', evt.VerticalScrollCount);
else
	camUpdate(ax, 'altzoom', evt.VerticalScrollCount);
end	



function mouseMove(fig, ~)

% update last point, get move distance
ax			= getappdata(fig,'main_ax');

cam = get(ax, 'userdata');
pnt = get(fig, 'CurrentPoint');
dst = pnt - cam.mouselast;
cam.mouselast = pnt;
set(ax, 'userdata', cam);

% move according to button state
button		= get(fig, 'SelectionType');
alternate	= getappdata(fig,'alternate');
switch button
	case 'normal'
		if ~alternate
			camUpdate(ax, 'rotate', dst);
		else
			camUpdate(ax, 'altrotate', dst);
		end			
	case 'alt'
		if ~alternate
			camUpdate(ax, 'move1', dst);
		else
			camUpdate(ax, 'move2', dst);
		end
	case 'extend'
		newrope		= false;
		ropeUpdate(fig, newrope);		
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% make all the object handles

function handles = makeObjects(m)

% auto scale
[~, sizes]  = mj('spatial');
extent		= max([1E-10; sizes]);
scale		= 10^ceil(log10(extent)-.5);

% prepare sphere
[x,y,z]			= sphere(15);
sph				= surf2patch(x,y,z);
sph.vertices	= sph.vertices * scale * 0.01;

% === ground ===
groundhandle = makeGround(m,scale);

% === cursor === 
cursorhandle = patch('edgecolor', 'r', 'edgealpha', 0.5, 'linewidth', 3, ...
					'tag', 'cursor', 'visible', 'off', 'linesmoothing', 'on');				

						
% === joints ===	TODO		
						
% === geoms ===
geomhandles = makeGeoms(m);

% === equivalent inertias ===
inertiahandles = makeInertias(m);

% === sites ===
sph_col = [1 1 1; 0 1 1; 1 0 1];
sitehandles = zeros(1,m.nsite);
for i=1:m.nsite
	grp = min(2, max(0, m.site_group(i))) + 1;
	sitehandles(i) = patch( 'vertices',sph.vertices,...
							'faces', sph.faces, 'edgecolor', 'none', ...
							'facecolor', sph_col(grp,:), 'userdata', sph.vertices, ...
							'tag', 'site', 'visible', 'off');
end

% === tendons ===
tendon_act = nan(m.ntendon,1);
for i=1:m.nu3
	dyntype = m.actuator_dyntype(i+m.nu2);
	if dyntype==2 % why only mjDYN_INTEGRATOR ?
		actuated_tendon				= m.actuator_trnid(m.nu2+i,1)+1;
		tendon_act(actuated_tendon)	= i;
	end
end

tendonhandles = zeros(1,m.ntendon);
for i=1:m.ntendon	
	tendonhandles(i) = patch('linewidth', 1.5,'tag', 'tendon', 'userdata', tendon_act(i),...
						'edgecolor', [0.5 0.7 0.7], 'edgealpha',1, 'linewidth', 1.5,'linesmoothing','on');
end


% === contacts and contactforces ===
contacthandles		= nan(1,m.ncmax);
contactforcehandles	= nan(1,m.ncmax);
for i=1:m.ncmax
	contacthandles(i)	= patch( 'vertices', sph.vertices, ...
                              'faces', sph.faces, 'edgecolor', 'none', ...
                              'facecolor', [1 0.7 1], ...
                              'userdata',  sph.vertices, ...
                              'tag', 'contact', 'visible', 'off');
	contactforcehandles(i)	= patch('edgecolor', [1 1 1], 'edgealpha',0.3, 'linewidth', 1.5, ...
									'tag', 'force', 'linesmoothing','on', 'visible', 'off');						  
end
		
		
		
% === handles ===		
handles = struct(	'cursor',cursorhandle,...
					'ground',groundhandle,...
					'geoms',geomhandles,...
					'inertias',inertiahandles,...					
					'sites',sitehandles,...
					'tendons',tendonhandles,...
					'contacts',contacthandles,...
					'contactforces',contactforcehandles);

					
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% make ground plane
function groundhandle = makeGround(m,scale)

tile_size	= scale;		% length of the side of a tile
nt				= 7;		% number of tiles from the origin to the edge
nl				= 2*nt+1;	% number of lines in the grid
lines			= linspace(-nt*tile_size, nt*tile_size, nl);

% search for ground: horizontal plane in world body
planepos = [];
for i=1:m.ngeom
	if m.geom_type(i)==0 && m.geom_bodyid(i)==0 && m.geom_quat(i,1)>0.99,
		planepos = m.geom_pos(i,:);
		break;
	end
end

% draw plane if found
if ~isempty(planepos)
	checker		 = reshape( mod(1:nl*nl, 2), [nl nl]);
	Cdata			 = ones(nl,nl,3)*0.2;
	Cdata(:,:,1) = Cdata(:,:,1) + checker * 0.4;
	Cdata(:,:,2) = Cdata(:,:,2) + checker * 0.4;
	Cdata(:,:,3) = Cdata(:,:,3) + checker * 0.4;
	Xdata			 = lines + planepos(1);
	Ydata			 = lines + planepos(2);
	Zdata			 = ones(nl)*planepos(3);
	[f,v,c]		 = surf2patch(Xdata,Ydata,Zdata,Cdata,'triangles');
	groundhandle = patch('faces', f, 'vertices', v, ...
		'FaceVertexCData',c,'FaceColor','flat',... 
		'facealpha', 1, 'facelighting', 'flat', ...
		'backfacelighting', 'unlit', 'hittest','off',...
		'edgecolor', 'none','tag','ground');
	setappdata(groundhandle,'info', struct('type', 'plane', 'id', i, 'pos', planepos))
else
	groundhandle = [];
end					
					
					
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% make geom objects
function geoms = makeGeoms(model)

% sphere, ellipsoid and cylinder detail
NSphere = 15;

% initialize with NaNs, used to indicate planes
% (and world geoms in poses > 1)
geoms = nan(model.ngeom,1);

% get meshes if needed
if model.nmesh>0,
	mesh = mj('getmesh');
end

% construct geoms
for i=1:model.ngeom
	switch model.geom_type(i),
		case 0,                 % plane - skip
			F = [];
			V = [];
			
		case 1,                 % sphere
			[x,y,z]		= sphere(NSphere);
			[F, V]		= surf2patch(x,y,z);
			V			= V * model.geom_size(i,1);
			
		case 2,                 % capsule
			% make sphere
			[x,y,z]		= sphere(NSphere);
			[F, V]      = surf2patch(x,y,z);
			V           = V * model.geom_size(i,1);
			
			% stretch into capsule
			itop        = (V(:,3)>0);
			V(itop,3)	= V(itop,3) + model.geom_size(i,2);
			V(~itop,3)	= V(~itop,3) - model.geom_size(i,2);
			
		case 3,                 % ellipsoid
			% make sphere, scale axes independently
			[x,y,z]     = sphere(NSphere);
			x           = x * model.geom_size(i,1);
			y           = y * model.geom_size(i,2);
			z           = z * model.geom_size(i,3);
			[F, V]      = surf2patch(x,y,z);
			
		case 4,                 % cylinder... close faces ???
			[x,y,z]		= cylinder([model.geom_size(i,1) model.geom_size(i,1)], NSphere);
			z			= 2 * (z-0.5) * model.geom_size(i,2);
			[F, V]		= surf2patch(x,y,z);
			
		case 5,                 % box
			[F,V]		= makeBox(model.geom_size(i,:));			
		case 7,                % mesh
			id				= model.geom_meshid(i) + 1;
			F				= mesh.face(mesh.faceadr(id)+1 : mesh.faceadr(id)+mesh.facenum(id), :);
			vmin			= min(F(:));
			vmax			= max(F(:));
			V				= mesh.vert(vmin+1 : vmax+1, :);
			F				= F - vmin + 1;
	end
	
	if ~isempty(F)
		% make vertices unique to enable phong shading
		[F,V] = uniqueVertices(F,V);
		
		% make the patch
		geoms(i) = patch( 'vertices', V, 'faces', F, ...
						'facecolor', model.geom_rgba(i,1:3), 'facealpha',  model.geom_rgba(i,4), ...
						'userdata',V,'tag', 'geom','BackFaceLighting','reverselit',...
						'edgecolor', 'none', 'visible', 'on', 'facelighting', 'phong');
		setappdata(geoms(i),'parent', model.geom_bodyid(i))
		
	end
	
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% make equivalent-inertia objects
function inertias = makeInertias(model)

inertias	= nan(model.nbody,1);

quat2mat = @(a,b,c,d) [a -d c]'*[a d -c] + [b c d]'*[b c d] + [-c b a]'*[c -b a] + [d a -b]'*[-d a b];

for i = 2:model.nbody
		inert	= model.body_inertia(i,:);
		sides	= sqrt(sum([-inert; inert([2 3 1]); inert([3 1 2])])/model.body_mass(i));
		[F,V]	= makeBox(sides);
		
		% local rotation and translation
		q		= model.body_iquat(i,:);
		R		= quat2mat(q(1),q(2),q(3),q(4));
		V		= V*R + model.body_ipos(i*ones(1,8),:);
		
		icolor	= [0.9 0.5 0.1];
		
		% make the patch
		inertias(i) = patch( 'vertices', V, 'faces', F, ...
						'facecolor', icolor, ...
						'userdata',V,'tag', 'inertia',...
						'edgecolor', icolor, 'visible', 'off', 'facelighting', 'flat');		
end


function [F,V] = makeBox(sides)

V	= [...
		 1,  1,  1; ...
		-1,  1,  1; ...
		-1, -1,  1; ...
		 1, -1,  1; ...
		 1,  1, -1; ...
		-1,  1, -1; ...
		-1, -1, -1; ...
		 1, -1, -1 ] * diag(sides);
F	= [ ...
		1 2 3 4; ...
		5 8 7 6; ...
		1 4 8 5; ...
		3 2 6 7; ...
		4 3 7 8; ...
		2 1 5 6 ];


%%%%%%%%%%%%%%%
% draw the rope
function ropeUpdate(fig, newrope)

% get primary axes
ax			= getappdata(fig,'main_ax');

% === anchor point ===
sg = getappdata(fig,'selected_geom');
if sg < 1 % no active selection
	sg = select_geom(fig, 'uparrow');
end
geom_xpos = mj('get','geom_xpos');
anchor    = geom_xpos(sg,:);	

% === cursor point ===	

% get camera position and orientation
cam		= get(ax, 'userdata');

% get camera position and orientation
az			= cam.azimuth/180*pi;
el			= cam.elevation/180*pi;
vec			= [cos(el)*cos(az), cos(el)*sin(az), sin(el)];
camdir		= -vec;
campos		= cam.lookat + cam.distance*vec;

% get line through cursor (and camera)
ltc		= [1 -1]*get(ax, 'currentpoint');
ltc		= ltc / norm(ltc);

% orient ltc with the camera
ltc	= ltc * sign(ltc*camdir');

% get current rope data
rope	= getappdata(fig,'rope');

if newrope || isempty(rope)	% new cursor distance and rope length

	% intersect line through cursor with plane through anchor
	t  =  ((anchor-campos)*camdir') / (ltc*camdir');

	% get 3D location of cursor
	cursorpos = campos + t*ltc;

	% get distance of new interaction plane from camera
	dist	= (cursorpos - campos) * camdir';
	
	% new rope settings
	rope.dist	= dist;
	rope.length = 1.1*norm(cursorpos - anchor);
	rope.sag		= 1;
else									% use current rope distance	

	% find point on line-through-cursor at cursordist	
	cursorpos	= campos + ltc*rope.dist;
end

fromto = cursorpos - anchor;

% --- get rope shape
% project onto vertical plane
pT = fromto(1:2);		% tangent to plane
pT = pT / norm(pT);
pN = [-pT(2) pT(1)];	% normal to plane
xy = [anchor(1:2)' cursorpos(1:2)'];
a  = pT*xy;				% project onto tangent
b  = pN*xy;				% project onto normal

% start and end points
p1	= [a(1) anchor(3)];
p2	= [a(2) cursorpos(3)];

% number of intermediate points
N = 20;

% get rope shape in vertical plane
[A,Z,sag] = catenary(p1, p2, rope.length, N, rope.sag);

% project back into 3D space
B		= mean(b)*ones(1,N);
ROPE	= [[pT'*A + pN'*B; Z] nan(3,1)];

% --- update cursor and rope structure
% get cursor handle
handles     = getappdata(fig, 'handles');
cursor      = handles.cursor;	

% update cursor object
set(cursor, 'xdata', ROPE(1,:),...
			'ydata', ROPE(2,:),...
			'zdata', ROPE(3,:));
			
% update rope structure
rope.geom	= sg;
rope.cursor = cursorpos;
rope.sag	= sag;
setappdata(fig,'rope',rope)


function [X,Y,sag] = catenary(a,b,r_length,N,sagInit)
% given two points a=[ax ay] and b=[bx by] in the vertical plane,
% rope length r_length, and the number of intermediate points N,
% outputs the coordinates X and Y of the hanging rope from a to b
% the optional input sagInit initializes the sag parameter for the
% root-finding procedure.

maxIter		= 10;		 % maximum number of iterations
minGrad		= 1e-10;     % minimum norm of gradient
minVal		= 1e-8;      % minimum norm of sag function
stepDec		= 0.5;       % factor for decreasing stepsize
minStep		= 1e-6;		 % minimum step size
minHoriz	= 1e-3;		 % minumum horizontal distance

if nargin < 5
	sag = 1;
else
	sag = sagInit;
end

if a(1) > b(1)
	[a,b]	= deal(b,a);
end

d = b(1)-a(1);
h = b(2)-a(2);


if abs(d) < minHoriz % almost perfectly vertical
	X = ones(1,N)*(a(1)+b(1))/2;
	if r_length < abs(h) % rope is stretched
		Y		= linspace(a(2),b(2),N);
	else
		sag	= (r_length-abs(h))/2;
		n_sag = ceil( N * sag/r_length );
		y_max = max(a(2),b(2));
		y_min = min(a(2),b(2));
		Y		= linspace(y_max,y_min-sag,N-n_sag);
		Y		= [Y linspace(y_min-sag,y_min,n_sag)];
	end
	return;
end

X = linspace(a(1),b(1),N);

if r_length <= sqrt(d^2+h^2) % rope is stretched: straight line
	Y = linspace(a(2),b(2),N);
	
else
	% find rope sag
	g  = @(s) 2*sinh(s*d/2)/s - sqrt(r_length^2-h^2);
	dg = @(s) 2*cosh(s*d/2)*d/(2*s) - 2*sinh(s*d/2)/(s^2);

	for iter = 1:maxIter
		val		= g(sag); 
		grad		= dg(sag);
		if abs(val) < minVal || abs(grad) < minGrad
			break
		end
		search	= -g(sag)/dg(sag);
		
		alpha		= 1;
		sag_new  = sag + alpha*search;
		
		while sag_new < 0 || abs(g(sag_new)) > abs(val)
			alpha		= stepDec*alpha;
			if alpha < minStep
				break;
			end
			sag_new	= sag + alpha*search;			
		end
		
		sag = sag_new;
	end

	% get location of rope minimum and vertical bias
	x_left	= 1/2*(log((r_length+h)/(r_length-h))/sag-d);
	x_min		= a(1) - x_left;
	bias		= a(2) - cosh(x_left*sag)/sag;

	Y			= cosh((X-x_min)*sag)/sag + bias;
end


%%%%%%%%%%%%%%%%%%%
% utility functions

function [f,v] = uniqueVertices(f,v)

[v,~,n]  = unique(v,'rows');
f        = n(f);


function str = help_text()

str	= ' Type ''help mjplot'' at the command prompt for documentation.';


function data = getData()

% these are the fields of data we'll save
data_fields = {...
'ne'
'ncon'
'nflc'
'time'
'com'
'energy'
'qpos'
'qvel'
'qvel_next'
'act'
'act_dot'
'ctrl'
'qfrc_bias'
'qfrc_external'
'qfrc_impulse'
'xpos'
'xmat'
'geom_xpos'
'geom_xmat'
'site_xpos'
'site_xmat'
'ten_wrapadr'
'ten_wrapnum'
'ten_length'
'wrap_obj'
'wrap_xpos'
'contact'
'con_id'}';

% get the field values
V		= cell(1,length(data_fields));
[V{:}]	= mj('get',data_fields{:});

% create a matlab struct
FV		= [data_fields; V];
data	= struct(FV{:});