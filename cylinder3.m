 function Cylinder = cylinder3(X1,X2,r,n,cyl_color,lines,alpha, h)


% Calculating the length of the cylinder
length_cyl=norm(X2-X1);

% Creating a circle in the YZ plane
t=linspace(0,2*pi,n)';
x2=r*cos(t);
x3=r*sin(t);

% Creating the points in the X-Direction
x1=[0 length_cyl];

% Creating (Extruding) the cylinder points in the X-Directions
xx1=repmat(x1,length(x2),1);
xx2=repmat(x2,1,2);
xx3=repmat(x3,1,2);

if(nargin == 7)
	Cylinder=mesh(xx1,xx2,xx3);

	% Defining Unit vector along the X-direction
	unit_Vx=[1 0 0];
	
	% Calulating the angle between the x direction and the required direction
	% of cylinder through dot product
	angle_X1X2=acos( dot( unit_Vx,(X2-X1) )/( norm(unit_Vx)*norm(X2-X1)) )*180/pi;
	
	% Finding the axis of rotation (single rotation) to roate the cylinder in
	% X-direction to the required arbitrary direction through cross product
	axis_rot=cross([1 0 0],(X2-X1) );
	
	% Rotating the plotted cylinder and the end plate circles to the required
	% angles
	if angle_X1X2~=0 % Rotation is not needed if required direction is along X
		rotate(Cylinder,axis_rot,angle_X1X2,[0 0 0])
	end
	
	set(Cylinder,'XData',get(Cylinder,'XData')+X1(1))
	set(Cylinder,'YData',get(Cylinder,'YData')+X1(2))
	set(Cylinder,'ZData',get(Cylinder,'ZData')+X1(3))

else
	Cylinder = h;
	set(Cylinder,'XData', xx1+X1(1));
	set(Cylinder,'YData', xx2+X1(2));
	set(Cylinder,'ZData', xx3+X1(3));
	
	unit_Vx=[1 0 0];
	angle_X1X2=acos( dot( unit_Vx,(X2-X1) )/( norm(unit_Vx)*norm(X2-X1)) )*180/pi;
	axis_rot=cross([1 0 0],(X2-X1) );
	if angle_X1X2~=0 % Rotation is not needed if required direction is along X
		rotate(Cylinder,axis_rot,angle_X1X2,X1)
	end
	
end


% Setting the color to the cylinder and the end plates
set(Cylinder,'FaceColor',cyl_color)

% If lines are not needed making it disapear
if lines==0
    set(Cylinder,'EdgeAlpha',0)
	set(Cylinder,'EdgeColor','k')
end

set(Cylinder, 'FaceAlpha',alpha);