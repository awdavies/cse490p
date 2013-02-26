% Defines joint indices for specific DOFs
%
% Will need to be changed if the model is changed.
% the ending bit is for the type of joint movement.  XZ
% would imply rotation along the XZ plane.  X would imply
% a point on the X line.
%
% There is also a special range param for specifying the
% torso DOF range.  These should all be contiguous in the
% xml definition to make changes minimal.
classdef joints
	properties (Constant)
		TORSO_DOF_RANGE = 1:3;
		TORSO_X = 1;
		TORSO_Y = 2;
		TORSO_XZ = 3;
		RIGHT_THIGH_XZ = 4;
		RIGHT_SHIN_XZ = 5;
		RIGHT_FOOT_XZ = 6;
		LEFT_THIGH_XZ = 7;
		LEFT_SHIN_XZ = 8;
		LEFT_FOOT_XZ = 9;
	end

end
