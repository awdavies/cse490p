function vcom = get_vcom(model, m, J)


% Find velocity of the center of mass.  This will grab all of
% the x vectors and add them together.
xvel = zeros(3, m.nbody);
for i = 1:m.nbody
  xvel(:, i) = J(:,:, i) * model.v * m.body_mass(i);
end
vcom = sum(xvel(1, :)) / sum(m.body_mass(:));
