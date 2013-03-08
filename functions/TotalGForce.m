%{
Calculates the total gravitational force on the bodies
with given indices in the model.
%}
function f = TotalGForce( model, body_indices )

    gravity = [0; 0; -9.81];

    % Sum up the gravitational force on each of
    % the bodies that are specified in body_indices.
    f = 0;
    for i = body_indices
        mass_i = model.body_mass(i);
        J_i = mj('jacbody', i - 1);
        
        f = f + (J_i.')*(mass_i * gravity);
    end

end

