% Proportional-derivative control

function u = pd_controller(q, v, k, b, qplan, vplan)
    u = k * (qplan - q) + b * (vplan - v);
end