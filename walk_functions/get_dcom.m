function [ dcom_l, dcom_r ] = get_dcom(model)

right_f = model.x(joints.RIGHT_FOOT_XZ);
left_f = model.x(joints.LEFT_FOOT_XZ);
dcom_l = left_f - model.com(1);
dcom_r = right_f - model.com(1);
