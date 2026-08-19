function [pqr_o, alpha_o, beta_o, euler_o] = uav_blend_step(pqr_h, alpha_h, beta_h, euler_h, ...
                                                            pqr_t, alpha_t, beta_t, euler_t, BL)
%UAV_BLEND_STEP  Per-channel switch between estimated and true feedback.
%
%   BL = [pqr; alpha; beta; euler], 1 = feed the controller the ESTIMATE,
%   0 = feed it plant TRUTH. Keeping the blend separate leaves the estimator
%   HDL-targetable and permits per-channel feedback selection.

%#codegen
pqr_o   = BL(1)*pqr_h(:)   + (1-BL(1))*pqr_t(:);
alpha_o = BL(2)*alpha_h    + (1-BL(2))*alpha_t;
beta_o  = BL(3)*beta_h     + (1-BL(3))*beta_t;
euler_o = BL(4)*euler_h(:) + (1-BL(4))*euler_t(:);
end
