import numpy as np


def allocate(F_total: float, tau_x: float, tau_y: float, tau_z: float,
             k_F: float, k_T: float, arm_length: float) -> np.ndarray:
    """Convert [F, τx, τy, τz] to [RPM0, RPM1, RPM2, RPM3].

    Motor layout (X-config, top-down view, body x=forward y=left):
      0: front-right  (CW)
      1: rear-left    (CW)
      2: front-left   (CCW)
      3: rear-right   (CCW)
    """
    a = arm_length * np.sqrt(2) / 2.0

    # Inverse allocation: [ω²] = M_inv × [F, τx, τy, τz]ᵀ
    # Derived from exact 4×4 matrix inversion.
    inv_kF = 1.0 / k_F
    inv_kFa = 1.0 / (k_F * a)
    inv_kT = 1.0 / k_T

    w_sq = np.zeros(4)
    w_sq[0] = 0.25 * (F_total * inv_kF - tau_x * inv_kFa - tau_y * inv_kFa - tau_z * inv_kT)
    w_sq[1] = 0.25 * (F_total * inv_kF + tau_x * inv_kFa + tau_y * inv_kFa - tau_z * inv_kT)
    w_sq[2] = 0.25 * (F_total * inv_kF + tau_x * inv_kFa - tau_y * inv_kFa + tau_z * inv_kT)
    w_sq[3] = 0.25 * (F_total * inv_kF - tau_x * inv_kFa + tau_y * inv_kFa + tau_z * inv_kT)

    # Clamp negative squares (shouldn't happen with valid inputs, but guard)
    w_sq = np.maximum(w_sq, 0.0)

    return np.sqrt(w_sq)