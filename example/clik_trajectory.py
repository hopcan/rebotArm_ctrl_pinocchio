import pinocchio as pin
import numpy as np
from pathlib import Path
import sys
import time

project_root = Path(__file__).resolve().parent.parent
sys.path.append(str(project_root))
rebotArm_DM_model_path = project_root / "urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf"

model = pin.buildModelFromUrdf(rebotArm_DM_model_path)
data = model.createData()


def interp_SE3(oM0: pin.SE3, oM1: pin.SE3, s: float) -> pin.SE3:
    """Interpolate between two SE3 poses using SE3 logarithm/exponential.

    s in [0,1]
    """
    if s <= 0.0:
        return oM0
    if s >= 1.0:
        return oM1
    Trel = oM0.actInv(oM1)
    xi = pin.log(Trel)
    return oM0 * pin.exp(xi * s)


def cartesian_linear_trajectory(oM0: pin.SE3, oM1: pin.SE3, T: float, dt: float):
    """Generate N poses from oM0 to oM1 over time T with timestep dt."""
    if T <= 0:
        return [oM1]
    N = max(2, int(np.ceil(T / dt)) + 1)
    traj = [interp_SE3(oM0, oM1, float(i) / (N - 1)) for i in range(N)]
    return traj


def clik_follow(traj, q0, model, data, frame_name="end_link", damp=1e-6, dt=1e-2):
    """Follow a Cartesian trajectory using Closed-Loop IK (CLIK).

    Returns an array of joint configurations (N x nq).
    """
    q = np.array(q0, dtype=float)
    q_hist = []
    frame_id = model.getFrameId(frame_name)
    for oMdes in traj:
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        iMd = data.oMf[frame_id].actInv(oMdes)
        err = pin.log(iMd).vector

        if np.linalg.norm(err) > 1e-12:
            J = pin.computeJointJacobian(model, data, q, 6)
            # Apply left-trivialized Jacobian correction for SE3 error
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)
            # CLiK velocity command (damped least squares in task-space)
            v = -J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err))
            q = pin.integrate(model, q, v * dt)

        q_hist.append(q.copy())
    return np.array(q_hist)


def sample_run():
    # initial joint configuration (6 DOF)
    q0 = np.zeros(model.nq)

    # initial and final end-effector poses
    frame_id = model.getFrameId("end_link")
    pin.forwardKinematics(model, data, q0)
    pin.updateFramePlacements(model, data)
    oM0 = data.oMf[frame_id]

    # target pose: translate 0.2m along x and lower slightly
    target_pos = oM0.translation + np.array([0.2, 0.0, -0.05])
    # keep current orientation
    oM1 = pin.SE3(oM0.rotation, target_pos)

    # generate Cartesian straight-line trajectory over 3 seconds
    traj = cartesian_linear_trajectory(oM0, oM1, T=3.0, dt=0.02)

    # run CLiK follower
    q_traj = clik_follow(traj, q0, model, data, damp=1e-6, dt=0.02)

    print("Generated trajectory points:", q_traj.shape[0])
    print("Final joint config:", q_traj[-1])
    # save to file
    np.save(project_root / "clik_q_traj.npy", q_traj)


if __name__ == "__main__":
    sample_run()
