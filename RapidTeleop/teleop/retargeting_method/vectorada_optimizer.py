import numpy as np
import torch
from typing import List
from dex_retargeting.optimizer import Optimizer
from dex_retargeting.robot_wrapper import RobotWrapper


class VectorAdaOptimizer(Optimizer):
    retargeting_type = "VECTORADA"

    def __init__(
        self,
        robot: RobotWrapper,
        target_joint_names: List[str],
        target_origin_link_names: List[str],
        target_task_link_names: List[str],
        target_link_human_indices: np.ndarray,
        huber_delta: float = 0.02,
        norm_delta: float = 4e-3,
        scaling: float = 1.0,
        num_constrain: int = 4,
    ):
        super().__init__(robot, target_joint_names, target_link_human_indices)

        self.origin_link_names = target_origin_link_names
        self.task_link_names = target_task_link_names

        self.huber_loss = torch.nn.SmoothL1Loss(beta=huber_delta, reduction="mean")
        self.norm_delta = norm_delta
        self.scaling = scaling
        self.num_constrain = num_constrain

        # Unique list of all links involved in optimization
        self.computed_link_names = list(set(self.origin_link_names + self.task_link_names))
        self.origin_link_indices = torch.tensor(
            [self.computed_link_names.index(name) for name in self.origin_link_names]
        )
        self.task_link_indices = torch.tensor(
            [self.computed_link_names.index(name) for name in self.task_link_names]
        )
        self.computed_link_indices = self.get_link_indices(self.computed_link_names)

        self.opt.set_ftol_abs(1e-6)

        # Norm thresholds (empirical)
        self.norm_min = np.array([0.0015, 0.0072, 0.0166, 0.0123])[:num_constrain]
        self.norm_max = np.array([0.1621, 0.1963, 0.2094, 0.2239])[:num_constrain]

    def get_objective_function(self, target_vector: np.ndarray, fixed_qpos: np.ndarray, last_qpos: np.ndarray):
        """
        Build the NLopt-compatible objective function.
        """
        qpos = np.zeros(self.num_joints)
        qpos[self.idx_pin2fixed] = fixed_qpos

        # Compute adaptive weights for final touch joints
        touch_weights = np.ones(len(target_vector)) * 1.2
        touch_weights[-self.num_constrain:] = self._compute_touch_degree(target_vector[-self.num_constrain:])
        touch_weights = torch.as_tensor(touch_weights, dtype=torch.float32)
        touch_weights.requires_grad_(False)

        target_vector_tensor = torch.as_tensor(target_vector * self.scaling, dtype=torch.float32)
        target_vector_tensor.requires_grad_(False)

        def objective(x: np.ndarray, grad: np.ndarray) -> float:
            qpos[self.idx_pin2target] = x

            # Apply optional forward kinematics adaptor
            if self.adaptor is not None:
                qpos = self.adaptor.forward_qpos(qpos)

            self.robot.compute_forward_kinematics(qpos)
            target_poses = [self.robot.get_link_pose(idx) for idx in self.computed_link_indices]
            body_positions = np.array([pose[:3, 3] for pose in target_poses])

            torch_body_pos = torch.tensor(body_positions, requires_grad=True)

            origin_pos = torch_body_pos[self.origin_link_indices]
            task_pos = torch_body_pos[self.task_link_indices]
            robot_vectors = task_pos - origin_pos

            # Apply weighting to error
            vector_diff = robot_vectors - target_vector_tensor
            error_norm = torch.norm(vector_diff, dim=1, keepdim=False)
            weighted_error = error_norm * touch_weights
            loss = self.huber_loss(weighted_error, torch.zeros_like(weighted_error))

            result = loss.cpu().detach().item()

            if grad.size > 0:
                jacobians = []
                for idx, link_index in enumerate(self.computed_link_indices):
                    local_jac = self.robot.compute_single_link_local_jacobian(qpos, link_index)[:3]
                    rot = target_poses[idx][:3, :3]
                    jac_world = rot @ local_jac
                    jacobians.append(jac_world)

                jacobians = np.stack(jacobians, axis=0)
                loss.backward()

                grad_pos = torch_body_pos.grad.cpu().numpy()[:, None, :]  # (N, 1, 3)

                if self.adaptor is not None:
                    jacobians = self.adaptor.backward_jacobian(jacobians)
                else:
                    jacobians = jacobians[..., self.idx_pin2target]

                grad_qpos = np.matmul(grad_pos, jacobians).mean(1).sum(0)
                grad_qpos += 2 * self.norm_delta * (x - last_qpos)
                grad[:] = grad_qpos

            return result

        return objective

    def _compute_touch_degree(self, adapt_vectors: np.ndarray) -> np.ndarray:
        """
        Compute per-vector touch weight using norm scaling and sigmoid shaping.
        """
        norms = np.linalg.norm(adapt_vectors, axis=1)

        # Update normalization bounds
        for i in range(self.num_constrain):
            self.norm_min[i] = min(self.norm_min[i], norms[i])
            self.norm_max[i] = max(self.norm_max[i], norms[i])

        normalized = 1.0 - (norms - self.norm_min) / (self.norm_max - self.norm_min)
        return fine_tune_touch_degree(normalized)


def fine_tune_touch_degree(norms: np.ndarray, k=10.0, c=0.8, A=1.8) -> np.ndarray:
    """
    Sigmoid-like function to enhance low-norm penalties and smooth sharp transitions.
    """
    return A / (1.0 + np.exp(-k * (norms - c)))
