from dex_retargeting.retargeting_config import RetargetingConfig
from dex_retargeting.seq_retarget import SeqRetargeting
from .vectorada_optimizer import VectorAdaOptimizer


def build_retarget_tool(retarget_config: dict, hand_urdf_dir: str):
    """
    Construct the retargeting tool based on the configuration.
    Supports different types: vector, position, dexpilot, vectorada.
    """
    retarget_type = retarget_config["retargeting"].get("type", "")
    if not retarget_type:
        raise ValueError("Missing retargeting type in configuration.")

    RetargetingConfig.set_default_urdf_dir(hand_urdf_dir)

    if retarget_type in {"vector", "position", "dexpilot"}:
        retargeting_config = RetargetingConfig.from_dict(retarget_config["retargeting"])
        return retargeting_config.build()

    elif retarget_type == "vectorada":
        # Temporarily set type to "vector" for base class parsing
        retarget_config["retargeting"]["type"] = "vector"
        base_config = RetargetingConfig.from_dict(retarget_config["retargeting"])
        retarget_config["retargeting"]["type"] = "vectorada"  # Restore original

        base_tool = base_config.build()
        base_optimizer = base_tool.optimizer

        # Construct VectorAda optimizer
        vector_ada_optimizer = VectorAdaOptimizer(
            robot=base_optimizer.robot,
            target_joint_names=base_optimizer.target_joint_names,
            target_origin_link_names=base_optimizer.origin_link_names,
            target_task_link_names=base_optimizer.task_link_names,
            target_link_human_indices=base_optimizer.target_link_human_indices,
            scaling=base_config.scaling_factor,
            norm_delta=base_config.normal_delta,
            huber_delta=base_config.huber_delta,
            num_constrain=retarget_config.get("num_constrain", 4),
        )

        # Wrap in sequential retargeting container
        return SeqRetargeting(
            vector_ada_optimizer,
            has_joint_limits=base_tool.has_joint_limits,
            lp_filter=base_tool.filter,
        )

    else:
        raise ValueError(f"Unsupported retargeting type: '{retarget_type}'")
