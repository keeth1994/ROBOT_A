"""B9-1 CAD-derived simulation and walking controller."""


def servo_properties(config, joint):
    """Return output torque, speed and servo mass for the configured joint."""
    servo = config["servos"][config["joint_servos"][joint]]
    yaw = joint.endswith("_yaw")
    ratio = config["gear_ratio_speed"] if yaw else 1.0
    efficiency = config["gear_efficiency_assumed"] if yaw else 1.0
    return (
        servo["stall_Nm"] * efficiency / ratio,
        servo["free_speed_deg_s"] * ratio,
        servo["mass_kg"],
    )
