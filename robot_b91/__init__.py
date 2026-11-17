"""B10 CAD-derived simulation and walking controller."""


def servo_properties(config, joint):
    """Return output torque, speed and servo mass for the configured joint."""
    servo = config["servos"][config["joint_servos"][joint]]
    return (
        servo["stall_Nm"],
        servo["free_speed_deg_s"],
        servo["mass_kg"],
    )
