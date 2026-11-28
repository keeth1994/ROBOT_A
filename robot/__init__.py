"""Robot simulation and control."""


def servo_properties(config, joint):
    """Return servo torque, speed and mass."""
    servo = config["servos"][config["joint_servos"][joint]]
    return (
        servo["stall_Nm"],
        servo["free_speed_deg_s"],
        servo["mass_kg"],
    )
