"""Bounded stationary rear support test at the first crest.

No forward strokes or steering commands. Manual control first enables at neutral
and slowly restores the crest stance. Front feet then hold that stance while
each rear foot lifts, flares, and replants separately. Then each rear
foot lifts and replants once at that placement. Three feet stay commanded down.
Uses the existing calibrated manual API; no firmware upload is required.
Run --dry-run to inspect the complete plan without accessing the robot.
"""
from crest_recovery import main


def plan():
    # Physical servo degrees, FL/RL/FR/RR yaw,pitch. Rear yaw mounting is
    # reversed: increasing physical RL yaw and decreasing RR yaw flare them.
    pose = [14, 45, 14, 45, -14, 45, -14, 45]
    # The installed manual API refuses a non-neutral first command on every
    # disabled channel. Enable at its measured rest angle, then interpolate.
    stages = [('enable measured neutral stance', 1., [0, 45, 0, 45, 0, 45, 0, 45]),
              ('restore crest stance gradually', 2., pose.copy()),
              ('hold crest stance', .8, pose.copy())]
    for leg, yaw in ((1, 23), (3, -23)):
        pose[2*leg+1] = 31
        stages.append((f'rear {leg}: lift before flare', .8, pose.copy()))
        pose[2*leg] = yaw
        stages.append((f'rear {leg}: flare outward', 1., pose.copy()))
        pose[2*leg+1] = 45
        stages.append((f'rear {leg}: replant and load', 1., pose.copy()))
        stages.append((f'rear {leg}: hold support', .5, pose.copy()))
    for leg in (1, 3):
        pose[2*leg+1] = 31
        stages.append((f'rear {leg}: lift at flared placement', .8, pose.copy()))
        pose[2*leg+1] = 45
        stages.append((f'rear {leg}: press back to support', 1., pose.copy()))
    stages.append(('hold all feet down', 1., pose.copy()))
    return stages


if __name__ == '__main__':
    main(plan_factory=plan, max_seconds=22, description=__doc__,
         completion_message='Completed stationary rear support probe; inspect chassis clearance before any walking')
