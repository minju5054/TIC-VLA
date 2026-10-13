"""Only the user's exact instruction is changed; original scene checks remain."""
import copy

OLD_INSTRUCTION = ('Move forward toward the staircase, then turn left to enter the hallway. '
                   'Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.')
INSTRUCTION = OLD_INSTRUCTION + ' If an obstacle blocks your path, navigate around it.'


def scene_config(cfg, reference):
    """Project only task text for unchanged official-scene validation, not inference."""
    if cfg.get('instruction') != INSTRUCTION or reference.get('instruction') != OLD_INSTRUCTION:
        raise ValueError('Exact authorized old/new instructions required')
    projected = copy.deepcopy(cfg)
    projected['instruction'] = OLD_INSTRUCTION
    if projected != reference:
        raise ValueError('Only instruction may differ from the previous wheelchair scenario')
    return projected
