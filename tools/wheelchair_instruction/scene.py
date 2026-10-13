"""Instruction-only adapter around the frozen mixed-object simulation."""
from pathlib import Path
from tools.c24_wheelchair.scene import MixedSimulation, read
from tools.wheelchair_instruction.policy import scene_config, INSTRUCTION
from research.records import write_json

PREVIOUS = Path('outputs/c10-human-c24-wheelchair-20261013T150824Z-fc819a')


class InstructionSimulation(MixedSimulation):
    def __init__(self, cfg, records, actor_factory=None):
        root = Path(__file__).resolve().parents[2]
        reference = read(root/PREVIOUS/'metadata.json')['config']
        geometry = scene_config(cfg, reference)
        # Scene creation reads the original instruction solely to retain its
        # unchanged episode guard. No inference occurs in the constructor.
        super().__init__(geometry, records, actor_factory=actor_factory)
        self.cfg = cfg
        if self.cfg['instruction'] != INSTRUCTION:
            raise ValueError('Instruction override was not retained for inference')
        write_json(records.path/'instruction-intervention.json', {
            'previous_run': str(PREVIOUS), 'previous_instruction': reference['instruction'],
            'model_instruction': cfg['instruction'], 'changed_config_keys': ['instruction'],
            'non_instruction_config_equal': True,
            'scene_validation_note': 'Original instruction used only by unchanged official scene-geometry guard; actual new instruction retained in metadata, sim.cfg, model config and every prediction request.',
            'model_calls_at_this_point': 0})
