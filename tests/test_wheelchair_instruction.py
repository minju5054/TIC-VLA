import copy
import unittest
from tools.wheelchair_instruction.policy import scene_config, OLD_INSTRUCTION, INSTRUCTION


class InstructionOnly(unittest.TestCase):
    def pair(self):
        old={'instruction':OLD_INSTRUCTION,'scene':{'goal':[1,2,0]},'controller':{'max_v':1.5},'fixed_humans':[{'id':'C10'}],'fixed_wheelchair':{'position':[-1.7567275762557983,10.988419532775879,0]}}
        new=copy.deepcopy(old);new['instruction']=INSTRUCTION
        return old,new

    def test_only_exact_instruction_allowed_without_mutation(self):
        old,new=self.pair();before=copy.deepcopy(new)
        self.assertEqual(scene_config(new,old),old)
        self.assertEqual(new,before)
        self.assertEqual(INSTRUCTION,OLD_INSTRUCTION+' If an obstacle blocks your path, navigate around it.')

    def test_scene_controller_actor_changes_rejected(self):
        for key in ('scene','controller','fixed_humans','fixed_wheelchair'):
            old,new=self.pair();new[key]={}
            with self.assertRaisesRegex(ValueError,'Only instruction'):scene_config(new,old)

    def test_other_instruction_rejected(self):
        old,new=self.pair();new['instruction']+=' Please hurry.'
        with self.assertRaisesRegex(ValueError,'Exact authorized'):scene_config(new,old)
