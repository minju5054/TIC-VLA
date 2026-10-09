"""Bright profile applied to actual sensor RGB, retaining Nova execution semantics."""
import json
import numpy as np
from research.nova_simulation import NovaSimulation
from research.nova_bright import apply_profile,image_stats,bright_pass
from research.records import write_json


class BrightSimulation(NovaSimulation):
    def __init__(self,cfg,records,actor_factory=None):
        super().__init__(cfg,records,actor_factory=actor_factory)
        actual=apply_profile(cfg['bright_profile'],self.stage)
        write_json(records.path/'bright_profile_applied.json',{'profile':cfg['bright_profile'],'renderer_readback':actual,
            'note':'Actual model RGB uses this fixed profile; no following/request-dependent light'})
        self.segmentation=None
        if self.pedestrian:
            import omni.replicator.core as rep
            self.segmentation=rep.AnnotatorRegistry.get_annotator('semantic_segmentation',init_params={'colorize':False})
            self.segmentation.attach([self.product])
        (records.path/'raw/brightness').mkdir();(records.path/'raw/visibility').mkdir()

    def capture(self,name):
        data,obs,path=super().capture(name)
        s=image_stats(data);write_json(self.records.path/'raw/brightness'/(name+'.json'),{
            **s,'pass':bright_pass(s,self.cfg['bright_validation']),'observation_sim_time':obs['sim_time'],
            'rgb_reference':str(path.relative_to(self.records.path))})
        if self.segmentation is not None:
            from PIL import Image
            from research.nova_occlusion import segmentation_stats,visibility_state
            packed=self.segmentation.get_data();stats,mask=segmentation_stats(packed['data'],packed['info']['idToLabels'])
            write_json(self.records.path/'raw/visibility'/(name+'.json'),{**stats,
                'state':visibility_state(stats,self.cfg['occlusion']['visibility_rule']),'observation_sim_time':obs['sim_time'],
                'labels':packed['info']['idToLabels'],'model_input':False})
            Image.fromarray((mask*255).astype('uint8')).save(self.records.path/'raw/visibility'/(name+'.mask.png'))
        return data,obs,path
