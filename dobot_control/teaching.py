"""A fresh set of taught positions is required for each calibration session."""
from dataclasses import replace


class Teaching:
    def __init__(self, config):
        self.config = replace(config, calibrated=False)
        self.captures = {}

    def capture(self, key, pose):
        if key not in ('1','2','t','p','l'):
            raise ValueError('Unknown capture key')
        self.captures[key] = pose

    def swap_axes(self):
        c = self.config
        self.config = replace(c, right_axis=c.forward_axis, forward_axis=c.right_axis,
                              right_sign=c.forward_sign, forward_sign=c.right_sign)

    def reverse(self, direction):
        key = direction + '_sign'
        self.config = replace(self.config, **{key:-getattr(self.config,key)})

    def finish(self, workspace_confirmed):
        if set(self.captures) != {'1','2','t','p','l'}:
            raise ValueError('Capture corners 1/2 and heights T/P/L first')
        if not workspace_confirmed:
            raise ValueError('Confirm the entire rectangle is reachable and obstacle-free with W')
        a,b = self.captures['1'],self.captures['2']
        return replace(self.config, calibrated=True,
                       x_min=min(a.x,b.x), x_max=max(a.x,b.x),
                       y_min=min(a.y,b.y), y_max=max(a.y,b.y),
                       travel_z=self.captures['t'].z,
                       pick_z=self.captures['p'].z,
                       place_z=self.captures['l'].z).validate()
