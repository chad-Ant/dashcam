"""Exposure timing contract. Host arrival times are never exposure timestamps."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ExposurePair:
    trigger_left: int
    trigger_right: int
    clock_domain: str
    left_mid_us: float
    right_mid_us: float
    uncertainty_each_us: float
    exposure_us: float
    global_shutter: bool
    hardware_sync_verified: bool

    def effective_interval_s(self,max_skew_us=100.):
        if self.global_shutter is not True or self.hardware_sync_verified is not True:
            raise ValueError('Moving scenes need verified hardware-synchronized global shutters')
        if self.clock_domain not in ('shared_hardware_clock','validated_common_trigger'):
            raise ValueError('Independent ESP/host arrival clocks cannot prove exposure synchronization')
        if type(self.trigger_left) is not int or type(self.trigger_right) is not int or self.trigger_left<0 or self.trigger_left!=self.trigger_right:
            raise ValueError('Missing/mismatched hardware trigger sequence IDs')
        values=(self.left_mid_us,self.right_mid_us,self.uncertainty_each_us,self.exposure_us,max_skew_us)
        if not all(math.isfinite(v) and v>=0 for v in values) or self.exposure_us==0 or max_skew_us<=0:
            raise ValueError('Invalid exposure timing bounds')
        skew=abs(self.left_mid_us-self.right_mid_us)+2*self.uncertainty_each_us
        if skew>max_skew_us:
            raise ValueError(f'Exposure skew bound {skew:g} us exceeds {max_skew_us:g} us')
        # Conservative extra time term for blur, not a full motion/deblurring model.
        return (skew+self.exposure_us)*1e-6
