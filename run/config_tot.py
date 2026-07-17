"""Total-energy simulation configuration derived from config_dirfix."""

from copy import deepcopy

from config_dirfix import params as directional_params
from mypkg.constants import CbfType


params = deepcopy(directional_params)
params.CBF_type = CbfType.TOTAL
params.log_filename = "output/simulation_total.csv"
