"""Operational-energy ROS 1 experiment configuration derived from config_exp_dirfix."""

from copy import deepcopy

from config_exp_dirfix import params as directional_params
from mypkg.constants import CbfType


params = deepcopy(directional_params)
params.CBF_type = CbfType.OPERATIONAL
params.log_filename = "output/experiment_operational.csv"
