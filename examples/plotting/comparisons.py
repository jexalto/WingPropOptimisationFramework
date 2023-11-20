# --- Built-ins ---
import os
import logging
from pathlib import Path

# --- Internal ---
from src.base import WingPropInfo
from src.postprocessing.utils.plotting_utils import get_niceColors, \
                                                    get_delftColors, \
                                                    get_SuperNiceColors, \
                                                    prop_circle
from examples.example_classes.PROWIM_classes import PROWIM_wingpropinfo
from src.postprocessing.comparison_plots import comaprison_wing

# --- External ---
import matplotlib.pyplot as plt
import niceplots
from openmdao.recorders.sqlite_reader import SqliteCaseReader
import numpy as np

logging.getLogger('matplotlib.font_manager').disabled = True
BASE_DIR = Path(__file__).parents[0]
        
if __name__=='__main__':
    db_name_isowing = os.path.join(BASE_DIR, '..', 'optimisation', 'results', 'final', 'data_wing.db')
    db_name_isoprop = os.path.join(BASE_DIR, '..', 'optimisation', 'results', 'final', 'data_propeller.db')
    db_name_coupled = os.path.join(BASE_DIR, '..', 'optimisation', 'results', 'final', 'data_wingprop.db')
    db_name_trim = os.path.join(BASE_DIR, '..', 'optimisation', 'results', 'final', 'data_wingprop_trim.db')

    wingiso_opt = SqliteCaseReader(db_name_isowing, pre_load=True)
    propiso_opt = SqliteCaseReader(db_name_isoprop, pre_load=True)
    coupled_opt = SqliteCaseReader(db_name_coupled, pre_load=True)
    trimmed = SqliteCaseReader(db_name_coupled, pre_load=True)

    comaprison_wing(db_name_isowing=wingiso_opt,
                      db_name_isoprop=propiso_opt,
                      db_name_coupled=coupled_opt,
                      db_name_trim=trimmed,
                      wingpropinfo=PROWIM_wingpropinfo,
                      savedir=os.path.join(BASE_DIR, 'figures'))