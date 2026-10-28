# -*- coding: utf-8 -*-
"""Official single-case entry point for analytical-rigid sensor models."""
import os
import inspect

SCRIPT_PATH = inspect.currentframe().f_code.co_filename
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(SCRIPT_PATH)))
if not os.path.isfile(os.path.join(PROJECT_DIR, 'config', 'indenter_cases.json')):
    PROJECT_DIR = os.path.abspath(os.getcwd())
while not os.path.isfile(os.path.join(PROJECT_DIR, 'config', 'indenter_cases.json')):
    PARENT_DIR = os.path.dirname(PROJECT_DIR)
    if PARENT_DIR == PROJECT_DIR:
        raise RuntimeError('Could not find sensor_project/config/indenter_cases.json')
    PROJECT_DIR = PARENT_DIR
BUILDER = os.path.join(PROJECT_DIR, 'scripts', 'build_analytic_rigid_cases.py')
execfile(BUILDER, globals(), globals())
