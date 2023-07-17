#!/usr/bin/env python

"""
This is a python file containning all the jobs for the test environment
"""

import rospy
import yaml
import os


class Jobs:
    def __init__(self):
        self.file_path = os.path.dirname(os.path.realpath(__file__))
        self.yaml_path = os.path.join(self.file_path, "params/job_describ.yaml")
        self.jobs = {}

    def get_jobs(self):
        with open(self.yaml_path, 'r') as stream:
            try:
                self.jobs = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)
        return self.jobs
