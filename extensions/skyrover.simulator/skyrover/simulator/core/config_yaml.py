"""
| File: backend.py
| Author: Fei Wang (feiwang@dlmu.edu.cn)
| License: BSD-3-Clause. Copyright (c) 2025, Fei Wang. All rights reserved.
"""
__all__ = ["ConfigYaml"]

import yaml

import carb


class ConfigYaml():
    def __init__(self, filename: str = None):
        """Initialize the BackendConfig class
        """
        self.config_data = {}
        self.filename = filename
        if self.filename is not None:
            self.load()

        
    def load(self):
        """Load the configuration from the YAML file    
        """
        try:
            with open(self.filename, 'r') as file:
                self.config_data = yaml.safe_load(file)
        except:
            carb.log_warn("Could not retrieve config from: " + str(self.filename))


    def save(self):
        """Save the configuration to the YAML file    
        """
        temp_config = {}
        try:
            with open(self.filename, 'r') as file:
                temp_config = yaml.safe_load(file)
                
            # for key, value in self.__dict__.items():
            for key, value in self.config_data.items():
                temp_config[key] = value

            with open(self.filename, 'w') as file:
                yaml.dump(temp_config, file)
        except:
            carb.log_warn("Could not save config to: " + str(self.filename))


    def get(self, key: str, default_value=None):
        return self.config_data.get(key, default_value)


    def set(self, key: str, value):
        self.config_data[key] = value

