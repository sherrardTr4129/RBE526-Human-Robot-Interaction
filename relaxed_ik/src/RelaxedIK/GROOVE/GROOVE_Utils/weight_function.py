__author__ = 'drakita'

import numpy as np
from abc import ABCMeta, abstractmethod, abstractproperty

class Weight_Function:
    __metaclass__ = ABCMeta

    @abstractmethod
    def name(self): pass

    @abstractmethod
    def __call__(self, vars): pass

class Identity_Weight(Weight_Function):
    def name(self): return 'Identity_weight'
    def __call__(self, vars): return 1.0

class Dynamic_Weight(Weight_Function):

    def __init__(self):
        self.value = 1.0

    def name(self): return 'Dynamic_weight'

    def __call__(self, vars): return self.value

    def set_value(self,new_value):
        self.value = new_value
    def get_value(self):
        return self.value