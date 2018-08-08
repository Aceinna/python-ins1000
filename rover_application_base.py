# coding:utf8
from abc import ABCMeta, abstractmethod


class RoverApplicationBase():
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def on_reinit(self):
        pass

    @abstractmethod
    def on_find_active_rover(self):
        pass

    @abstractmethod
    def on_message(self, *args):
        pass

    @abstractmethod
    def on_exit(self):
        pass
