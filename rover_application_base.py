# coding:utf8
from abc import ABCMeta, abstractmethod


class RoverApplicationBase():
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def on_reinit(self):
        '''on_reinit will be invoked when user removes cable,
        '''
        pass

    @abstractmethod
    def on_find_active_rover(self):
        '''on_find_active_rover will be invoked when driver find an active rover.
        '''
        pass

    @abstractmethod
    def on_message(self, *args):
        '''on_message will be invoked when driver parse a whole frame successful.
        args[0] is packet type,
        args[1] is payload.
        args[2] is a flag to show whether current frame is variable length or not.
        '''
        pass

    @abstractmethod
    def on_exit(self):
        '''on_exit will be invoked when driver exit.
        '''
        pass
