#!/usr/bin/env python
class IKReachException(Exception):
    def __init__(self,foot,requested_pos):
        self.foot = foot
        self.requested_pos = requested_pos