#! /usr/bin/env python
import roslib; roslib.load_manifest('C42_DRCSim2_tools')
import rospy
from rosgraph_msgs.msg import Clock
import threading
import functools
import copy

class controller(object):
    def __init__(self,name = "controller",Fs = 100,clock_topic = '/clock'):
        self._dt = rospy.Duration.from_sec(1/float(Fs))
        self._dt_real = copy.copy(self._dt)
        self._Started = False
        if rospy.get_name() == '/unnamed':
            rospy.init_node(name)
        self._clk = rospy.Subscriber(clock_topic,Clock,self.on_update_cb)

    def on_update_cb(self,msg):
        if self._Started:               
            if msg.clock - self._last_update >= self._dt:
                self._dt_real = msg.clock - self._last_update
                self._last_update = msg.clock
                self.on_update()
            pass

    def on_update(self):
        pass

    def start(self):
        self._Started = True
        self._last_update = rospy.Time.now()
        pass

    def stop(self):
        self._Started = False
        pass

    def reset(self):
        pass

class state_machine(controller):
    def __init__(self,name,Fs):
        controller.__init__(self,name,Fs)
        self.phase = 0
        self.states = []
        self.cur_state = 0

    def next_state(self):
        self.cur_state += 1
        if self.cur_state >= len(self.states):
            self.cur_state = 0
        pass

    def on_update(self):
        self.states[self.cur_state]()

class sm_timer(object):

    def __init__(self,tau,ID,init_phase = 0,max_phase = 10000):
        self.phase = init_phase
        self.MAX_PHASE = max_phase
        self.lock = threading.Lock()
        self.tau = tau
        self.listners = []
        self.ticks = [max_phase]
        self._cur_state = 0
        self.id = ID

    def _send_tick(self):
        for listner_cb in self.listners:
            listner_cb()
            self._cur_state += 1
        # self.reset()

    def add_listner(self,listner_callback):
        self.listners.append(listner_callback)

    def add_phase(self,tau):
        self.lock.acquire()
        self.phase += tau
        self.lock.release()

    def set_phase(self,phase):
        if phase <= self.MAX_PHASE:
            self.lock.acquire()
            self.phase = phase
            self.lock.release()

    def on_update(self):
        # print self.id, ' update'
        self.add_phase(self.tau)
        if self.phase >= self.ticks[self._cur_state]:
            self._send_tick()
            if self.phase >= self.MAX_PHASE:
                self.reset()
     
    def reset(self):
        self.lock.acquire()
        self.phase = 0
        self._cur_state = 0
        self.lock.release()

    def get_phase(self):
        self.lock.acquire()
        ph = copy.copy(self.phase)
        self.lock.release()
        return ph


class CPG(controller):
    def __init__(self,Fs):
        controller.__init__(self,Fs = Fs)
        self.timers = []
        self.id_dict = {}
        self.couples = []

    def on_update(self):
        # print 'CPG update', rospy.Time.now()
        for couple in self.couples:
            self._couple_func(couple)
        for timer in self.timers:
            timer.on_update()

    def _couple_func(self,couple):
        pass

    def add_timer(self,timer):
        self.id_dict.update({timer.id:len(self.timers)})
        self.timers.append(timer)
        print timer.id, ' added'

    def couple(self,id1,id2,strength):
        if id1 in self.id_dict.keys() and id2 in self.id_dict.keys():
            self.couples.append({'timers':[id1,id2],'k':strength})
            print self.couples
        else:
            print 'no such timers'

class filter():
    def __init__(self,b,a):
        self.buffer_lock = threading.Lock()
        self.a = a
        self.b = b
        self.u_buffer = [0.0 for k in xrange(len(self.b))]
        self.y_buffer = [0.0 for k in xrange(len(self.a))]
    def update(self,u):           
                self.buffer_lock.acquire()
                self.u_buffer.insert(0,u)
                self.u_buffer.pop()
                y = self._filter_func()
                self.y_buffer.insert(0,y)
                self.y_buffer.pop()
                self.buffer_lock.release()

    def _filter_func(self):
        pass

    def get_buffer(self):
        self.buffer_lock.acquire()
        buf = copy.copy(self.y_buffer)
        self.buffer_lock.release()
        return buf
