#from threading import Thread
import numpy

def wrap_to_pi(x):
    return numpy.mod(x+numpy.pi,2*numpy.pi)-numpy.pi

def sign(x):
    if x > 0: return +1
    if x < 0: return -1
    return 0

#
#class WorkerThread(Thread):
#
#    """Convenience wrapper around Thread"""
#
#    def __init__(self, run_callback, finished_callback = None):
#        super(WorkerThread, self).__init__()
#        self._run_callback = run_callback
#        self._finished_callback = finished_callback
#        
##    def start(self):
##        self.run = self._run_callback
#        
#    def run(self):
#        self._run_callback()
#        if self._finished_callback:
#            self._finished_callback()
            
