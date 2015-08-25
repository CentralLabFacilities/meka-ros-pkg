import rsb

import logging
import time

class RSBInterface():
        
    def __init__(self, scope, funct):
        self.log = logging.getLogger(self.__class__.__name__)  
        self.log.setLevel(logging.DEBUG)
        
        with rsb.createListener(scope) as listener:
    
            listener.addHandler(funct)           
            while True:
                time.sleep(1) 
        
        
    def publish(self, scope, msg):
        
        with rsb.createInformer("/meka/error", dataType=str) as informer:
            
            # Send and event using a method that directly accepts data.
            informer.publishData("MEKA ERROR: %s ", msg)

        
        
  
  

