import rsb
import time

import logging

class RSBInterface():
        
    def __init__(self, scope, serverscope, handler, assumePoseRPC, getPosesRPC):
        self.log = logging.getLogger(self.__class__.__name__)  
        self.log.setLevel(logging.DEBUG)
        self._serverscope = serverscope
        self._scope = scope

        # ------------------------------------------------------------------
        
        self.__localServer = rsb.createServer(self._serverscope)
        self.__localServer.addMethod('assumePose', assumePoseRPC, requestType=str, replyType=bool)
        self.__localServer.addMethod('getPoses', getPosesRPC, replyType=str)
        self.__localServer.activate();
                
        self.log.debug('rsb server started on' + self._serverscope)
        print 'rsb server started on' + self._serverscope

        # ------------------------------------------------------------------
        
        self._listener = rsb.createListener(self._scope)
        self._listener.addHandler(handler)         

        self.log.debug('rsb listener started on' + self._scope)
        print 'rsb server started on' + self._scope  

    def publish(self, scope, msg):
        
        with rsb.createInformer("/meka/error", dataType=str) as informer:
            
            # Send and event using a method that directly accepts data.
            informer.publishData("MEKA ERROR: %s ", msg)



if __name__ == '__main__':
     print "test"
     
     with rsb.createRemoteServer('/meka/posture_execution/server') as server:
         
         print "server started"
         
         #print('server replied to synchronous getPoses: "%s"' % server.getPoses())
         
         reply = server.assumePose.async('allwaiting')
         
         while not reply.isDone():
             print "not done"
             time.sleep(1)
                    
         print "server done"
         print reply.get()
  

