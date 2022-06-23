from GripperModbusRtu import Gripper2f85Communication
from threading import Thread
import warnings
import time

class GripperCommand(Gripper2f85Communication):
    def __init__(self, id=0, stroke=0.085, comPort='/dev/ttyUSB0',baud_rate=115200, max_gap=0.1, min_gap=0.0):
        super().__init__(device_id=id, stroke=0.085, com_port=comPort, baud=baud_rate)

        self.max_gap = max_gap
        self.min_gap = min_gap
        self.time_expired = False
        sec_before_expire = 2
        self.threadTimer = Thread(target=self.__internalCountDown_sec, args=(sec_before_expire,))
        self.threadCheckConnection = Thread(target=self.__CheckConnection, args=())
        self.is_gripper_connected = False

    def __internalCountDown_sec(self, seconds):
        #print("Started Countdown of ", seconds, "seconds")
        start = round(time.perf_counter(), 3)
        while ( (round(time.perf_counter(), 3) - start) < seconds):
            self.time_expired = False
        self.time_expired = True
    
    def __CheckConnection(self):
        self.is_gripper_connected = self.checkConnection()
        if self.is_gripper_connected:
            print('Connection Active')
        else:
            print('Connection Lost')
        time.sleep(1)

    def initialize(self):
        
        
        if self.gripperConnect():
            self.threadCheckConnection.start()
            self.is_gripper_connected = True



        self.deactivate_gripper()
        if self.is_reset():
            print("Activation Request\n")
            self.activate_gripper()
            time.sleep(2)
            
        if self.is_ready():
            print("Gripper is Rdy")
            return True
        else:
            return False

    def getGipperStatus(self):

        feedback = self.is_ready()
        feedback = self.is_reset()
        feedback = self.is_moving()
        feedback = self.object_detected()
        feedback = self.get_fault_status()
        feedback = self.get_pos()
        feedback = self.get_req_pos()
        feedback = self.get_current()

        return feedback


    def goTo(self, pos, speed, force):
        send_success = self.sendUnmonitoredMotionCmd(pos, speed, force)

        if send_success:
            print('Cmd Sent')
        else:
            print('Cmd Not Sent')

    def open_(self, speed=0.1, force=100):
        return self.sendUnmonitoredMotionCmd( self.max_gap, speed, force)

    def close_(self, speed=0.1, force=100):
        return self.sendUnmonitoredMotionCmd(self.min_gap, speed, force)


if __name__ == "__main__":

    gripperComm = GripperCommand()
    
    threadMonitorGripperStatus = Thread(target=gripperComm.getGipperStatus())

    if gripperComm.initialize():
        #print(gripperComm.getGipperStatus())
        gripperComm.goTo(pos=0.1, speed=0.3, force=100)
        #print(gripperComm.getGipperStatus())

        # print(gripperComm.open_())
        # print(gripperComm.close_())