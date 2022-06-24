from GripperModbusRtu import Gripper2f85Communication
from threading import Thread
import warnings
import time

class GripperCommand(Gripper2f85Communication):
    def __init__(self, id=0, stroke=0.085, comPort='/dev/ttyUSB0',baud_rate=115200, max_gap=0.1, min_gap=0.0):
        super().__init__(device_id=id, stroke=0.085, com_port=comPort, baud=baud_rate)

        self._max_stroke = max_gap
        self._min_stroke = min_gap
        self.gripper_stroke = stroke
        self.time_expired = False
        sec_before_expire = 2
        self.threadTimer = Thread(target=self.__internalCountDown_sec, args=(sec_before_expire,))
        self.threadCheckConnection = Thread(target=self.__CheckConnection)
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


    def _clamp_position(self,pos):
        out_of_bouds = False
        if (pos <= self._min_stroke):
            out_of_bouds = True
            pos_corrected = self._min_stroke
        elif (pos > self._max_stroke):
            out_of_bouds = True
            pos_corrected = self._max_stroke
        if(out_of_bouds):
            warnings.warn("Position (%.3f[m]) out of limits for %d[mm] gripper: \n- New position: %.3f[m]\n- Min position: %.3f[m]\n- Max position: %.3f[m]" % (pos, self._max_stroke, pos_corrected, self._min_stroke, self._max_stroke))
            pos = pos_corrected
        return pos

    def _clamp_speed(self,vel):
        out_of_bouds = False
        if (vel <= 0.013):
            out_of_bouds = True
            vel_corrected = 0.013
        elif (vel > 0.101):
            out_of_bouds = True
            vel_corrected = 0.1
        if(out_of_bouds):
            warnings.warn("Speed (%.3f[m/s]) out of limits for %d[mm] gripper: \n- New speed: %.3f[m/s]\n- Min speed: %.3f[m/s]\n- Max speed: %.3f[m/s]" % (vel, self._max_stroke*1000, vel_corrected, 0.013, 0.1))
            vel = vel_corrected
        return vel
    
    def _clamp_force(self,force):
        out_of_bouds = False
        if (force < 0.0):
            out_of_bouds = True
            force_corrected = 0.0
        elif (force > 100.0):
            out_of_bouds = True
            force_corrected = 100.0
        if(out_of_bouds):
            # rospy.logdebug("Force (%.3f[%]) out of limits for %d[mm] gripper: \n- New force: %.3f[%]\n- Min force: %.3f[%]\n- Max force: %.3f[%]" % (force, self._gripper.stroke*1000, force_corrected, 0, 100))
            force = force_corrected
        return force

    def getGipperStatus(self):
        
        feedback = []
        feedback.append(self.is_ready())
        feedback.append(self.is_reset())
        feedback.append(self.is_moving())
        feedback.append(self.object_detected())
        feedback.append(self.get_fault_status())
        feedback.append(self.get_pos())
        feedback.append(self.get_req_pos())
        feedback.append(self.get_current())

        return feedback

    def goTo(self, pos, speed, force):
        pos     = self._clamp_position(pos)
        speed   = self._clamp_speed(speed)
        force   = self._clamp_force(force)
        send_success = self.sendUnmonitoredMotionCmd(pos, speed, force)

        if send_success:
            print('Cmd Sent')
        else:
            print('Cmd Not Sent')

        return send_success
        
    def open_(self, speed=0.1, force=100):
        return self.sendUnmonitoredMotionCmd(self._max_stroke, speed, force)

    def close_(self, speed=0.1, force=100):
        return self.sendUnmonitoredMotionCmd(self._min_stroke, speed, force)


if __name__ == "__main__":

    gripperComm = GripperCommand()
    
    threadMonitorGripperStatus = Thread(target=gripperComm.getGipperStatus())

    if gripperComm.initialize():
        #print(gripperComm.getGipperStatus())
        gripperComm.goTo(pos=0.1, speed=0.3, force=100)
        #print(gripperComm.getGipperStatus())

        # print(gripperComm.open_())
        # print(gripperComm.close_())