#! /usr/bin/python3
"""
room_control software driver control window opener and window blind motors and user interface web service.

The Hardware relays controling the motors are switched by the output of shift registers 74HC595.
These get connected together as on large shift register where the input is set by the GPIO
of the Raspberry Pi microcomputer.

The software is

  Common:
    - class Config gives access to configurated values 
    - class Bitset to handle data for the shift register

  Motor control:
    - class ShiftRegister as driver to the 74HC595 hardware via the GPIO
    - class Motor to handle the status of a window opener or window blind motor
    - class Window to hanlde all motors of a window 
    - class RoomEvent
      - class SR_Event to handle shift register operations bypassing the motor (just for hardware testing) 
      - class WindowEvent 
        - class MotorEvent
          - class SetPositionEvent represents request to set the motors position
          - class DriveDirectionEvent represents request to drive a motors (manually) to a direction
    - class Room is a collection of all Windows of a room and handles the RoomEvents in a event loop thread

  Web interface:
    - class RequestHandler handles a UI request sends the answer and may push RoomEvents to the room
    - class CommServer threaded HTTPServer handling one ip address and port bound listening socket
    - class Project abstracts over all configurations for this projects room and web service
"""
__no_pydoc__ = []
#__all__ = ['Config','Bitset']
__author__ = "Eduard Gode <eduard@gode.de>"
__date__   = "12 June 2020"
__credits__ = """Jakob Gode, for the hardware design
"""

UseThreading = True # Without threading a hanging http request would block the server 


# HTTP server
from http.server import HTTPServer, BaseHTTPRequestHandler
import mimetypes
import ssl
import json
import time
import sys
import yaml
import types
import re
import _thread
import copy

try:
    import RPi.GPIO as GPIO
except:
    print("!!! use simulated RPI.GPIO !!!\n")
    class GPIO:
        """This is just a fake GPIO class to allow hardware independent development."""
        BOARD = 0
        OUT   = 0
        LOW   = 0
        HIGH  = 1
        def setmode( pin_name_mode ):
            if GPIO.BOARD != pin_name_mode: raise TypeError("illegal pin_name_mode '{}' in GPIO.setmode".format(pin_name_mode))
        def setup( pin, pin_mode ):
            if not isinstance(pin,int) or pin < 1 or pin >= 40: raise TypeError("illegal pin value '{}' in GPIO.output".format(pin))
            if GPIO.OUT: raise TypeError("illegal pin_mode '{}' in GPIO.setup".format(pin_mode))
        def output( pin, value ):
            if not isinstance(pin,int) or pin < 1 or pin >= 40: raise TypeError("illegal pin value '{}' in GPIO.output".format(pin))
            if GPIO.LOW != value and GPIO.HIGH != value: raise TypeError("illegal output value '{}' in GPIO.output".format(value))
            #print("GPIO.output( {}, {} )".format(pin,value))

        
if UseThreading:
    # to use a ThreadingHTTPServer
    import threading
    from socketserver import ThreadingMixIn
    class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
        pass
    ServerBaseClass = ThreadingHTTPServer
else:
    ServerBaseClass = HTTPServer



LOG_CRITICAL = 0
LOG_ERROR    = 1 
LOG_WARN     = 2 
LOG_DEBUG    = 3 
LOG_CALL     = 4
currentLogLevel = LOG_DEBUG
def log(level,msg):
    if level <= currentLogLevel:
        print(msg,file=sys.stderr)

            
MANDATORY = object()


class Config:
    """The Config class gives access to configuration variables"""
    __pydoc__ = {'__init__','get','subconfig','isExisting'}
    
    def __init__ (self,dict,defaults=None,name=None):
        """
        dict       configuration dict
        defaults   an optional dict representing default values
        name       an optional name for use as substructured data
        """
        self.dict     = dict
        self.defaults = defaults
        self.name     = name

        
    def _getFromDict(self,dict,name,default=None):
        v = dict
        for i in name.split('.'):
            if v is not None:
                try:
                    v = v[i]
                except KeyError:
                    v = None
            else:
                break
        if v is None: v = default
        return v

    
    def get(self,name,default=MANDATORY):
        """
        returns the config value specified by name
        
        name     is the name of the value
        default  is an optional value or the MANDATORY object
                 if default == MANDATORY then a ValueError is raised if no value is configured
        """
        if self.defaults is not None:
            default = self._getFromDict(self.defaults, name, default)
        val = self._getFromDict(self.dict, name, default)
        if val == MANDATORY:
            raise ValueError("config value '{}' is not specified in {} and {} in {}".format(name,self.dict,self.defaults,self.name));
        return val


    def subConfig(self,name,defaults=None):
        dict = self.get(name)
        if defaults is None and self.defaults is not None:
            defaults = self._getFromDict(self.defaults,name)
        c_name  = name if self.name is None else self.name + '.' + name;
        return Config(dict,defaults,c_name)


    def isExisting(self):
        return self.dict is not None


    def __str__(self):
        return "Config({},{},{})".format(self.name,self.dict,self.defaults)


    
class Bitset:
    """Simple implementation of a set of 0s an 1s"""
    __pydoc__ = {'__init__','clear','copy','set','get','toByteArray'}

    
    def __init__(self,bits):
        """
        bits       number of bits in this bitset
        """
        self.bits = bits
        self.clear()


    def clear(self):
        self.data = [ 0 for i in range(0,self.bits) ]


    def copy(self):
        return copy.deepcopy(self)


    def set(self,idx,bit=1):
        if not isinstance(idx,int):
            raise ValueError('idx must be a int value')
        if idx < 0 or idx >= self.bits:
            raise IndexError("idx {} is out of range [0,{}] error".format(idx,self.bits))
        self.data[idx] = ( 1 if bit else 0 )
        

    def get(self,idx):
        if not isinstance(idx,int):
            raise ValueError('idx must be a int value')
        if not isinstance(idx,int) or idx < 0 or idx >= self.bits:
            raise IndexRangeError('idx out of range error [0,{}]'.format(self.bits))
        return self.data[idx]

    
    def toByteArray(self):
        """
        convert bits of this bitset into a byte sequence (ls-bit-first, ls-byte-first)
        """
        a = []
        byteIdxMax = int((self.bits+7)/8)
        for byteIdx in range(0,byteIdxMax):
            b = 0;
            for bitIdx in range(0,8):
                idx = byteIdx * 8 + bitIdx
                if idx >= self.bits:
                    a.append(b)
                    return a
                if self.get(idx):
                    b |= 1 << bitIdx
            a.append(b)
        return a

    
    def __str__(self):
        return self.toByteArray()


    
class LatchTime:

    def __init__(self):
        self.time = None

    def setTime(self):
        self.time = time.time()

        
    
class ShiftRegister: # 74HC595
    """
    74HC959 hardware shift register

    pinDATA      is the number of the GPIO output pin connected to the DATA pin.
    pinCLOCK     is the number of the GPIO output pin connected to the CLOCK pin.
    pinLATCH     is the number of the GPIO output pin connected to the LATCH pin.
    pinENABLE    is the number of the GPIO output pin connected to the ENABLE pin.

    enableLevel  'HIGH' or 'LOW', it is the enabled signal voltage level, default: HIGH
    dataLevel    'HIGH' or 'LOW', it is the '1' data signal voltage level, default: HIGH

    bits         is the number of bits to control, may be more then 8 for sequencelly connected shift registers.

    reverseBitOrder  if True the bits are stuffed into the register starting by the last bit (bits-1)

    setupTime    the minimal time in seconds after data is set before the raising edge of clock.
    holdTime     the minimal time in seconds for data must be stable after the raising edge of clock.
    clockTime    the minimal time in seconds between raising two raising clock edges.
                 It may be 0 if it is less than setupTime + holdTime

    The methods to modify the bits just sets it into an output cache:
    clearBits()
    setBit()
  
    The latch() method wites all bits into the hardware and raises the hardware latch pin.

    The enable() method hardware latch pin.
    """
    __no_pydoc__ = {'_shiftBit'}

    outLevel_LOW_HIGH = [ GPIO.LOW, GPIO.HIGH ]
    outLevel_HIGH_LOW = [ GPIO.HIGH, GPIO.LOW ]
    
    def __init__(self,pinDATA,pinCLOCK,pinLATCH,pinENABLE,bits=8,enableLevel='HIGH',dataLevel='HIGH',reverseBitOrder=True,setupTime=0,holdTime=0,clockTime=0):
        print("ShiftRegister: pinDATA = '{}'".format(pinDATA))
        self.pinDATA   = pinDATA
        self.pinCLOCK  = pinCLOCK
        self.pinLATCH  = pinLATCH
        self.pinENABLE = pinENABLE

        if enableLevel != 'HIGH' and enableLevel != 'LOW': raise ValueError( 'enableLevel must be HIGH or LOW' )
        if dataLevel   != 'HIGH' and dataLevel   != 'LOW': raise ValueError( 'dataLevel   must be HIGH or LOW' )
        self.enableLevel = ( self.outLevel_LOW_HIGH if enableLevel == 'HIGH' else self.outLevel_HIGH_LOW )
        self.dataLevel   = ( self.outLevel_LOW_HIGH if dataLevel   == 'HIGH' else self.outLevel_HIGH_LOW )

        if setupTime < 0: raise ValueError('setupTime must be 0 or positive float')
        if holdTime  < 0: raise ValueError('holdTime must be 0 or positive float')
        if clockTime < 0: raise ValueError('clockTime must be 0 or positive float')
        self.setupTime = setupTime
        self.holdTime  = holdTime
        self.clockTime = 0 if clockTime < setupTime + holdTime else clockTime 

        self.reverseBitOrder = reverseBitOrder
        
        print("shift register enableLevel: '{}'".format(enableLevel))
        print("shift register dataLevel: '{}'".format(dataLevel))
        
        self.bitset    = Bitset(bits)
        self.dirty     = True
        self.enabled   = None
        self.output    = None
        self.lastTime  = time.time()

        self.nextLatchTime = LatchTime()

        self.stData    = None
        self.stClock   = None
        self.stLatch   = None

        
    def togglePin(self,pin):
        """
        toggle DATA, CLOCK or LATCH pin. This is just for hardware testing!
        pin numbers: -2 = DATA, -3 = CLOCK, -4 = LATCH
        """
        if   -2 == pin:
            self.stData  = 1 - self.stData
            GPIO.output(self.pinDATA, self.dataLevel[self.stData and 1 or 0])
        elif -3 == pin:
            self.stClock = 1 - self.stClock
            GPIO.output(self.pinCLOCK, self.stClock and GPIO.HIGH or GPIO.LOW)
        elif -4 == pin:
            self.stLatch = 1 - self.stLatch
            GPIO.output(self.pinLATCH, self.stLatch and GPIO.HIGH or GPIO.LOW)

            
    def _shiftBit(self,bit):
        """
        shift a bit into the shift register hardware.
        bit is 0 or 1.
        This method handles the clockTime, setupTime and holdTime by busy waiting.
        """
        if self.clockTime:
            now = time.time()
            dt  = now - self.lastTime
            if dt < self.clockTime:
                time.sleep(clockTime-dt) # sleep to assure data is processed
            self.lastTime = now
        GPIO.output(self.pinCLOCK, GPIO.LOW)
        self.stClock = 0
        GPIO.output(self.pinDATA,  self.dataLevel[bit and 1 or 0])
        self.stData = bit and 1 or 0
        if self.setupTime:
            time.sleep(self.setupTime) # sleep to assure data valid
        GPIO.output(self.pinCLOCK, GPIO.HIGH)
        self.stClock = 1
        if self.holdTime:
            time.sleep(self.holdTime) # sleep to assure data valid


    def clearRegister(self):
        """
        clear the hardware register by shifting 'bits' times a 0 bit.
        bit is 0 or 1.
        This method handles the clockTime, setupTime and holdTime by busy waiting.
        """
        for i in range(0,self.bits):
            self._shiftBit(0)
        self.dirty = True


    def clearBits(self):
        """
        set all bits to 0.
        """
        self.bitset.clear()
        self.dirty = True

    
    def setBit(self,idx,value=1):
        self.bitset.set(idx,value)
        self.dirty = True

        
    def latch(self): # shift all bits into register and latch
        if not self.dirty: return
        print( "\n latch\n" )
        GPIO.output(self.pinLATCH, GPIO.LOW)
        self.stLatch = 0
        idxMax = self.bitset.bits - 1
        for idx in range(0, self.bitset.bits):
            bitsetIdx = idxMax - idx if self.reverseBitOrder else idx
            self._shiftBit( self.bitset.get(bitsetIdx) )
        GPIO.output(self.pinLATCH, GPIO.HIGH)
        self.stLatch = 1
        #if self.holdTime: time.sleep(self.holdTime) # sleep to assure data latched
        self.nextLatchTime.setTime()
        self.nextLatchTime = LatchTime()
        self.dirty = False
        self.output = self.bitset.copy()


    def enable(self,bit=1):
        self.enabled = bit and 1 or 0
        GPIO.output( self.pinENABLE, self.enableLevel[self.enabled] )

        
    def getOutputBit(self,idx):
        """
        Returns bit idx of last latched data.
        This is the current hardware output bit state.
        """
        return self.bitset.get(idx)

        
    def getState(self):
        return  { 'enable': self.enabled,
                  'bits':   self.bitset.bits,
                  'output': self.output,
                  'data':   self.stData,
                  'clock':  self.stClock,
                  'latch':  self.stLatch
                }


    
class Motor:

    busy_motors = {}

    DIR_FORWARD  =  0 # open the window, lower the blind, close the blind slats
    DIR_BACKWARD =  1 # close the window, lower the blind, open the blind, slats
    STOP_MOTOR   = -1 # for _planDrive

    STATE_NEED_POS      = 0 # initital state
    STATE_DETERMINE_POS = 1 # driving to determine the position
    STATE_DRIVE_MANUAL  = 2 # initital state
    STATE_DRIVE_TO_POS  = 3 # initital state
    
    # The tricky thing of the blind motor is to handle how far it moved backward.
    # Moving backward opens the blind slat before raising the blind 
    
    def checkBusyMotors():
        ids = [ id for id in Motor.busy_motors ]
        for id in ids:
            try:
                motor = Motor.busy_motors[id]
            except:
                break
            #log(LOG_CALL ,"\ncall motor._checkDrive()".format(motor.id))
            #print("\ncall motor._checkDrive(): {}".format(motor.id))
            motor._checkDrive()
            
        
    def __init__(self,room,id,config,motorType,hard_position=None):
        self.id        = id
        self.room      = room
        self.motorType = motorType
        
        self.bitOn           = config.get('on_bit',MANDATORY)
        self.bitDir          = config.get('dir_bit',MANDATORY)
        self.position_min    = config.get('position_min',MANDATORY)
        self.position_max    = config.get('position_max',MANDATORY)
        self.speed_dir0      = config.get('speed_dir0',MANDATORY)
        self.speed_dir1      = config.get('speed_dir1',MANDATORY)
        self.hard_pos_min    = config.get('hard_pos_min',MANDATORY)
        self.hard_pos_max    = config.get('hard_pos_max',MANDATORY)
        self.slip            = config.get('slip',0)
        self.drift           = config.get('drift',0)

        if motorType == 'blind':
            self.pos_back_max = config.get('turn_pos_diff',MANDATORY)
            self.angle_min    = config.get('angle_min',MANDATORY)
            self.angle_max    = config.get('angle_max',MANDATORY)
            if self.angle_min == self.angle_max:
                raise ValueError('blind.angle_min = blind.angle_max')
        else:
            self.pos_back_max = 0
            self.angle_min    = 0
            self.angle_max    = 0
           
        self.state           = 0 # 0 = stopped, 1 = determine, 2 = manual, 3 = to position
        self.d_pos_max       = ( self.hard_pos_max - self.hard_pos_min ) / 100
        self.d_pos_back_max  = self.pos_back_max / 30
        self.drift_sum       = 0
        
        self.hard_pos        = hard_position
        self.hard_pos_back   = 0

        self.drive_start_time      = None
        self.drive_start_pos       = None
        self.drive_start_pos_back  = None
        self.drive_start_drift_sum = None
        self.drive_to_pos          = None
        self.drive_to_pos_back     = None
        self.drive_direction       = None

    def __str__(self):
        return 'Motor: {}'.format(str({'id':self.id,'bitOn':self.bitOn,'bitDir':self.bitDir,'position_min':self.position_min,'position_max':self.position_max}))

    
    def getPosition(self):
        if self.hard_pos is None:
            self.determinePosition()
            while self.hard_pos is None:
                time.sleep(1)
        pos = self.hard_pos
        if   pos < self.position_min: pos = self.position_min
        elif pos > self.position_max: pos = self.position_max
        return pos

    
    def getAngle(self):
        if 'blind' != self.motorType: return 0
        pos = self.hard_pos_back
        da  = self.angle_max - self.angle_min
        angle = pos / self.pos_back_max * da + self.angle_min  
        return angle
        
        
    def determinePosition(self,dir=None):
        print( "determinePosition() state: {}".format(self.state) )
        if dir is None:
            if self.hard_pos is None:
                dir = 1;
            else:
                p = ( self.hard_pos - self.hard_pos_min ) / ( self.hard_pos_max - self.hard_pos_min )
                dir = ( 1 if p < 0.5 else 0 )
        if 1 == dir:
            self.hard_pos_back = 0
            self.hard_pos      = self.hard_pos_max
            self._startDrive(None,self.hard_pos_min,self.pos_back_max)
        else:
            self.hard_pos_back = self.pos_back_max
            self.hard_pos      = self.hard_pos_min
            self._startDrive(None,self.hard_pos_max,0)
        self.state = 1
            

    def _calcHardPos(self):
        #print( "_calcHardPos() state: {}".format(self.state) )
        if not self.state: return
        
        now = time.time()
        pos_back = self.drive_start_pos_back
        pos      = self.drive_start_pos
            
        dt = now - self.drive_start_time.time
        if self.DIR_FORWARD == self.drive_direction:    
            s  = dt * self.speed_dir0
            pos_back -= s
            if pos_back < 0:
                pos -= pos_back
                pos_back = 0
            if pos > self.hard_pos_max: pos = self.hard_pos_max
        elif self.DIR_BACKWARD == self.drive_direction:
            s  = dt * self.speed_dir1
            pos_back += s
            if pos_back > self.pos_back_max:
                pos -= ( pos_back - self.pos_back_max )
                pos_back = self.pos_back_max
            if pos < self.hard_pos_min: pos = self.hard_pos_min
        else:
            raise ValueError("Internal error: direction={}".format(self.drive_direction))

        #log(LOG_DEBUG, "_calcHardPos returns: {}, {}".format(pos,pos_back))
        self.hard_pos      = pos
        self.hard_pos_back = pos_back


    def _startMotor(self,dir):
        #print("_startMotor(dir={}), bitDir:{}, bitOn:{}".format(dir,self.bitDir,self.bitOn))
        sr = self.room.shift_register
        sr.setBit(self.bitDir,dir)
        sr.setBit(self.bitOn,1)
        #sr.latch()
        self._calcHardPos() # calculate when switching the motor, before setting new drive parameter
        self.drive_start_time     = sr.nextLatchTime # time.time()
        self.drive_direction      = dir
        self.drive_start_pos      = self.hard_pos
        self.drive_start_pos_back = self.hard_pos_back
        Motor.busy_motors[self.id] = self

        
    def _stopMotor(self):
        #print("_stopMotor({}), {}, {}, {}, {}".format(dir,self.bitDir,self.bitOn, self.hard_pos, self.hard_pos_back))
        sr = self.room.shift_register
        sr.setBit(self.bitDir,0)
        sr.setBit(self.bitOn,0)
        #sr.latch()
        self._calcHardPos() # calculate when switching the motor off 
        self.drive_direction      = None
        self.drive_start_pos      = None
        self.drive_start_pos_back = None
        self.drive_start_time     = None
        self.state = 0
        del Motor.busy_motors[self.id]


    def _planDrive(self): # determine direction to drive 
        #print("_planDrive() state:{}, dir:{}, to: {},{}".format(self.state,self.drive_direction,self.drive_to_pos,self.drive_to_pos_back) )

        dir = None
        
        # first check position
        if self.drive_to_pos is not None:
            d_pos = self.drive_to_pos - self.hard_pos
            #print("d_pos: {}, d_pos_max = {}".format(d_pos, self.d_pos_max))
            if abs(d_pos) > self.d_pos_max:
                return self.DIR_FORWARD if self.hard_pos < self.drive_to_pos else self.DIR_BACKWARD
            else:
                dir = self.STOP_MOTOR

        if self.drive_to_pos_back is not None:
            d_pos = self.drive_to_pos_back - self.hard_pos_back
            #print("d_pos: {}, d_pos_max_back = {}".format(d_pos, self.d_pos_back_max))
            if abs(d_pos) > self.d_pos_back_max:
                return self.DIR_BACKWARD if self.hard_pos_back < self.drive_to_pos_back else self.DIR_FORWARD
            else:
                dir = self.STOP_MOTOR

        return dir
            

    def _checkDrive(self):
        #print("_checkDrive() id: {}, state:{}, dir:{}".format(self.id,self.state,self.drive_direction) )
        if 0 == self.state: return
        if self.drive_start_time.time is None:
            print( "not yet latched time for {},{}".format(self.id,self.motorType) )

        self._calcHardPos()

        if self.STATE_DETERMINE_POS == self.state or self.STATE_DRIVE_TO_POS == self.state:
            dir = self._planDrive()
        elif self.STATE_DRIVE_MANUAL == self.state:
            dir = None
            if self.DIR_FORWARD == self.drive_direction:
                if self.hard_pos >= self.hard_pos_max and self.hard_pos_back >= self.hard_pos_back_max:
                    dir = self.STOP_MOTOR
            elif self.DIR_BACKWARD == self.drive_direction:
                if self.hard_pos <= self.hard_pos_min and self.hard_pos_back <= 0:
                    dir = self.STOP_MOTOR
            
        #log(LOG_DEBUG, "_checkDrive state:{} dir:{} is:{},{} to:{},{}, new dir:{}".format(self.state, self.drive_direction, self.hard_pos, self.hard_pos_back, self.drive_to_pos, self.drive_to_pos_back, dir))

        if dir is None:
            log(LOG_DEBUG, 'dir=None in _checkDrive')
        elif self.STOP_MOTOR == dir:
            self._stopMotor()
        elif dir != self.drive_direction:
            self._startMotor(dir)
        else:
            #log(LOG_DEBUG, 'dir not changed in _checkDrive()')
            pass

    def _startDrive(self,dir,to_pos,to_pos_back):
        log( LOG_CALL, "_startDrive({}, {}, {})".format(dir,to_pos,to_pos_back))

        if self.hard_pos is None:
            self.determinePosition()

        state = self.STATE_NEED_POS
        if dir is not None:
            if to_pos is not None or to_pos_back is not None:
                raise ValueError( 'in _startDrive dir and to_pos/to_pos_back are specified')
            state = self.STATE_DRIVE_MANUAL
        else:
            if to_pos is None and to_pos_back is None:
                raise ValueError( 'in _startDrive neither dir nor to_pos/to_pos_back are specified')
            if to_pos is not None:
                if to_pos < self.hard_pos_min or to_pos > self.hard_pos_max:
                    raise ValueError( '_startDrive to_pos: {} is not in range [{}, {}]'.format(self.hard_pos_min,self.hard_pos_max))
            if to_pos_back is not None:
                if to_pos_back < 0 or to_pos_back > self.pos_back_max:
                    raise ValueError( '_startDrive to_pos_back: {} is not in range [0, {}]'.format(self.pos_back_max))
            self.drive_to_pos      = to_pos
            self.drive_to_pos_back = to_pos_back
            dir = self._planDrive()
            if -1 == dir:
                log(LOG_DEBUG,'_startDrive: already on position')
                self.drive_to_pos      = None
                self.drive_to_pos_back = None
                return
            state = self.STATE_DRIVE_TO_POS

        self._startMotor(dir)
        self.state = state 
        
        
    def _stopDrive(self):
        self._stopMotor()
        self.drive_to_pos      = None
        self.drive_to_pos_back = None


    def driveToPosition(self,pos,angle=None):
        pos_back = None
        if angle is not None:
            da = self.angle_max - self.angle_min
            pos_back = (angle - self.angle_min ) / da * self.pos_back_max
            if pos_back < 0:                 pos_back = 0
            if pos_back > self.pos_back_max: pos_back = self.pos_back_max
        if   pos <= self.position_min: pos = self.hard_pos_min
        elif pos >= self.position_max: pos = self.hard_pos_max
        self._startDrive(None,pos,pos_back)

    
    def driveInDirection(self,dir):
        if -1 == dir:
            if 0 == self.state:
                return
            if 2 == self.state:
                self._stopDrive()
                return
        self._startDrive(dir,None,None)

        
    def getConfig(self):
        data = { 'bitOn':  self.bitOn,
                 'bitDir': self.bitDir,
                 'hard_pos_min': self.hard_pos_min,
                 'hard_pos_max': self.hard_pos_max,
                 'position_min': self.position_min,
                 'position_max': self.position_max,
                 'slip':  self.slip,
                 'drift': self.drift
               }
        
        if 'blind' == self.motorType: 
            data['pos_back_max'] = self.pos_back_max
            data['angle_min'] = self.angle_min
            data['angle_max'] = self.angle_max
            
        return data


    def getState(self):
        data = { 'hard_pos':        self.hard_pos,
                 'hard_pos_back':   self.hard_pos_back,
                 'position':        self.getPosition(),
                 'drift_sum':       self.drift_sum,
                 'state':           self.state,
                 'drive_direction': self.drive_direction,
               }
        return data

    
    
class Window:

    def __init__(self,config,room):
        motors = room.motors
        self.room = room
        self.id   = config.get('id')
        self.name = config.get('name')

        print("init window: '{}'".format(self.id))
        self.opener = self.addMotor(self.id+'_opener',config.subConfig('opener'),'opener')
        self.blind  = self.addMotor(self.id+'_blind', config.subConfig('blind' ),'blind')


    def addMotor(self,id,config,motorType):
        room  = self.room
        motor = Motor(room,id,config,motorType)
        id    = motor.id
        if id in room.motors:
            raise ValueError("{} motor id '{}' is already registered".format(motorType,id))
        if 'opener' == motorType:
            self.opener = motor
        else:
            self.blind = motor
        return motor

    
    def getConfig(self):
        return { 'id':     self.id,
                 'name':   self.name,
                 'opener': self.opener.getConfig(),
                 'blind':  self.blind.getConfig()
               }

    
    def getState(self):
        return { 'id':     self.id,
                 'name':   self.name,
                 'opener': self.opener.getState(),
                 'blind':  self.blind.getState()
               }
    


class RoomEvent:
    
    def __init__(self,room,args):
        self.room = room
        self.args = args
        self.time = time.time()

        
    def handle(self,room):
        print( "Event handling of base class event with args: {}".format(str(self.args)) )

        
    def isWaiting(self): return False



class SR_Event (RoomEvent):

    def __init__(self,room,args):
        super().__init__(room,args)
        self.action = args.pop('action')
        self.bit    = int(args.pop('bit'))
        if 'toggle_bit' != self.action: raise ValueError('unknown sr action: '+self.action)
        

    def handle(self,room):
        print( "\nSR_Event.handle() toggle bit {}".format(self.bit) )
        sr  = self.room.shift_register
        bit = self.bit
        if   -1 == bit:
            sr.enable( 0 if sr.enabled else 1 )
        elif 0 > bit:
            sr.togglePin(bit)
        else:
            b = sr.getOutputBit(bit)
            sr.setBit(bit, 0 if b else 1 )
            #sr.latch()
        return { 'result': 0, 'sr_state': sr.getState() }

    

class WindowEvent (RoomEvent):

    def __init__(self,room,args):
        super().__init__(room,args)
        self.window = room.windows[args.pop('win_id')]


    def isWaiting(self):
        return False
    
    
    
class MotorEvent (WindowEvent):
    """
    This is the abstract base class for events handling window motor events.
    """
    __pydoc__ = [];

    def __init__(self,room,args):
        super().__init__(room,args)
        if   'opener' == args['motor']: self.motor = self.window.opener
        elif 'blind'  == args['motor']: self.motor = self.window.blind
        else: raise ValueError("illegal motor parameter '{}'".format(args[motor]))
        #print( "MotorEvent_end( {}, {} ), motor: {}".format(room,args,str(self.motor)) )
        

    def isWaiting(self):
        motor = self.motor
        if motor.hard_pos is None: motor.determinePosition()
        motor._checkDrive()
        if 1 == motor.state: return True
        return False


    
class SetPositionEvent (MotorEvent):
    """
    This is a RoomEvent which starts a window motor driving to a specified position.
    """
    __pydoc__ = ['__init__'];
    
    def __init__(self,room,args):
        
        super().__init__(room,args)
        self.pos   = int(args['pos']  )
        self.angle = int(args['angle']) if args['angle'] is not None else None
        #print( "SetPositionEvent.__init__( {}, {} ), motor: {}".format(room,args,str(self.motor)) )
        if self.pos < self.motor.position_min or self.pos > self.motor.position_max:
            raise ValueError('pos {} is not in the range of [{}, {}]'.format(self.pos,self.motor.position_min,self.motor.position_max))
        if self.angle is not None and ( self.angle < self.motor.angle_min or self.angle > self.motor.angle_max ):
            raise ValueError('angle {} is not in the range of [{}, {}]'.format(pos,self.motor.angle_min,self.motor.angle_max))
        

    def handle(self,room):
        print( "\nSetPositionEvent.handle() {}, {}".format(self.pos,self.angle) )
        self.motor.driveToPosition(self.pos,self.angle)



class DriveDirectionEvent (MotorEvent):
    """
    This is a RoomEvent which starts a window motor driving in a specified direction.
    """
    __pydoc__ = ['__init__'];

    def __init__(self,room,args):
        """
        room  is the room to 
        args  is a dictionary specifying
              dir    '0' = forward (close window,lower blind,close slat), '1' = backward
              win_id id of window
              motor  id of motor to drive 'opener' or 'slat' or 'blind'
              stop   '1' = stop the motor, '0' = start/continue driving
        """
        super().__init__(room,args)
        self.stop  = ( '1' == args['stop'] )
        if not self.stop:
            self.direction = int(args['dir'])
            if self.direction != 0 and self.direction != 1:
                raise ValueError('dir {} must be 0 or 1')

    def handle(self,room):
        log(LOG_DEBUG, "handle SetPositionEvent()")
        self.motor.driveInDirection(-1 if self.stop else self.direction)



class Room:
    """
    An object of this class represents a room whith its windows.
    """
    __pydoc__ = ['__init__']
    
    def __init__(self,config):
        """
        config  is a Config object containing:
          'spin_rate'
          'shift_register'
          'defaults'
          'windows' with 'windows.count', 'windows_0' .. 'windows_N' defaults
        """
        self.config    = config
        self.spinRate  = self.config.get('spin_rate', 10) # hz
        self.events    = []
        self.eventLock = _thread.allocate_lock()
        self.reqLock   = _thread.allocate_lock()
        self.initError = None
        self.windows   = {}
        self.motors    = {}

        self.init_shift_register()
        self.init_windows()
        self.init_dimmer()
        self.shift_register.enable()
        if self.initError is not None:
            raise self.initError

        
    def init_shift_register(self):
        pinDATA         = self.config.get('shift_register.pinDATA')
        pinCLOCK        = self.config.get('shift_register.pinCLOCK')
        pinLATCH        = self.config.get('shift_register.pinLATCH')
        pinENABLE       = self.config.get('shift_register.pinENABLE')
        bits            = self.config.get('shift_register.bits',16)
        enableLevel     = self.config.get('shift_register.enableLevel','HIGH')
        dataLevel       = self.config.get('shift_register.dataLevel',  'HIGH')
        reverseBitOrder = self.config.get('shift_register.reverseBitOrder',False)
        setupTime       = self.config.get('shift_register.setupTime',0)
        holdTime        = self.config.get('shift_register.holdTime', 0)
        clockTime       = self.config.get('shift_register.clockTime',0)
        GPIO.setmode(GPIO.BOARD) # Number GPIOs by its physical location
        GPIO.setup(pinDATA,   GPIO.OUT)
        GPIO.setup(pinCLOCK,  GPIO.OUT)
        GPIO.setup(pinLATCH,  GPIO.OUT)
        GPIO.setup(pinENABLE, GPIO.OUT)
        self.shift_register = ShiftRegister(pinDATA, pinCLOCK, pinLATCH, pinENABLE, bits=bits, enableLevel=enableLevel, dataLevel=dataLevel, reverseBitOrder=reverseBitOrder, setupTime=setupTime, holdTime=holdTime)


    def init_windows(self):
        count    = self.config.get('windows.count')
        defaults = self.config.get('defaults')
        print("defaults are: {}".format(defaults))
        for i in range(0,count):
            win_key = 'window_'+str(i)
            win_config = self.config.subConfig(win_key, defaults)
            if not win_config.isExisting():
                error = "window '{:}' is not configured".format(win_key)
                log( LOG_ERROR, error )
                self.initError = ValueError( error )
            else:
                window = Window(win_config,self)
                id = window.id
                if id in self.windows:
                    raise ValueError( "window id '{:}' is not unique".format(id) )
                self.windows[id] = window
        self.shift_register.latch()

    def popEvent(self):
        self.eventLock.acquire()
        event = None
        for i in range(0,len(self.events)):
            if not self.events[i].isWaiting():
                event = self.events.pop(i)
                break
        self.eventLock.release()
        return event


    def pushEvent(self,event):
        self.eventLock.acquire()
        self.events.append(event)
        self.eventLock.release()

    
    def handleEvents(self):
        while len(self.events):
            event = self.popEvent()
            if event is None: break
            print("\nhandle event "+str(event))
            event.handle(self)
        if self.shift_register.dirty:
            self.shift_register.latch()


    def getState(self,request,args):
        self.reqLock.acquire()
        if 'static' == request:
            return { 'windows': [ id for id in self.windows ] }
        elif 'window_config' == request:
            window = self.window[args['id']]
            return window.getConfiguration()
        elif 'window_state' == request:
            window = self.window[args['id']]
            return window.getState()
        self.reqLock.release()


    def doRequest(self,action,args):
        if 'sr_state' == action:
            return { 'result': 0, 'sr_state':   self.shift_register.getState() }
        if 'window/config' == action:
            window = self.windows[args['win_id']]
            return { 'result': 0, 'win_config': window.getConfig() }
        if 'room/action' == action:
            room = self
            action = args['action']
            if 'all_close' == action:
                for win in self.windows.values():
                    motor = win.opener
                    room.pushEvent( SetPositionEvent(self,{'win_id':win.id,'motor':'opener','pos':motor.position_min,'angle':None}) )
            if 'all_open' == action:
                for win in self.windows.values():
                    motor = win.opener
                    room.pushEvent( SetPositionEvent(self,{'win_id':win.id,'motor':'opener','pos':motor.position_max,'angle':None}) )
            if 'all_raise' == action:
                for win in self.windows.values():
                    motor = win.blind
                    room.pushEvent( SetPositionEvent(self,{'win_id':win.id,'motor':'blind','pos':motor.position_min,'angle':motor.angle_max}) )
            if 'all_lower' == action:
                for win in self.windows.values():
                    motor = win.blind
                    room.pushEvent( SetPositionEvent(self,{'win_id':win.id,'motor':'blind','pos':motor.position_max,'angle':motor.angle_min}) )
            #return { 'result': 0, 'room_state': self.getState('window_state',{'win_id':None}) }
            return self.doRequest('window/state',{'win_id':None})
        if 'window/state' == action:
            win_id = args['win_id']
            if win_id is None:
                data = { 'result': 0,
                         'window': { 'count': len(self.windows) },
                       }
                i = 0
                for win in self.windows.values():
                    data['window_'+str(i)] = win.getState()
                    i += 1
                return data
            else:
                window = self.windows[args['win_id']]
                return { 'result': 0, 'win_config': window.getState('window_state') }
        return { 'result': 1, 'error': 'unknown request '+str(action) }

        
    def init_dimmer(self):
        pass

    
    def serve_forever(self):
        spinInterval = 1 / self.spinRate
        logRate = 0.2
        logDiff = logRate / self.spinRate
        logCnt  = 0
        log(LOG_DEBUG,"logDiff: {}".format(logDiff))
        while True:
            self.eventLock.acquire()
            if self.shift_register.dirty:
                self.shift_register.latch()
            Motor.checkBusyMotors()
            self.eventLock.release()
            self.handleEvents()
            logCnt += logDiff
            if logCnt > 1: logCnt = 0; print('room thread events: '+str(self.events));
            time.sleep(spinInterval)

        
    def getConfig(self):
        data = { 'spin_rate': self.spin_rate,
                 'shift_register': { 'pinDATA':   self.config.get('shift_register.pinDATA'),
                                     'pinCLOCK':  self.config.get('shift_register.pinCLOCK'),
                                     'pinLATCH':  self.config.get('shift_register.pinLATCH'),
                                     'pinENABLE': self.config.get('shift_register.pinENABLE'),
                                   },
                 'windows': { 'count': len(self.windows) }
               }
        i = 0
        for win in self.windows.values():
            data['window_'+str(i)] = win.getConfig()
            i += 1
        return data



svgExtPat   = re.compile(r'\.svg$')
templatePat = re.compile(br'<!--\s+start\s+template="([\w\.\-]*)"\s*-->(.*?)<!--\s+end\s+template="\1"\s*-->',re.S)
usePat1     = re.compile(br'(<use\s+id="[\w\.\-]+"\s+href="#[\w\.\-]+"\s+x="\d+"\s+y="\d+"\s*/>)')
usePat2     = re.compile(br'(<use\s+id="([\w\.\-]+)"\s+href="#([\w\.\-]+)"\s+x="(\d+)"\s+y="(\d+)"\s*/>)')

def handleContentSVG(content): # replace <use href="#templID" ...> by content of <!-- start template="templID"-->...<!-- end template="templID" -->
    log(LOG_CALL, 'handleContentSVG()' )
    map = {}
    for c in templatePat.finditer(content):
        #print("c: "+str(c)+', '+str(c.group(0))+', '+str(c.group(1))+', '+str(c.group(2)))
        log(LOG_WARN, 'found template for <use id="{}" ...>'.format(c.group(1)))
        map[c.group(1)] = c.group(2)
    cont = b'';
    #print("svg map: "+str(map))
    for c in usePat1.split(content):
        #print( "\n\n\nchunk: '{}'".format(str(c)) )
        m = usePat2.match(c)
        if m:
            id   = m.group(2)
            href = m.group(3)
            x    = m.group(4)
            y    = m.group(5)
            log(LOG_WARN, "use: {} href='{}'".format(str(id),str(href)))
            cont += b'<g id="'+id+b'" transform="translate('+x+b','+y+b')">\n'+map[href]+b'\n</g>'
        else:
            cont += c
    return cont

                              
    
class JSONEncoder(json.JSONEncoder):
    """
    JSONEncoder to handle custom data classes.
    """
    __pydoc__ = []
    
    def default(self, obj):
        if isinstance(obj, Bitset):
            return obj.toByteArray()
        # Let the base class default method raise the TypeError
        return json.JSONEncoder.default(self, obj)

        

class RequestHandler(BaseHTTPRequestHandler):
    """
    This class handles the web requests.
    """
    __pydoc__ = [ 'server_version' ]

    server_version = 'Room-Control/0.0.2'
                   
    def get_parsed_path(self):
        try:
            return self.parsed_path
        except AttributeError:
            pass

        try:
            ( path, query_string ) = self.path.split('?')
        except ValueError:
            ( path, query_string ) = ( self.path, None )

        if query_string is None:
            parameters = None
        else:
            parameters = [ p.split('=') for p in query_string.split('&') ]

        self.parsed_path = { 'path': path, 'query_string': query_string, 'parameters': parameters }
        return self.parsed_path


    def parameter_value(self,key,default=None,value=1): # if parameter is not specified return default, if parameter is just a flag return value 
        parsed_path = self.get_parsed_path()
        parameters  = parsed_path['parameters']
        if parameters is None: return default
        for i in parameters:
            if i[0] == key:
                try:
                    return i[1]
                except:
                    return value
        return default

    
    def parameter_values(self,key,default=1):
        parsed_path = self.get_parsed_path()
        parameters  = parsed_path['parameters']
        l = []
        if parameters is None: return default
        for i in parameters:
            if i[0] == key:
                try:
                    l.append(i[1])
                except:
                    l.append(default)
        return l


    def parameter_args(self,defaults):
        param = {}
        if defaults is not None:
            for k in defaults:
                #log(LOG_DEBUG, "k: '{}' of: {}".format(k,defaults))
                default = defaults[k]
                value   = self.parameter_value(k,default);
                if MANDATORY == value:
                    raise ValueError("no parameter '{}'".format(k))
                param[k] = value
        return param 
        
                              
    def handleRequestRequest(self,path,params=None):
        req = path[9:]; # cut of leading /request/
        req = req[0:len(req)-5]; # cut of tailing .json
        project = self.server.project
        try:
            param = self.parameter_args(params)
            #print("req: '{}, param: {}'".format(req,param))
            if 'config' == req:
                data = { 'result': 0, 'config': project.getConfig() }
            else:
                data = project.room.doRequest(req,param)
        except ValueError as err:
            data = { 'result': 1, 'error': str(err) }
        content = json.dumps(data,cls=JSONEncoder)
        contentType = 'application/json'
        return ( content, contentType )


    def do_GET(self):
        project     = self.server.project
        encoding    = 'utf-8'
        status      = 200
        content     = None
        contentType = None #'text/html'
        parsed_path = self.get_parsed_path()
        path = parsed_path['path']
        #print( "parsed_path: "+str(parsed_path))
        if '/' == path: path = '/index.html'
        if '/request/config.json' == path:
            ( content, contentType ) = self.handleRequestRequest(path)
        elif '/request/window/config.json' == path:
            ( content, contentType ) = self.handleRequestRequest(path,{'win_id':None})
        elif '/request/window/state.json' == path:
            ( content, contentType ) = self.handleRequestRequest(path,{'win_id':None})
        elif '/request/room/action.json' == path:
            ( content, contentType ) = self.handleRequestRequest(path,{'action':MANDATORY})
        elif '/request/sr_state.json' == path:
            ( content, contentType ) = self.handleRequestRequest(path)
        elif '/request/sr/action.json' == path:
            room = project.room
            data = {}
            try:
                args = self.parameter_args({'action':MANDATORY,'bit':MANDATORY})
                data['result'] = 0
                if 'toggle_bit' == args['action']:
                    room.pushEvent( SR_Event(room,args) )
                else:
                    data['error' ] = "unknown action: '{}'".format(action)
                    data['result'] = 1
            except ValueError as e:
                data['result'] = 1
                data['error' ] = "error: '{}'".format(e)
            content = json.dumps(data,cls=JSONEncoder)
            contentType = 'application/json'
        elif '/request/window/action.json' == path:
            room = project.room
            data = {}
            try:
                args = self.parameter_args({'action':MANDATORY,'win_id':MANDATORY,'motor':MANDATORY,'dir':None,'pos':None,'angle':None,'stop':None})
                data['result'] = 0
                if 'set_position' == args['action']:
                    if args['angle'] is     None and args['motor'] == 'blind': raise ValueError("parameter 'angle' missing")
                    if args['pos']   is     None: raise ValueError("parameter 'pos' missing"  )
                    if args['dir']   is not None: raise ValueError("parameter 'dir' specified")
                    room.pushEvent( SetPositionEvent(room,args) )
                elif 'drive_direction' == args['action']:
                    if args['stop']  is not None:
                        for i in [ 'dir', 'pos', 'angle' ]:
                            if args[i]   is not None: raise ValueError("parameter '{}' specified".format(i) )
                    else:
                        if args['dir']   is     None: raise ValueError("parameter 'dir' missing"    )
                        if args['pos']   is not None: raise ValueError("parameter 'pos' specified"  )
                        if args['angle'] is not None: raise ValueError("parameter 'angle' specified")
                    room.pushEvent( DriveDirectionEvent(room,args) )
                else:
                    data['error' ] = "unknown action: '{}'".format(action)
                    data['result'] = 1
            except ValueError as e:
                data['result'] = 1
                data['error' ] = "error: '{}'".format(e)
            content = json.dumps(data,cls=JSONEncoder)
            contentType = 'application/json'
        elif '/request/sr_action.json' == path:
            room = project.room
            data = {}
            try:
                args = self.parameter_args({'action':MANDATORY,'bit':MANDATORY})
                data['result'] = 0
                if 'toggle' == args['action']:
                    if args['bit'] is None: raise ValueError("parameter 'bit' missing")
                    room.pushEvent( BitEvent(room,args) )
                else:
                    data['error' ] = "unknown action: '{}'".format(action)
                    data['result'] = 1
            except ValueError as e:
                data['result'] = 1
                data['error' ] = "error: '{}'".format(e)
            content = json.dumps(data,cls=JSONEncoder)
            contentType = 'application/json'
        elif '/' == path:
            content = '<html><head><meta http-equiv="Content-Type" content="text/html; charset=utf-8"/><title>Room Control</title></head><body>Room Control</body></html>'
        elif '/test.txt' == path:
            content  = 'Hello, world!\n\n'
            content += 'requestline:      '+self.requestline     +'\n'
            content += 'path:             '+self.path            +'\n'
            content += 'address_string(): '+self.address_string()+'\n\n'

            content += 'query_string:          '+str(parsed_path['query_string'] )+'\n'
            content += 'parameters:            '+str(parsed_path['parameters'  ] )+'\n'
            content += "parameter_value('v'):  "+str(self.parameter_value('v') )+'\n'
            content += "parameter_values('v'): "+str(self.parameter_values('v'))+'\n'
        elif not len(path) or path[0] != '/' :
            status = 404
            content = "illegal request: GET '"+self.path+"', '"+str(full_path)+"'"
        else:
            full_path = project.base_dir+'/web'+path
            try:
                file = open(full_path, 'rb')
            except FileNotFoundError as e:
                log( LOG_ERROR, str(e) )
                status  = 404
                content = "unknown request: GET '"+self.path+"', '"+str(full_path)+"' is not readable"
            else:
                encoding  = None
                chunksize = 4096
                content   = bytearray(b'')
                chunk = file.read(chunksize)
                while len(chunk):
                    content += chunk
                    chunk = file.read(chunksize)
                print("full_path: '{}'".format(full_path))
                #if re.search('g',full_path):
                if svgExtPat.search(full_path):
                    print("handle SVG")
                    content = handleContentSVG(content)
                    #contentType = 'application/svg'
                else:
                    print("no SVG: '{}'".format(full_path))

        self.send_response(status)
        if contentType is not None: self.send_header('Content-Type', contentType);
        self.end_headers()
        self.wfile.write( bytes(content,encoding) if encoding else content )

        
    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        body = self.rfile.read(content_length)
        self.send_response(200)
        self.end_headers()
        response = BytesIO()
        response.write(b'This is POST request. ')
        response.write(b'Received: ')
        response.write(body)
        self.wfile.write(response.getvalue())



bindPat = re.compile(r'^(tls:)?(.*):(\d+)$' )

class CommServer ( ServerBaseClass ):
    """
    This class handles the communication with one socket. Multiple CommServer objects may be used to open sockets of different ip addresses, ports and protocols.
    """
    __pydoc__ = ['__init__']
    
    def __init__(self,project,bind):
        """
        project  is project object to do communication for
        bind     is the socket sqpecification to bind to examples:
                 "tls::443"           to bind to port  443 to all addresses using tls
                 ":80"                to bind to port   80 to all addresses without tls
                 "tls:127.0.0.1:8443" to bind to port 8443 to localhost using tsl
                 ipv6 is not yet implemented
        """
        self.project = project
        
        #( address, port ) = bind.split(':')
        match = bindPat.match(bind)
        if not match:
            raise ValueError("illegal bind address: '{}'".format(bind))
        tls     = match.group(1) is not None
        address = match.group(2)
        port    = int(match.group(3))
        print( "address: '{}', port: {}, tls: ".format(address, port, tls))

        super(ServerBaseClass, self).__init__( (address,port), RequestHandler )
               
        if tls:
            cert_path = project.cert_path
            if cert_path is None:
                raise ValueError("cert_path not specified for {}".format(bind))
            if cert_path[0] != '/':
                cert_path = project.base_dir + '/' + cert_path
            print( "using cert_path: '{}'".format(cert_path) )
            context = ssl.create_default_context(purpose=ssl.Purpose.CLIENT_AUTH, cafile=None, capath=None, cadata=None)
            context.load_cert_chain(cert_path)            
            self.socket = context.wrap_socket( self.socket, server_side=True )

        #mimetypes.init()      
        #print( 'mimetypes: '+str(mimetypes.types_map) )
        #self.extensions_map = mimetypes.types_map


    
class Project:

    def __init__ (self,config):
        self.base_dir  = config.get( 'project.directory'         )
        self.domain    = config.get( 'web.domain',   'localhost' )
        self.directory = config.get( 'web.directory','web'       )
        self.cert_path = config.get( 'web.cert_path' )
        self.binds     = config.get( 'web.bind',     'localhost:8080' )

        if self.base_dir is None:
            raise ValueError('project.directory is not configured')
        # todo: check base_dir for readable directory
        
        if self.directory is None:
            raise ValueError('project.directory is not configured')

        self.servers = [ CommServer(self,bind) for bind in self.binds.split(',') ]

        self.room = Room(config.subConfig('room'))

        
    def serve_forever(self):
        _thread.start_new_thread(self.room.serve_forever,())

        for server in self.servers[1:]:
            _thread.start_new_thread(server.serve_forever,())
        self.servers[0].serve_forever()

        
    def getConfig(self):
        return { 'web': { 'base_dir':  self.base_dir,
                          'directory': self.directory,
                          'domain':    self.domain,
                          'cert_path': self.cert_path,
                          'binds':     self.binds,
                          
                        },
                 'version': RequestHandler.server_version,
                 'room': self.room.getConfig(),
               }



def printEnvVars(name,value): # print yaml data as environment variables
    if type(value) == str: # types.StringType:
        print(name+'="'+value.replace('$','\\$')+'"')
    elif type(value) == dict: # types.DictType:
        #prefix = if name is None : '' else name+'.'
        prefix = '' if name is None else name+'__'
        for key in value:
            printEnvVars(prefix+key,value[key])
        

if __name__ == '__main__':

    try:
        ( prog, args )    = ( sys.argv[0], sys.argv[1:] )
        ( cmd, confPath ) = args
        if len(args) != 2 or ( cmd != 'run' and cmd != 'config' and cmd != 'env' ):
            raise ValueError('invocation arguments')
    except Exception as e:
        log( LOG_ERROR, "Illegal arguments: "+str(' '.join(sys.argv[1:]))+"\n" )
        log( LOG_ERROR, "call: "+prog+" run path_to_config.yaml" )
        sys.exit(-1)
    
    if cmd == 'run':
        #try:
            config_dict = yaml.load(open(confPath, 'r')) 
            print( "config: "+str(config_dict) )
            config = Config(config_dict)

            server = Project(config)          
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass
        #except Exception as e:
        #    log( LOG_ERROR, str(e) )
        #    sys.exit(-1)
        
    elif cmd == 'env':
        config = yaml.load(open(confPath, 'r'))
        print( "# config: "+str(config) )
        printEnvVars( None, config )

else:
    #print("__name__ = '{}'".format(__qname__))
    pass


if 'pydoc' in sys.modules:
    def __instrument_pydoc__():
        # suppress __all__, __dict__, and __weakref__  and handle __pydoc__
        pydoc = sys.modules['pydoc']
        o_visiblename = pydoc.visiblename
        def n_visiblename(name, all=None, obj=None):
            """Decide whether to show documentation on a variable."""
            # Certain special names are redundant or internal.
            print("visiblename('{}',{},{})".format(name,all,str(obj)))
            if name in {'__all__','__author__','__builtins__','__credits__','__date__',
                        '__cached__','__doc__','__loader__','__name__','__package__','__spec__','__file__',
                        '__pydoc__','__no_pydoc__','__instrument_pydoc__','__dict__','__weakref__' }:
                return False
            if hasattr(obj, '__pydoc__') and not name in obj.__pydoc__:
                return False
            if hasattr(obj, '__no_pydoc__') and name in obj.__no_pydoc__:
                return False
            return o_visiblename(name,all,obj)
        pydoc.visiblename = n_visiblename

        # "class MyClass(builtins.object)" => "class MyClass"
        TextDoc = pydoc.TextDoc
        o_docclass = TextDoc.docclass
        emptyBasesPat = re.compile(r'^(\s*class\s+.+)(\(builtins\.object\))(\s*)') # may contain esc sequences
        emptyBases = lambda m : m.group(1)+m.group(3)
        def n_docclass(self, object, name=None, mod=None, *ignored):
            #bases = object.__bases__
            #object.__bases__ = None
            ret = o_docclass(self,object,name,mod,*ignored)
            #object.__bases__ = bases
            ret = emptyBasesPat.sub(emptyBases,ret)
            return ret    
        TextDoc.docclass = n_docclass
        
    __instrument_pydoc__()
