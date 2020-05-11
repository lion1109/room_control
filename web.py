#!/usr/bin/python3

# HTTP server
from http.server import HTTPServer, BaseHTTPRequestHandler
import ssl

# Raspberry Pi GPIO 
#import RPi.GPIO as GPIO
class GPIO:
    BOARD = 0
    OUT   = 0
    def setmode( pin_name_mode ):
        pass
    def setup( pin, pin_mode ):
        pass


import time
import sys
import yaml
import types

import _thread


class Config:

    def __init__ (self,dict,defaults=None):
        self.dict     = dict
        self.defaults = defaults

        
    def getFromDict (self,dict,name,default=None):
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

    
    def get (self,name,default=None):
        if self.defaults is not None:
            default = self.getFromDict(self.defaults, name, default)
        return self.getFromDict(self.dict, name, default)


    def subConfig(self,name,defaults=None):
        dict = self.get(name)
        return Config(dict,defaults)


class ShiftRegister: # 74HC595

    def __init__(self,pinDATA,pinCLOCK,pinLATCH,pinENABLE,bits=8):
        self.pinDATA   = pinDATA
        self.pinCLOCK  = pinCLOCK
        self.pinLATCH  = pinLATCH
        self.pinENABLE = pinENABLE
        self.bits      = bits


    def shiftBit(self,bit=0):
        GPIO.output(self.pinCLOCK, GPIO.LOW)
        GPIO.output(self.pinDATA,  bit and GPIO.HIGH or GPIO.LOW)
        GPIO.output(self.pinCLOCK, GPIO.HIGH)


    def enable(self,bit=1):
        GPIO.output(self.pinENABLE, bit and GPIO.HIGH or GPIO.LOW)


        
class Window:
        
    def __init__(self,config,motors):
        self.id   = config.get('id')
        self.name = config.get('name')
        #self. = config.get('')


class Event:

    def __init__(self,name,args):
        self.name = name
        self.args = args


    def handle(self,room):
        print( "handle event '{}': args:{}".format(self.name,str(self.args)) )
        pass # todo


class Room:

    def __init__(self,config):
        self.config = config
        self.events = []
        self.eventLock = _thread.allocate_lock()
        self.reqLock   = _thread.allocate_lock()
        self.init_shift_register()
        self.init_windows()
        self.init_dimmer()


    def init_shift_register(self):
        pinDATA   = self.config.get('shift_register.pinDATA')
        pinCLOCK  = self.config.get('shift_register.pinCLOCK')
        pinLATCH  = self.config.get('shift_register.pinLATCH')
        pinENABLE = self.config.get('shift_register.pinENABLE')
        GPIO.setmode(GPIO.BOARD)        # Number GPIOs by its physical location
        GPIO.setup(pinDATA,   GPIO.OUT)
        GPIO.setup(pinCLOCK,  GPIO.OUT)
        GPIO.setup(pinLATCH,  GPIO.OUT)
        GPIO.setup(pinENABLE, GPIO.OUT)
        self.shift_register = ShiftRegister(pinDATA, pinCLOCK, pinLATCH, pinENABLE, 3*8)


    def init_windows(self):
        count    = self.config.get('windows.count')
        defaults = self.config.get('defaults')
        self.windows = {}
        self.motors  = {}
        for i in range(0,count):
            win_config = self.config.subConfig('window_'+str(i), defaults)
            window = Window(win_config,self.motors)
            id = window.id
            if id in self.windows:
                raise ValueError( "window id '{:}' is not unique".format(id) )
            self.windows[id] = window


    def popEvent(self):
        self.eventLock.acquire()
        event = self.events.pop(0)
        self.eventLock.release()
        return event


    def pushEvent(self,event):
        self.eventLock.acquire()
        self.events.append(event)
        self.eventLock.release()

    
    def handleEvents(self):
        while len(self.events):
            event = self.popEvent()
            if event is not None:
                event.handle(self)
                

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
        
        
    def init_dimmer(self):
        pass

    
    def serve_forever(self):
        i = 0
        while True:
            self.handleEvents()
            i += 1
            if not ( i % 10 ): print('room thread')
            time.sleep(0.5)

        
    def getWindows(self):
        # todo
        pass


    def getDimmer(self):
        # todo
        pass


class RequestHandler(BaseHTTPRequestHandler):

    
    #def __init__ (self,server):
    #    self.projectServer = server
    #    super(BaseHTTPRequestHandler,self).__init__()

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


    def parameter_value(self,key,default=1,none=None):
        parsed_path = self.get_parsed_path()
        parameters  = parsed_path['parameters']
        if parameters is None: return none
        for i in parameters:
            if i[0] == key:
                try:
                    return i[1]
                except:
                    return default
        return none

    
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


    def do_GET(self):
        encoding = 'utf-8'
        status   = 200
        parsed_path = self.get_parsed_path()
        path = parsed_path['path']
        print( "parsed_path: "+str(parsed_path))
        if '/' == path: path = '/index.html'
        if '/request/static.txt' == path:
            room = self.server.room
            data = { 'base': room.getState('static',None) }
            content = str(data)
        elif '/request/window_config.txt' == path:
            room = self.server.room
            data = { 'base': room.getState('window_config',{'id':self.parameter_value('id')}) }
            content = str(data)
        elif '/request/window' == path:
            room = self.server.room
            data = {}
            action = int(self.parameter_value('action',0))
            if action > 0 and action <= 6:
                args = { 'id': self.parameter_value('id') }
                room.pushEvent( Event('action',args) )
                data['result'] = 0
            else:
                data['result'] = 1
                data['error' ] = "unknown action: '{}'".format(action)
            content = str(data)
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
        else:
            full_path = self.server.base_dir+'/web'+path
            file = open(full_path, 'rb')
            if file is None:
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

        self.send_response(status)
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


        
class ProjectServer ( HTTPServer ):

    def __init__ (self,config):
        self.config = config

        self.base_dir  = config.get( 'project.directory'         )
        self.domain    = config.get( 'web.domain',   'localhost' )
        self.directory = config.get( 'web.directory','docs'      )
        bind      = config.get( 'web.bind',     'localhost:8080' )
        cert_path = config.get( 'web.cert_path' )
        
        if self.base_dir is None:
            raise ValueError('project.directory is not configured')
        # todo: check base_dir for readable directory
        
        if self.directory is None:
            raise ValueError('project.directory is not configured')

        ( address, port ) = bind.split(':')
        print( "address: '"+str(address)+"' port: '"+str(port)+"'" )

        super(HTTPServer,self).__init__( (address,int(port)), RequestHandler )
        if cert_path is not None:
            if cert_path[0] != '/':
                cert_path = self.base_dir + '/' + cert_path
            # using a SSLContext
            context = ssl.create_default_context(purpose=ssl.Purpose.CLIENT_AUTH, cafile=None, capath=None, cadata=None)
            context.load_cert_chain(cert_path)
            self.socket = context.wrap_socket( self.socket, server_side=True )
            # deprecated without SSLContext
            #self.socket = ssl.wrap_socket( self.socket, certfile=cert_path, server_side=True)

            
    def set_room (self,room):
        self.room = room



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
        print( "Illegal arguments: "+str(' '.join(sys.argv[1:]))+"\n" )
        print( "call: "+prog+" run path_to_config.yaml" )
        sys.exit(-1)
    
    if cmd == 'run':
        config_dict = yaml.load(open(confPath, 'r')) 
        print( "config: "+str(config_dict) )
        config = Config(config_dict)
        room = Room(config.subConfig('room'))
        _thread.start_new_thread(room.serve_forever,())
        server = ProjectServer(config)
        server.set_room(room)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
    elif cmd == 'config':
        config = yaml.load(open(confPath, 'r'))
        print( "# config: "+str(config) )
        printEnvVars( None, config )
    elif cmd == 'env':
        config = yaml.load(open(confPath, 'r'))
        print( "# config: "+str(config) )
        printEnvVars( None, config )
