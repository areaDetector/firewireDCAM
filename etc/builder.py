from iocbuilder import Device, AutoSubstitution, SetSimulation
from iocbuilder.arginfo import *

from iocbuilder.modules.areaDetector import AreaDetector, _ADBase, simDetector
from iocbuilder.modules.dc1394 import Dc1394

class _firewireDCAM(AutoSubstitution):
    TemplateFile="firewireDCAM.template"

class firewireDCAM(_ADBase):
    '''Creates a firewireDCAM camera areaDetector driver'''
    _SpecificTemplate = _firewireDCAM
    Dependencies = (Dc1394,)    
    def __init__(self, ID, SPEED = 800, disableScalable = 0, BUFFERS = 50, MEMORY = -1, **args):
        # Init the superclass
        self.__super.__init__(**args)
        # Store the args
        self.__dict__.update(locals())

    # __init__ arguments
    ArgInfo = _ADBase.ArgInfo + _firewireDCAM.ArgInfo + makeArgInfo(__init__,
        ID     = Simple('Cam ID with 0x prefix', str),
        SPEED  = Choice('Bus speed', [400, 800]),
        disableScalable = Enum  ('Disable scalable (format 7) mode', ['No', 'Yes']),
        BUFFERS = Simple('Maximum number of NDArray buffers to be created for '
            'plugin callbacks', int),
        MEMORY  = Simple('Max memory to allocate, should be maxw*maxh*nbuffer '
            'for driver and all attached plugins', int))

    # Device attributes
    LibFileList = ['firewireDCAM']
    DbdFileList = ['firewireDCAM']

    def InitialiseOnce(self):
        print '# Scan the firewire bus for cameras'
        print 'FDC_InitBus()'

    def Initialise(self):
    	print '# FDC_Config(asynPort, cameraID, busSpeed, nBuffers,' \
	            ' maxMemory, disableScalable)'
        print 'FDC_Config("%(PORT)s", %(ID)s, %(SPEED)d, %(BUFFERS)d,' \
            ' %(MEMORY)d, %(disableScalable)d)' % self.__dict__

def firewireDCAM_sim(**kwargs):
    return simDetector(1024, 768, **kwargs)

SetSimulation(firewireDCAM, firewireDCAM_sim)
