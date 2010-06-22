from iocbuilder import Device, AutoSubstitution, SetSimulation
from iocbuilder.arginfo import *

from iocbuilder.modules.areaDetector import AreaDetector, _ADBase, ADBase, simDetector
from iocbuilder.modules.dc1394 import Dc1394

class _firewireDCAM(AutoSubstitution):
    TemplateFile="firewireDCAM.db"
    SubstitutionOverwrites = [_ADBase]

class firewireDCAM(ADBase):
    '''Creates a firewireDCAM camera areaDetector driver'''
    gda_template = "firewireDCAM"
    Dependencies = (Dc1394,)    
    def __init__(self, ID, SPEED = 800, COLOUR = 0, BUFFERS = 50, MEMORY = -1, **args):
        # Init the superclass
        self.__super.__init__(**filter_dict(args, ADBase.ArgInfo.Names()))
        # Init the firewireDCAM class
        self.template = _firewireDCAM(**filter_dict(args, _firewireDCAM.ArgInfo.Names()))
        # Store the args
        self.__dict__.update(self.template.args)
        self.__dict__.update(locals())

    # __init__ arguments
    ArgInfo = ADBase.ArgInfo + _firewireDCAM.ArgInfo + makeArgInfo(__init__,
        ID     = Simple('Cam ID with 0x prefix', str),
        SPEED  = Choice('Bus speed', [400, 800]),
        COLOUR = Enum  ('Colour mode', ['B+W', 'Col']),
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
        print 'FDC_Config("%(PORT)s", %(ID)s, %(SPEED)d, %(BUFFERS)d,' \
            ' %(MEMORY)d, %(COLOUR)d)' % self.__dict__

def firewireDCAM_sim(**kwargs):
    return simDetector(1024, 768, **kwargs)

SetSimulation(firewireDCAM, firewireDCAM_sim)
