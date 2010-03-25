from iocbuilder import Device, AutoSubstitution, SetSimulation
from iocbuilder.arginfo import *

from iocbuilder.modules.areaDetector import AreaDetector, ADBase, simDetector
from iocbuilder.modules.dc1394 import Dc1394

class _firewireDCAM(AutoSubstitution):
    TemplateFile="firewireDCAM.db"

class firewireDCAM(Device):
    '''Creates a firewire DCAM camera'''
    Dependencies = (Dc1394,)
    
    def __init__(self, ID, MEMORY, BUFFERS = 16, SPEED = 800, COLOUR = 0, **args):  
        # Pass the arguments to the relevant templates
        self.template = _firewireDCAM(**filter_dict(args, _firewireDCAM.ArgInfo.Names()))
        self.base = ADBase(**filter_dict(args, ADBase.ArgInfo.Names()))        
        # Init the superclass
        self.__super.__init__()
        # Store the args
        self.__dict__.update(self.base.args)        
        self.__dict__.update(self.template.args)
        self.__dict__.update(locals())

    # __init__ arguments        
    ArgInfo = _firewireDCAM.ArgInfo + \
        ADBase.ArgInfo.filtered(without = _firewireDCAM.ArgInfo.Names()) + \
               makeArgInfo(__init__,
        ID     = Simple('Cam ID with 0x prefix', str),
        MEMORY = Simple(
            'Max memory to allocate, should be maxw*maxh*nbuffer for driver'
            ' and all attached plugins', int),        
        BUFFERS = Simple(
            'Maximum number of NDArray buffers to be created for plugin'
            ' callbacks', int),        
        SPEED  = Choice('Bus speed', [400, 800]),
        COLOUR = Enum  ('Colour mode', ['B+W', 'Col'])) 
        
    # Device attributes
    LibFileList = ['firewireDCAM']
    DbdFileList = ['firewireDCAM'] 

    def InitialiseOnce(self):
        print '# Scan the firewire bus for cameras'
        print 'FDC_InitBus()'

    def Initialise(self):
        # ADBase has stored these for us
        print 'FDC_Config("%(PORT)s", %(ID)s, %(SPEED)d, %(BUFFERS)d,' \
            ' %(MEMORY)d, %(COLOUR)d)' % self.__dict__
        
# just use a simDetector for simulations
def firewireDCAM_sim(**args):
    return simDetector(
        WIDTH = 1024, HEIGHT = 768,
        **filter_dict(args, simDetector.ArgInfo.Names()))
SetSimulation(firewireDCAM, firewireDCAM_sim)
