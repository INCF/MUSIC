#
#  This file is part of MUSIC.
#  Copyright (C) 2012 Mikael Djurfeldt
#
#  MUSIC is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
#
#  MUSIC is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

# This API allows for querying the MPI rank and specification of the
# information in MUSIC configuration files

import os

from music.predict_rank import predictRank

# This function now defined in predict_rank.py
#
#def predictRank ():
#    """
#    Returns the predicted MPI rank of this process
#    
#    :rtype: Integer
#    """
#    return 0

CONFIGVARNAME = '_MUSIC_CONFIG_'

OUTPUT = '0'
INPUT = '1'


def launchedByMusic ():
    return CONFIGVARNAME in os.environ


def supersedeArgv (argv):
    """
    Replace argc and argv before MUSIC initialization
    """
    # Should insert escape characters
    args = argv[0]
    for arg in argv[1:]:
        args += ' ' + arg
    os.environ['MUSIC_ARGV'] = args


def postponeSetup ():
    """
    Postpones processing of configuration info until the creation of
    the first port.  Must be called before creation of MUSIC::Setup.
    """
    if not thisApp:
        raise RuntimeError, \
              'must define Application before calling postponeSetup ()'
    os.environ[CONFIGVARNAME] = 'POSTPONE:' + str (thisApp.number)


portCodes = {}
code = 0

def portCode (appName, portName):
    global code
    key = (appName, portName)
    if key in portCodes:
        return portCodes[key]
    portCodes[key] = code
    ans = code
    code += 1
    return ans


class ConnectivityMap (object):
    def __init__ (self):
        self.ports = {}
    
    def register (self, portName, direction, width, *connection):
        if portName in self.ports:
            (direction_, width_, connections) = self.ports[portName]
            #*fixme* error checks here
        else:
            connections = []
            self.ports[portName] = (direction, width, connections)
        connections.append (connection)

    def conf (self):
        conf = str (len (self.ports))
        for portName in self.ports:
            (direction, width, connections) = self.ports[portName]
            conf += ':' + portName + ':' + direction + ':' + width
            conf += ':' + str (len (connections))
            for connection in connections:
                for item in connection:
                    conf += ':' + item
        return conf


configDict = {}
appNumber = 0
rankSum = 0
thisRank = predictRank ()
thisApp = None

class Application (object):
    def __init__ (self, np = None, binary = None, args = None, name = None):
        global appNumber, rankSum, thisApp
        self.name = name
        self.number = appNumber
        appNumber += 1
        self.np = np
        self.leader = rankSum
        rankSum += np
        self.this = False
        if self.leader <= thisRank and thisRank < rankSum:
            self.this = True
            thisApp = self

        self.configDict = {}
        self.connectivityMap = ConnectivityMap ()

        if binary:
            self.define ('binary', binary)
        if args:
            self.define ('args', args)

        applicationMap.register (self)

    def __getitem__ (self, varName):
        if varName in self.configDict:
            return self.configDict[varName]
        return configDict[varName]
    
    def define (self, varName, value):
        """
        Define configuration variable varName to value value.
        """
        self.configDict[varName] = str (value)

    def connect (self, fromPort, toApp, toPort, width):
        """
        Connect fromPort to toPort.
        """
        connections.append ((self, fromPort, toApp, toPort, str (width)))


class ApplicationMap (object):
    def __init__ (self):
        self.applications = []
        self.nameMap = {}

    def register (self, app):
        if not app.name:
            app.name = 'application'
        if app.name in self.nameMap:
            # Adjust the name to become unique
            record = self.nameMap[app.name]
            base, suffix = record
            if base == app.name:
                suffix += 1
                record[1] = suffix
            else:
                base = app.name
                suffix = 0
                self.nameMap[base] = [base, suffix]
            app.name += str (suffix)
        else:
            self.nameMap[app.name] = [app.name, 0]
        self.applications.append (app)
    
    def conf (self):
        conf = str (len (self.applications))
        for app in self.applications:
            conf += ':' + app.name + ':' + str (app.np)
        return conf

applicationMap = ApplicationMap ()


def define (varName, value):
    """
    Define configuration variable varName to value value.
    """
    configDict[varName] = str (value)


# Communication algorithms
    
commAlgorithms =  { 'collective' : 0, 'point-to-point' : 1 }

# Processing methods

procMethods = { 'tree' : 0, 'table' : 1 }


def connect (fromApp, fromPort, toApp, toPort, width,
             commAlgName = 'collective', procMethodName = 'table'):
    """
    Connect fromPort to toPort specifying port width width.
    """
    width = str (width)
    commAlg = commAlgorithms[commAlgName]
    procMethod = procMethods[procMethodName]
    fromApp.connectivityMap.register (fromPort, OUTPUT, width,
                                      toApp.name, toPort,
                                      str (portCode (toApp.name, toPort)),
                                      str (toApp.leader), str (toApp.np),
                                      str (commAlg), str (procMethod))
    toApp.connectivityMap.register (toPort, INPUT, width,
                                    toApp.name, toPort,
                                    str (portCode (toApp.name, toPort)),
                                    str (fromApp.leader), str (fromApp.np),
                                    str (commAlg), str (procMethod))


configured = False

def configure ():
    """
    Configure the MUSIC library using the information provided by
    define and connect.
    """
    conf = thisApp.name \
           + ':' + str (thisApp.number) \
           + ':' + applicationMap.conf () \
           + ':' + thisApp.connectivityMap.conf ()

    configDict.update (thisApp.configDict)

    for key in configDict:
        conf += ':' + key + '=' + configDict[key]

    os.environ[CONFIGVARNAME] = conf

    configured = True

    
def launch ():
    if not configured:
        configure ()
    binary = thisApp['binary']
    os.execvp (binary, [binary] + thisApp['args'].split (' '))
