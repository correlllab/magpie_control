########## INIT ####################################################################################

### Basic Imports ###
import builtins, datetime, time
from time import sleep
now = time.time

import numpy as np

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence

from magpie_control.poses import pose_error
from magpie_control.ur5 import UR5_Interface

_GRIP_WAIT_S = 1.5
_DUMMYPOSE   = np.eye(4)



########## BASE CLASS ##############################################################################

class BasicBehavior( Behaviour ):
    """ Abstract class for repetitive housekeeping """
    
    def __init__( self, name = None, ctrl = None ):
        """ Set name to the child class name unless otherwise specified """
        if name is None:
            super().__init__( name = str( self.__class__.__name__  ) )
        else:
            super().__init__( name = str( name ) )
        self.ctrl : UR5_Interface = ctrl
        self.msg    = ""
        self.paused = False
        if self.ctrl is None:
            self.logger.warning( f"{self.name} is NOT conntected to a robot controller!" )
        

    def setup( self ):
        """ Virtual setup for base class """
        self.logger.debug( f"[{self.name}::setup()]" )          
        
        
    def initialise( self ):
        """ Run first time behaviour is ticked or not RUNNING.  Will be run again after SUCCESS/FAILURE. """
        self.status = Status.RUNNING # Do not let the behavior idle in INVALID
        self.logger.debug( f"[{self.name}::initialise()]" ) 

        
    def terminate( self, new_status ):
        """ Log how the behavior terminated """
        self.status = new_status
        self.logger.debug( f"[{self.name}::terminate()][{self.status}->{new_status}]" )
        
        
    def update( self ):
        """ Return status """
        raise NotImplementedError( f"{self.__class__.__name__} REQUIRES an `update()` function!" )
    

    def pause( self ):
        """ Return status """
        self.paused = True


    def resume( self ):
        """ Return status """
        self.paused = False
    


########## MOVEMENT BEHAVIORS ######################################################################

### Constants ###
LIBBT_TS_S       = 0.25
DEFAULT_TRAN_ERR = 0.002
DEFAULT_ORNT_ERR = 3*np.pi/180.0

##### Move_Q #####################################


class Move_Q( BasicBehavior ):
    """ Move the joint config `qPos` """
    
    def __init__( self, qPos, name = None, ctrl = None, rotSpeed = 1.05, rotAccel = 1.4, asynch = True ):
        """ Set the target """
        # NOTE: Asynchronous motion is closest to the Behavior Tree paradigm, Avoid blocking!
        super().__init__( name, ctrl )
        self.qPos     = qPos
        self.rotSpeed = rotSpeed
        self.rotAccel = rotAccel
        self.asynch   = asynch
    
    
    def initialise( self ):
        """ Actually Move """
        super().initialise()
        self.ctrl.moveJ( self.qPos, self.rotSpeed, self.rotAccel, self.asynch )
    
    
    def update( self ):
        """ Return SUCCESS if the target reached """
        if self.ctrl.p_moving():
            self.status = Status.RUNNING
        else:
            error = np.subtract( self.qPos, self.ctrl.get_joint_angles() )
            error = error.dot( error )
            if( error > 0.1 ):
                self.status = Status.FAILURE
            else:
                self.status = Status.SUCCESS 
        return self.status
    


##### Move_Arm ###################################
    
    
class Move_Arm( BasicBehavior ):
    """ Move linearly in task space to the designated pose """
    
    def __init__( self, pose, name = None, ctrl = None, linSpeed = 0.25, linAccel = 0.5, asynch = True ):
        """ Set the target """
        # NOTE: Asynchronous motion is closest to the Behavior Tree paradigm, Avoid blocking!
        super().__init__( name, ctrl )
        self.pose     = pose
        self.linSpeed = linSpeed
        self.linAccel = linAccel
        self.asynch   = asynch
        
        
    def initialise( self ):
        """ Actually Move """
        super().initialise()
        self.ctrl.moveL( self.pose, self.linSpeed, self.linAccel, self.asynch )
        
        
    def update( self ):
        """ Return true if the target reached """
        if self.ctrl.p_moving():
            self.status = Status.RUNNING
        else:
            pM = self.ctrl.get_tcp_pose()
            pD = self.pose
            [errT, errO] = pose_error( pM, pD )
            if (errT <= DEFAULT_TRAN_ERR) and (errO <= DEFAULT_ORNT_ERR):
                self.status = Status.SUCCESS
            else:
                print( self.name, ", POSE ERROR:", [errT, errO] )
                self.status = Status.FAILURE
        return self.status
    


##### Move_Arm_w_Pause ###########################
    
    
class Move_Arm_w_Pause( BasicBehavior ):
    """ A version of `Move_Arm` that can be halted """
    
    def __init__( self, pose, name = None, ctrl = None, linSpeed = 0.25, linAccel = 0.5, useBB = False ):
        """ Set the target """
        # NOTE: Asynchronous motion is REQUIRED by this Behavior!
        super().__init__( name, ctrl )
        self.pose     = pose
        self.linSpeed = linSpeed
        self.linAccel = linAccel
        self.asynch   = True # This MUST be true
        self.paused   = False # Is the motion paused?
        self.nextMove = False # Will the motion be resumed?
        self.mvPaus_s = 0.25
        self.useBB    = useBB

        
    def initialise( self ):
        """ Actually Move """
        super().initialise()
        self.check_pause_bb()
        if self.paused:
            self.nextMove = True
        else:
            self.ctrl.moveL( self.pose, self.linSpeed, self.linAccel, self.asynch )
            sleep( self.mvPaus_s )
        
        
    def update( self ):
        """ Return true if the target reached, Handle `pause()`/`resume()` in between ticks """
        ## Running State ##
        if not self.paused:
            # Handle resume
            if self.nextMove:
                self.ctrl.moveL( self.pose, self.linSpeed, self.linAccel, self.asynch )
                sleep( self.mvPaus_s )
                self.status   = Status.RUNNING
                self.nextMove = False
            # Else motion is normal
            else:
                if self.ctrl.p_moving():
                    self.status = Status.RUNNING
                else:
                    pM = self.ctrl.get_tcp_pose()
                    pD = self.pose
                    [errT, errO] = pose_error( pM, pD )
                    if (errT <= DEFAULT_TRAN_ERR) and (errO <= DEFAULT_ORNT_ERR):
                        self.status = Status.SUCCESS
                    else:
                        print( self.name, ", POSE ERROR:", [errT, errO] )
                        self.status = Status.FAILURE
        ## Paused State ##
        elif self.paused:
            # Handle robot moving at beginning of `pause()`
            if self.ctrl.p_moving():
                self.ctrl.halt()
            # Handle state
            if self.status not in ( Status.SUCCESS, Status.FAILURE, ):
                self.status   = Status.RUNNING
                self.nextMove = True

        return self.status

    
    
##### Open_Hand ##################################
    
    
class Open_Gripper( BasicBehavior ):
    """ Open fingers to max extent """
    
    def __init__( self, name = None, ctrl = None ):
        """ Set the target """
        super().__init__( name, ctrl )
        self.wait_s = _GRIP_WAIT_S
        
        
    def initialise( self ):
        """ Actually Move """
        super().initialise()
        self.ctrl.open_gripper()
        sleep( self.wait_s )
        
        
    def update( self ):
        """ Return true if the target reached """
        self.status = Status.SUCCESS
        return self.status
        
        

##### Set_Fingers ##################################
    
    
class Set_Gripper( BasicBehavior ):
    """ Open fingers to max extent """
    
    def __init__( self, width_m, name = None, ctrl = None ):
        """ Set the target """
        super().__init__( name, ctrl )
        self.width_m = width_m
        self.wait_s = _GRIP_WAIT_S
        
        
    def initialise( self ):
        """ Actually Move """
        super().initialise()
        self.ctrl.set_gripper( self.width_m )
        sleep( self.wait_s )
        
    
    def update( self ):
        """ Return true if the target reached """
        self.status = Status.SUCCESS
        return self.status
    

    
##### Close_Hand ##################################
    
    
class Close_Gripper( BasicBehavior ):
    """ Close fingers completely """
    
    def __init__( self, name = None, ctrl = None ):
        """ Set the target """
        super().__init__( name, ctrl )
        self.wait_s = _GRIP_WAIT_S
        
        
    def initialise( self ):
        """ Actually Move """
        super().initialise()
        self.ctrl.close_gripper()
        sleep( self.wait_s )
        
        
    def update( self ):
        """ Return true if the target reached """
        self.status = Status.SUCCESS
        return self.status
    

##### Gripper_Aperture_OK ##################################

class Gripper_Aperture_OK( BasicBehavior ):
    """ Return SUCCESS if gripper separation (both, [m]) is within margin of target """
    
    def __init__( self, width_m, margin_m = None, name = None, ctrl = None ):
        """ Set the target """
        super().__init__( name, ctrl )
        self.width  = width_m
        self.margin = margin_m if (margin_m is not None) else (width_m * 0.25)

    def update( self ):
        """ Return true if the target maintained """
        # print( f"\nGripper Sep: {self.ctrl.get_gripper_sep()}\n" )
        sep = self.ctrl.get_gripper_sep()
        print( f"Gripper Width: {sep}" )
        if np.abs( sep - self.width ) <= self.margin:
            self.status = Status.SUCCESS
        else:
            self.status = Status.FAILURE
        return self.status
    
    

##### Jog_Safe ###################################

class Jog_Safe( Sequence ):
    """ Move to a target by traversing at a safe altitude """
    # NOTE: This behavior should not, on its own, assume any gripper state
    
    def __init__( self, endPose : np.ndarray, zSAFE=0.150, name="Jog_Safe", 
                  ctrl  = None ):
        """Construct the subtree"""
        super().__init__( name = name, memory = True )
        
        # Init #
        self.zSAFE = max( zSAFE, endPose[2,3] ) # Eliminate (some) silly vertical movements
        self.ctrl : UR5_Interface = ctrl
        
        # Poses to be Modified at Ticktime #
        self.targetP = endPose.copy()
        self.pose1up = _DUMMYPOSE.copy()
        self.pose2up = _DUMMYPOSE.copy()
        
        # Behaviors whose poses will be modified #
        self.moveUp = Move_Arm( self.pose1up, ctrl=ctrl )
        self.moveJg = Move_Arm( self.pose2up, ctrl=ctrl )
        self.mvTrgt = Move_Arm( self.targetP, ctrl=ctrl )
        
        
        # 1. Move direcly up from the starting pose
        self.add_child( self.moveUp )
        # 2. Translate to above the target
        self.add_child( self.moveJg )
        # 3. Move to the target pose
        self.add_child( self.mvTrgt )
       
        
    def initialise( self ):
        """
        ( Ticked first time ) or ( ticked not RUNNING ):
        Generate move waypoint, then move with condition
        """
        nowPose = self.ctrl.get_tcp_pose()
        
        self.pose1up = nowPose.copy()
        self.pose1up[2, 3] = self.zSAFE

        self.pose2up = self.targetP.copy()
        self.pose2up[2, 3] = self.zSAFE

        self.moveUp.pose = self.pose1up.copy()
        self.moveJg.pose = self.pose2up.copy()
        self.mvTrgt.pose = self.targetP.copy()

        
        
########## EXECUTION ###############################################################################


def pass_msg_up( bt : BasicBehavior, failBelow = False ):
    if bt.parent is not None:
        if bt.status == Status.FAILURE:
            if (bt.parent.status != Status.FAILURE) or (len( bt.parent.msg ) == 0):
                setattr( bt.parent, "msg", bt.msg )
                pass_msg_up( bt.parent, True )
            else:
                pass_msg_up( bt.parent )
        elif failBelow:
            setattr( bt.parent, "msg", bt.msg )
            pass_msg_up( bt.parent, True )


def connect_BT_to_robot( bt : BasicBehavior, robot ):
    """ Assign `robot` controller to `bt` and recursively to all nodes below """
    if hasattr( bt, 'ctrl' ):
        bt.ctrl = robot
    if len( bt.children ):
        for child in bt.children:
            connect_BT_to_robot( child, robot )



class BT_Runner:
    """ Run a BT with checks """

    def __init__( self, root : BasicBehavior, tickHz = 4.0, limit_s = 20.0 ):
        """ Set root node reference and running parameters """
        self.root   = root
        self.status = Status.INVALID
        self.freq   = tickHz
        self.period = 1.0 / tickHz
        self.msg    = ""
        self.Nlim   = int( limit_s * tickHz )
        self.i      = 0
        self.tLast  = now()


    def setup_BT_for_running( self ):
        """ Prep BT for running """
        self.root.setup_with_descendants()


    def display_BT( self ):
        """ Draw the BT along with the status of all the nodes """
        print( py_trees.display.unicode_tree( root = self.root, show_status = True ) )


    def p_ended( self ):
        """ Has the BT ended? """
        return self.status in ( Status.FAILURE, Status.SUCCESS )
    

    def set_fail( self, msg = "DEFAULT MSG: STOPPED" ):
        """ Handle external signals to halt BT execution """
        self.status = Status.FAILURE
        self.msg    = msg


    def halt( self, killStatus : Status ):
        """Kill all subtrees"""

        def kill( behav : BasicBehavior, stat ):
            for chld in behav.children:
                kill( chld, stat )
            behav.stop( stat )

        kill( self.root, killStatus )

        
    def pause( self ):
        """Pause all subtrees"""

        def set_pause( behav : BasicBehavior ):
            for chld in behav.children:
                set_pause( chld )
            behav.pause()

        set_pause( self.root )


    def resume( self ):
        """Resume all subtrees"""

        def set_resume( behav : BasicBehavior ):
            for chld in behav.children:
                set_resume( chld )
            behav.resume()

        set_resume( self.root )


    def per_sleep( self ):
        """ Sleep for the remainder of the period """
        # NOTE: Run this AFTER BT and associated work have finished
        tNow = now()
        elap = (tNow - self.tLast)
        if (elap < self.period):
            sleep( self.period - elap )
        self.tLast = now()


    def tick_once( self ):
        """ Run one simulation step """
        ## Advance BT ##
        if not self.p_ended():
            self.root.tick_once()
        self.status = self.root.status
        self.i += 1
        ## Check Conditions ##
        if (self.i >= self.Nlim) and (not self.p_ended()):
            self.set_fail( "BT TIMEOUT" )
        if self.p_ended():
            pass_msg_up( self.root )
            if (len( self.msg ) == 0) and hasattr( self.root, 'msg' ):
                self.msg = self.root.msg
            self.display_BT() 
        return self.root.tip().name