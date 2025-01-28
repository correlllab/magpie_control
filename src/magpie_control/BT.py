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
# from magpie_control.ur5 import UR5_Interface

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
            super().__init__( name = name )
        self.ctrl  = ctrl
        self.msg   = ""
        self.logger.debug( f"[{self.name}::__init__()]" )
        if self.ctrl is None:
            self.logger.warning( f"{self.name} is NOT conntected to a robot controller!" )
        self.count = 0
        

    def setup(self):
        """ Virtual setup for base class """
        self.logger.debug( f"[{self.name}::setup()]" )          
        
        
    def initialise( self ):
        """ Run first time behaviour is ticked or not RUNNING.  Will be run again after SUCCESS/FAILURE. """
        self.status = Status.RUNNING # Do not let the behavior idle in INVALID
        self.logger.debug( f"[{self.name}::initialise()]" ) 
        self.count = 0         

        
    def terminate( self, new_status ):
        """ Log how the behavior terminated """
        self.status = new_status
        self.logger.debug( f"[{self.name}::terminate().terminate()][{self.status}->{new_status}]" )
        
        
    def update( self ):
        """ Return status """
        return self.status
    

    def stall( self, Nwait ):
        """ Run at least `Nwait` ticks """
        rtnStat = Status.INVALID
        if self.count < Nwait:
            rtnStat = Status.RUNNING
        else:
            rtnStat = Status.SUCCESS
        self.count += 1
        return rtnStat
    
    
    
########## CONSTANTS & COMPONENTS ##################################################################

### Init data structs & Keys ###
MP2BB          = dict()  # Hack the BB object into the built-in namespace  # Hack the BB object into the built-in namespace
_PAUSE_KEY     = "robotPaused"
PROTO_PICK_ROT = np.array( [[ 0.0,  1.0,  0.0, ],
                            [ 1.0,  0.0,  0.0, ],
                            [ 0.0,  0.0, -1.0, ]] )

### Set important BB items ###
MP2BB[ _PAUSE_KEY ] = False



########## CONTROL FLOW BEHAVIORS ##################################################################

class SetBBVar( BasicBehavior ):
    """ Return `SUCCESS` on a periodic basis """

    def __init__( self, k, v, name = None, ctrl = None ):
        """ Set the blackboad value """
        super().__init__( name, ctrl )
        self.key = k
        self.val = v


    def initialise( self ):
        """ Actually Move """
        super().initialise()
        MP2BB[ self.key ] = self.val


    def update( self ):
        """ Return SUCCESS if the value was correctly set """
        if MP2BB[ self.key ] == self.val:
            self.status = Status.SUCCESS 
        else:
            self.status = Status.RUNNING
        return self.status



class CycleTimer( BasicBehavior ):
    """ Return `SUCCESS` on a periodic basis """

    def __init__( self, period_s, name = None, ctrl = None ):
        """ Set the period """
        super().__init__( name, ctrl )
        self.period = period_s
        self.tLast  = now()


    def update( self ):
        """ Return SUCCESS if the period expired """
        if (now() - self.tLast) >= self.period:
            self.tLast  = now()
            self.status = Status.SUCCESS 
        else:
            self.status = Status.RUNNING
        return self.status



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


class Pause_Robot( SetBBVar ):
    """ Halt robot motion """
    
    def __init__( name = None, ctrl = None ):
        """ Set up """
        super().__init__( _PAUSE_KEY, True, name, ctrl )


class Resume_Robot( SetBBVar ):
    """ Resume robot motion """
        
    def __init__( name = None, ctrl = None ):
        """ Set up """
        super().__init__( _PAUSE_KEY, False, name, ctrl )
    
    
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


    def pause( self ):
        """ Arm will be paused next tick, Can only be done while RUNNING """
        self.paused = True

        
    def resume( self ):
        """ Arm will resume motion next tick, Can only be done while RUNNING """
        self.paused = False


    def check_pause_bb( self ):
        """ Use a blackboard (dict) to `pause()`/`resume()` """
        if self.useBB:
            if MP2BB[ _PAUSE_KEY ]:
                self.pause()
            else:
                self.resume()

        
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
        # Fetch command, if applicable
        self.check_pause_bb()
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
    
    def __init__( self, endPose, zSAFE=0.150, name="Jog_Safe", 
                  ctrl  = None ):
        """Construct the subtree"""
        super().__init__( name = name, memory = True )
        
        # Init #
        self.zSAFE = max( zSAFE, endPose[2,3] ) # Eliminate (some) silly vertical movements
        self.ctrl  = ctrl
        
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
        
        
########## MANIPULATION BEHAVIORS ##################################################################


class Pick_at_Pose( Sequence ):
    """ Grasp at a target pose (Robot Frame) while traversing at a safe altitude """
    # NOTE: This behavior should not, on its own, assume any gripper state

    def __init__( self, target, zSAFE = 0.150, preGraspW_m = None, graspWdth_m = None, name = "Pick_at_Pose", ctrl = None ):
        """Construct the subtree"""
        super().__init__( name = name, memory = 1 )
        self.ctrl = ctrl
        
        # 1. Open the gripper
        if preGraspW_m is None:
            self.add_child(  Open_Gripper( name = "Open", ctrl = ctrl )  )
        else:
            self.add_child(  Set_Gripper( preGraspW_m, name = "Open", ctrl = ctrl )  )
        # 2. Jog to the target
        self.add_child(  Jog_Safe( target, zSAFE = zSAFE, name = "Jog to Grasp Pose", ctrl = ctrl )  )
        # 1. Close the gripper
        if graspWdth_m is None:
            self.add_child(  Close_Gripper( name = "Close", ctrl = ctrl )  )
        else:
            self.add_child(  Set_Gripper( graspWdth_m, name = "Close", ctrl = ctrl )  )
            
            
class Place_at_Pose( Sequence ):
    """ Grasp at a target pose (Robot Frame) while traversing at a safe altitude """
    # NOTE: This behavior should not, on its own, assume any gripper state

    def __init__( self, target, zSAFE = 0.150, postGraspW_m = None, name = "Place_at_Pose", ctrl = None ):
        """Construct the subtree"""
        super().__init__( name = name, memory = 1 )
        self.ctrl = ctrl
        # 2. Jog to the target
        self.add_child(  Jog_Safe( target, zSAFE = zSAFE, name = "Jog to Grasp Pose", ctrl = ctrl )  )
        # 1. Open the gripper
        if postGraspW_m is None:
            self.add_child(  Open_Gripper( name = "Open", ctrl = ctrl )  )
        else:
            self.add_child(  Set_Gripper( postGraspW_m, name = "Open", ctrl = ctrl )  )
            
            
########## EXECUTION ###############################################################################


class HeartRate: 
    """ Sleeps for a time such that the period between calls to sleep results in a frequency not greater than the specified 'Hz' """
    # NOTE: This fulfills a purpose similar to the rospy rate
    
    def __init__( self , Hz ):
        """ Create a rate object with a Do-Not-Exceed frequency in 'Hz' """
        self.period = 1.0 / Hz; # Set the period as the inverse of the frequency , hearbeat will not exceed 'Hz' , but can be lower
        self.last = time.time()
        
    def check_elapsed( self, reset = True ):
        """ Check if the period has elapsed, Optionally `reset` the clock """
        elapsed = time.time() - self.last
        update  = elapsed >= self.period
        if( update and reset ):
            self.last = time.time()
        return update
    
    def sleep( self ):
        """ Sleep for a time so that the frequency is not exceeded """
        elapsed = time.time() - self.last
        if elapsed < self.period:
            time.sleep( self.period - elapsed )
        self.last = time.time()


""" Return a formatted timestamp string, useful for logging and debugging """
def nowTimeStamp(): return datetime.datetime.now().strftime(
    '%Y-%m-%d_%H-%M-%S')  # http://stackoverflow.com/a/5215012/893511



class StopBeetle:
    """Invasive Beetle: Kills (stops) all branches of the tree"""

    def __init__(self, killStatus):
        """Set the status that will be assigned to all branches"""
        self.status = killStatus

    def run(self, behav):
        """Kill all subtrees"""
        for chld in behav.children:
            self.run(chld)
        behav.stop(self.status)


        
def run_BT_until_done(
    rootNode,
    N              = 10000,
    tickPause      =     0.25,
    Nverb          =    50,
    breakOnFailure = True,
    breakOnSuccess = True,
    treeUpdate     = 0,
    failureTree    = 1,
    successTree    = 0,
):
    """Tick root until `maxIter` is reached while printing to terminal"""

    if Nverb:
        print(
            "About to run",
            type( rootNode ),
            "named",
            rootNode.name,
            "at",
            nowTimeStamp(),
            "with",
            1 / tickPause,
            "Hz update frequency ...",
        )

    # 0. Setup
    # rootNode.setup_subtree( childrenFirst = 0 )
    rootNode.setup_with_descendants()
    pacer = HeartRate(Hz=1 / tickPause)  # metronome

    if Nverb:
        print("Running ...\n")

    # 1. Run
    for i in range(1, N + 1):
        try:
            rootNode.tick_once()

            if Nverb > 0 and i % Nverb == 0:
                print("\n--------- Tick {0} ---------\n".format(i))
                print("Root node, Name:", rootNode.name, ", Status:", rootNode.status)
                print("\n")
                if treeUpdate:
                    print(
                        py_trees.display.unicode_tree(root=rootNode, show_status=True)
                    )

            if breakOnFailure and (rootNode.status == Status.FAILURE):
                print("Root node", rootNode.name, "failed!\n")
                if failureTree:
                    print(
                        py_trees.display.unicode_tree(root=rootNode, show_status=True)
                    )
                break
            elif breakOnSuccess and (rootNode.status == Status.SUCCESS):
                print("Root node", rootNode.name, "succeeded!\n")
                if successTree:
                    print(
                        py_trees.display.unicode_tree(root=rootNode, show_status=True)
                    )
                break
            else:
                pacer.sleep()

        except KeyboardInterrupt:
            print("\nSimulation HALTED by user at", nowTimeStamp())
            break

    print("\nRun completed! with status:", rootNode.status, "\n\n")

    insect = StopBeetle(rootNode.status)

    for i in range(3):
        rootNode.visit(insect)  # HACK required coz tree doesn't complete sometimes
        sleep(0.5)

    print("Root node", rootNode.name, "was killed by the running script!")