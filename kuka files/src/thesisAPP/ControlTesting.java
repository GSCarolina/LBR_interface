package thesisAPP;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
// For hand guiding
import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

import shared.Training;
import thesisUtilities.*;
import thesisMethods.*;


import backgroundTask.StateHelper;

import com.kuka.med.controllerModel.MedController;
import com.kuka.med.cyclicBrakeTest.BrakeTestDiskSpaceState;
import com.kuka.med.cyclicBrakeTest.BrakeTestMonitor;
import com.kuka.med.cyclicBrakeTest.BrakeTestMonitorEvent;
import com.kuka.med.cyclicBrakeTest.BrakeTestOutcome;
import com.kuka.med.cyclicBrakeTest.CyclicMonitoringState;
import com.kuka.med.cyclicBrakeTest.IBrakeTestMonitorListener;
import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.Device;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;

import com.kuka.roboticsAPI.geometricModel.*;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.ErrorHandlingAction;
import com.kuka.roboticsAPI.motionModel.IErrorHandler;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.Spline;

import com.kuka.roboticsAPI.motionModel.controlModeModel.*;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.task.ITaskLogger;


import com.kuka.common.ThreadUtil;
// Joint pointers
import com.kuka.roboticsAPI.deviceModel.JointEnum;
//Accessing to Buttons and LED
import com.kuka.generated.ioAccess.LinkageIOGroup;
import com.kuka.generated.ioAccess.BodyIOGroup;

/**
 * Implementation of a sample application handling the cyclic brake test. As soon as the application is started, the
 * user is asked to manage the initial state of the cyclic monitoring, e.g. start the brake test execution or continue
 * with the application. Then, a continuous motion of the robot between two joint configurations is started to simulate
 * the normal application workflow. The IBrakeTestMonitorListener is used to receive events about state changes of the
 * cyclic brake test monitoring and react accordingly.
 */
public class ControlTesting extends RoboticsAPIApplication implements IBrakeTestMonitorListener
{

    /**
     * User action for interaction with application workflow.
     * */
    private enum UserAction
    {
        NO_ACTION,
        START_BRAKETEST,
        POSTPONE_BRAKETEST,
        EXIT_APPLICATION;
    }

    private MedController _controller;
    
    // The inject allows to create the object on the run, thus their pointers are not nil
    @Inject
    private ITaskLogger _logger;

    @Inject
    private LBR _lbr;	// Robot
	@Inject
	private LinkageIOGroup IOlinkage;	// Linkage
	@Inject
	@Named("Linkage")
	private Tool _linkage;

	@Inject
	private IApplicationControl appControl;
	
    private BrakeTestMonitor _brakeTestMonitor;
    private CyclicMonitoringState _currentStatus = CyclicMonitoringState.OK;
    private boolean _isPostponementAcceptable = false;
    private boolean _isBrakeTestExecutable = true;
    private BrakeTestDiskSpaceState _diskStatus = BrakeTestDiskSpaceState.UNKNOWN;

    private int _responseTime = 60;
    private double _vel = 0.99;
    private JointPosition _jointPos;
    private IMotionContainer _mc;

    /** User commanded action in the application dataflow. */
    private UserAction _action = UserAction.NO_ACTION;

    /** Additional thread to handle BrakeTestMonitorEvents with dialog boxes for user feedback. */
    private ExecutorService _workerToHandleEvent = Executors.newSingleThreadExecutor();

    // ----------------------- UDP ------------------------------------
	private UdpState _udpState;
	private static final int LOCAL_PORT_COPPELIA = 30007;	// This local port is for the simulator
	private ExecutorService _udpStateThread;
    // ----------------------- New: moving stuff around ------------------------------------
    // For movements
 	private ForceSensorData sensorData;
 	public double override;
 	public boolean runOnce;
 	public double zForce;
	/*
	 * 1. Implement Override Control
	 * 2. Execute motion command
	 * 3. Add default motion parameters
	 * 4. Asynchronously execute motion command
	 * 5. Add motion container for monitoring purposes
	 * 6. Wait for motion to finish while doing a second task
	 * 7. Implement IError handler
	 * */
 	 	// Others
 	public ObjectFrame t1;
 	public Frame target;
 	public int targetCnt;
 	public IMotionContainer targetMC;
	private AbstractFrame targetMs;
	// Write to file
	WriteToFile posFile;
	WriteToFile2 forFile;
	private int startFile, endFile, saveFile;
	String message, posName, forName;
	boolean stopRobot;
	int increment = 0;	// To follow the direction of a trajectory
	boolean nullUpdated;

	// Different controllers
	CartesianImpedanceControlMode cartImpCtrlMode;
	private ObjectFrame nullSpaceGoal;
    private JointPosition nullJoints, nullLimit;
    double[] qAL = new double[7];
    // Some UDP action
    private UdpHelper2 _udpHelper;
	private static final int LOCAL_PORT = 30007;	// This local port is for matlab
	private ExecutorService _udpReceiverThread;
	// For trajectories
	// Local variables
	TorqueSensorData measuredData, externalData;
	ForceSensorData measurements;
	Vector force;
	double[] measuredTorques, externalTorques;
	double[] tA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double[] teA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double[] fEE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
	final double LIMIT = 10;	// Constant value
	
	// Local variables: position
	JointPosition qA17, qA17d;
	double[] qA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double[] qAd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double[] dqA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double[] pEE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
	// Local: forces
	final double DELTAQ = Math.toRadians(5);	// Maximum deviation
	final double DELTAC = 30;			// 3cm tolerance distance 
	
	// Using the hand recording approach
	 List<Frame> recordedPath; 
	 boolean isRecorded;
	 
	 
		public void asyncErrorHandler(IApplicationControl applicationControl) {
			// Error handling of asynchronous motion
			IErrorHandler errorHandler = new IErrorHandler() {
				@Override
				public ErrorHandlingAction handleError(Device device, IMotionContainer failedContainer, List<IMotionContainer> canceledContainers) {
					getLogger().warn("Robert: the following motion command has failed: " + failedContainer.getErrorMessage());
					for (int i = 0; i < canceledContainers.size(); i++) {
						getLogger().info("I cannot execute the following commands: " + canceledContainers.get(i).toString());
					}
					return ErrorHandlingAction.Ignore;
				}
			};
			applicationControl.registerMoveAsyncErrorHandler(errorHandler);

		}
    @Override
    public void initialize()
    {
        _controller = (MedController) _lbr.getController();
        _brakeTestMonitor = new BrakeTestMonitor(_controller);
        _brakeTestMonitor.addMonitoringListener(this);

        _responseTime = _brakeTestMonitor.setResponseTime(_responseTime);
        _logger.info("response time changed to " + _responseTime);

        _jointPos = new JointPosition(
                Math.toRadians(30),
                Math.toRadians(30),
                Math.toRadians(0),
                Math.toRadians(0),
                Math.toRadians(0),
                Math.toRadians(0),
                Math.toRadians(0));
        override = 0.5;
        runOnce = false;
    	// Messing around
    	getLogger().info("Start up");
    	//t1 = getFrame("/TrajectoryFrame/TrajP1");
    	t1 = getFrame("/Recording/Center");
    	target = t1.copyWithRedundancy();
    	targetCnt = 0;
    	targetMs = _lbr.getFlange();
    	getLogger().info("d " + targetMs);
    	
    	
    	getLogger().info("Starting file");
    	startFile = 0;
    	saveFile = 1;
    	endFile = 2;
    	stopRobot = false;
    	
    	// Start end-effector
		_linkage.attachTo(_lbr.getFlange());
		_linkage.getLoadData().setMass(0.975);
		
		nullSpaceGoal = getFrame("/Recording/Center");
		nullUpdated = false;
		// The initial nullJoints are the normal initial ones (from deg to radians)
		nullJoints = new JointPosition(0,0,0, Math.toRadians(90),0,Math.toRadians(55),0);
		nullLimit = new JointPosition(Math.toRadians(150),Math.toRadians(100),Math.toRadians(150),Math.toRadians(100),Math.toRadians(150),Math.toRadians(100),Math.toRadians(150));
		qAL[0] = nullLimit.get(JointEnum.J1);
		qAL[1] = nullLimit.get(JointEnum.J2);
		qAL[2] = nullLimit.get(JointEnum.J3);
		qAL[3] = nullLimit.get(JointEnum.J4);
		qAL[4] = nullLimit.get(JointEnum.J5);
		qAL[5] = nullLimit.get(JointEnum.J6);
		qAL[6] = nullLimit.get(JointEnum.J7);
		// recording stuff
		recordedPath = new ArrayList<Frame>();
		isRecorded = false;
		asyncErrorHandler(this.appControl);
    }

    @Override
    public void run()
    {
    	IOlinkage.setLEDLightGreen(true);
    	IOlinkage.setLEDLightRed(false);
    	getLogger().info("Lights");
        // check the current status of the cyclic brake test because an event could have been missed due to
        // late start of the application
        _currentStatus = _brakeTestMonitor.getCurrentState();
        _logger.info("current state: " + _currentStatus);
        _diskStatus = _brakeTestMonitor.checkFreeDiskSpace();
        _logger.info("current disk space state: " + _diskStatus);
        // check whether a postponement is possible
        _isPostponementAcceptable = _brakeTestMonitor.isPostponementPossible();
        _isBrakeTestExecutable = _brakeTestMonitor.isBrakeTestExecutable();

        // handle the brake test execution at start of the application
        handleInitialBraketestExecution();
        
        // Impedance controller 
        cartImpCtrlMode = new CartesianImpedanceControlMode();

        while (!_action.equals(UserAction.EXIT_APPLICATION))
        {
            // ******************* Execution of a brake test commanded via user interaction *******************
            if (_action.equals(UserAction.START_BRAKETEST) && _isBrakeTestExecutable)
            {
                // if another motion is active, cancel it first---
                if (null != _mc && !_mc.isFinished())
                {
                    _mc.cancel();
                }

                // execute the brake test
                BrakeTestOutcome outcome = _brakeTestMonitor.executeBrakeTest(_lbr);
                _logger.info("The overall result of the brake test is '"
                        + outcome.getOverallResult().toString() + "'!");
                if (_brakeTestMonitor.getCurrentState().equals(CyclicMonitoringState.OK))
                {
                    _action = UserAction.NO_ACTION;
                }
            }

            // ******************* Different motions *******************
            if ((null == _mc || _mc.isFinished()) && !runOnce)
            {
            	getApplicationControl().setApplicationOverride(override);
            	getLogger().info("Hello world. Moving to start position.");
            	_lbr.move(ptp(getFrame("/Recording/Center")));
            	// Do once
            	runOnce = true;
            	getLogger().info("Do this once.");
            	startUdpState();
            	getLogger().info("UDP with Coppelia started.");
            	// Start impedance controller
            	//setCartesianImp();
            	setNullSpaceImp();
            }
            if ((null == _mc || _mc.isFinished()) && runOnce)
            {
            	//getLogger().info("wololooooo" + targetCnt);
            	sendToSimulator();
            	ThreadUtil.milliSleep(10);
            	//followTwithNullS();
            	//followTrajectoryPTP_async();
            	//targetCnt++;
            	//updateUdp();
            	
            	// Using hand guiding
            	//recordHandGuiding();
            	//if(isRecorded)
            	//{
            		//getLogger().info("Replicating trajectory");
            		//runRecordedPath();
            	//}
            }
        }
    }
    // ------------------------- Carol testing starts here -------------------------
    public void setCartesianImp()
    {
    	cartImpCtrlMode.parametrize(CartDOF.X,CartDOF.Y).setStiffness(200.0);
    	cartImpCtrlMode.parametrize(CartDOF.Z).setStiffness(100.0);
    	cartImpCtrlMode.parametrize(CartDOF.ROT).setStiffness(300.0);
    	cartImpCtrlMode.parametrize(CartDOF.ALL).setDamping(1.0);
    	getLogger().info("Moving to center point");
    	_lbr.move(ptp(getFrame("/Recording/Center")).setMode(cartImpCtrlMode));
    	ThreadUtil.milliSleep(2000);
    }
    
    public void setNullSpaceImp()
    {
    	cartImpCtrlMode.parametrize(CartDOF.X,CartDOF.Y).setStiffness(200.0);
    	cartImpCtrlMode.parametrize(CartDOF.Z).setStiffness(100.0);
    	cartImpCtrlMode.parametrize(CartDOF.ROT).setStiffness(10.0);
    	cartImpCtrlMode.parametrize(CartDOF.ALL).setDamping(1.0);
    	cartImpCtrlMode.setNullSpaceStiffness(0.3);
    	cartImpCtrlMode.setNullSpaceDamping(0.3);
    	getLogger().info("Moving to center point");
    	_lbr.move(ptp(getFrame("/Recording/Center")).setMode(cartImpCtrlMode));
    	ThreadUtil.milliSleep(2000);
    }
    
    public void runHandGuiding()
    {
    	// local variables
    	double[] tA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	boolean stopRobot = false;
    	// Select trajectory point
    	switch(targetCnt)
    	{
    	case 0:
    		t1 = getFrame("/Recording/Center");
    		_lbr.setESMState("1");
    		getLogger().info("Going to Center point");
    		break;
    	case 2:
    		t1 = getFrame("/Recording/Yleft");
    		_lbr.setESMState("1");
    		getLogger().info("Moving to Y left");
    		break;
    	}
    	target = t1.copyWithRedundancy();
    	
    	if(targetCnt != 1)
    	{
    		targetMC = _lbr.moveAsync(ptp(target).setMode(cartImpCtrlMode)); 
    	}
    	else
    	{
    		_lbr.setESMState("2");
    		getLogger().info("Hand guiding active");
    		targetMC = _lbr.moveAsync(handGuiding());
    	}
    	
    	while(!targetMC.isFinished())
    	{
    		// Forces
			measuredData = _lbr.getMeasuredTorque();
			externalData = _lbr.getExternalTorque();			
			// Joint torques
			tA[0] = externalData.getSingleTorqueValue(JointEnum.J1);
			tA[1] = externalData.getSingleTorqueValue(JointEnum.J2);
			tA[2] = externalData.getSingleTorqueValue(JointEnum.J3);
			tA[3] = externalData.getSingleTorqueValue(JointEnum.J4);
			tA[4] = externalData.getSingleTorqueValue(JointEnum.J5);
			tA[5] = externalData.getSingleTorqueValue(JointEnum.J6);
			tA[6] = externalData.getSingleTorqueValue(JointEnum.J7);
    		// Do this only for normal trajectory points (without handguiding)
			for(int k=0;k<7;k++)
			{
				stopRobot = (stopRobot || (tA[k]>=10))&&(targetCnt==0 || targetCnt==2);
			}
			
			// Stop motion if necessary
			if(stopRobot)
			{
				//getLogger().info("Stopping robot");
				targetMC.cancel();
		    	IOlinkage.setLEDLightGreen(false);
		    	IOlinkage.setLEDLightRed(true);
				break;	// get out of the while loop
			}
			
    		ThreadUtil.milliSleep(100);
    	}
    	
    	if(stopRobot)
    	{
    		ThreadUtil.milliSleep(3000);
    		IOlinkage.setLEDLightGreen(true);
	    	IOlinkage.setLEDLightRed(false);
    	}
    	else
    	{
        	targetCnt++;
        	if(targetCnt>=3)
        	{
        		targetCnt=0;
        	}	
    	}
    	
    }
    
    // Lets get real
    public void recordHandGuiding()
    {
    	// local variables
    	double[] tA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	boolean stopRobot = false;
    	// Select trajectory point
    	switch(targetCnt)
    	{
    	case 0:
    		t1 = getFrame("/Recording/Center");
    		_lbr.setESMState("1");
    		getLogger().info("Going to Center point");
    		break;
    	case 2:
    		t1 = getFrame("/Recording/Yleft");
    		_lbr.setESMState("1");
    		getLogger().info("Moving to Y left");
    		break;
    	}
    	target = t1.copyWithRedundancy();
    	
    	if(targetCnt != 1)
    	{
    		targetMC = _lbr.moveAsync(ptp(target).setMode(cartImpCtrlMode)); 
    	}
    	else
    	{
    		_lbr.setESMState("2");
    		getLogger().info("Hand guiding active");
    		targetMC = _lbr.moveAsync(handGuiding());
    	}
    	
    	while(!targetMC.isFinished())
    	{
    		// Forces
			measuredData = _lbr.getMeasuredTorque();
			externalData = _lbr.getExternalTorque();			
			// Joint torques
			tA[0] = externalData.getSingleTorqueValue(JointEnum.J1);
			tA[1] = externalData.getSingleTorqueValue(JointEnum.J2);
			tA[2] = externalData.getSingleTorqueValue(JointEnum.J3);
			tA[3] = externalData.getSingleTorqueValue(JointEnum.J4);
			tA[4] = externalData.getSingleTorqueValue(JointEnum.J5);
			tA[5] = externalData.getSingleTorqueValue(JointEnum.J6);
			tA[6] = externalData.getSingleTorqueValue(JointEnum.J7);
    		// Do this only for normal trajectory points (without handguiding)
			for(int k=0;k<7;k++)
			{
				stopRobot = (stopRobot || (tA[k]>=10))&&(targetCnt==0 || targetCnt==2);
			}
			
			// Stop motion if necessary
			if(stopRobot)
			{
				//getLogger().info("Stopping robot");
				targetMC.cancel();
		    	IOlinkage.setLEDLightGreen(false);
		    	IOlinkage.setLEDLightRed(true);
				break;	// get out of the while loop
			}
			
			// Recording action!
			if(targetCnt == 1)
			{
				Frame current = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"), getFrame("/Recording/Center"));
				recordedPath.add(current);
			}
			
			
    		ThreadUtil.milliSleep(500);
    	}
    	
    	if(stopRobot)
    	{
    		ThreadUtil.milliSleep(3000);
    		IOlinkage.setLEDLightGreen(true);
	    	IOlinkage.setLEDLightRed(false);
    	}
    	else
    	{
    		// A path has been recorded
    		isRecorded = (targetCnt == 1 && recordedPath.size()>=2);

        	targetCnt++;
        	if(targetCnt>=3)
        	{
        		targetCnt=0;
        	}	
    	}
    	
    }
    
    public void runRecordedPath()
    {
    	int nrPoints = recordedPath.size();
    	getLogger().info("Trajectory points: " + nrPoints);
    	//int curPoint = 0;
    	// Move through the whole set of points
    	for(int i=0; i<nrPoints; i++)
    	{
    		getLogger().info("Moving to recorded point " + i);
    		target = (recordedPath.get(i)).copyWithRedundancy();
    		targetMC = _lbr.moveAsync(ptp(target).setMode(cartImpCtrlMode)); 
    		
    		while(!targetMC.isFinished())
    		{
        		// Forces
    			measuredData = _lbr.getMeasuredTorque();
    			externalData = _lbr.getExternalTorque();			
    			// Joint torques
    			tA[0] = externalData.getSingleTorqueValue(JointEnum.J1);
    			tA[1] = externalData.getSingleTorqueValue(JointEnum.J2);
    			tA[2] = externalData.getSingleTorqueValue(JointEnum.J3);
    			tA[3] = externalData.getSingleTorqueValue(JointEnum.J4);
    			tA[4] = externalData.getSingleTorqueValue(JointEnum.J5);
    			tA[5] = externalData.getSingleTorqueValue(JointEnum.J6);
    			tA[6] = externalData.getSingleTorqueValue(JointEnum.J7);
        		// Do this only for normal trajectory points (without handguiding)
    			for(int k=0;k<7;k++)
    			{
    				stopRobot = (stopRobot || (tA[k]>=10))&&(targetCnt==0 || targetCnt==2);
    			}
    			
    			// Stop motion if necessary
    			if(stopRobot)
    			{
    				//getLogger().info("Stopping robot");
    				targetMC.cancel();
    		    	IOlinkage.setLEDLightGreen(false);
    		    	IOlinkage.setLEDLightRed(true);
    				break;	// get out of the while loop
    			}
    		} // while(!targetMC.isFinished())
    		
    	} // for(int i=0; i<nrPoints; i++)
    	
    	// Reset trajectory
    	isRecorded = false;
    	recordedPath.clear();
    	getLogger().info("Reset trajectory");
    }
    
    private void sendToSimulator()
    {
    	_lbr.setESMState("1");
    	t1 = getFrame("/Recording/Center");
    	target = t1.copyWithRedundancy();
    	targetMC = _lbr.moveAsync(ptp(target).setMode(cartImpCtrlMode)); 
    	
    	while(!targetMC.isFinished())
    	{
    		_jointPos = _lbr.getCurrentJointPosition();
			_udpState.qA17[0] = _jointPos.get(JointEnum.J1);
			_udpState.qA17[1] = _jointPos.get(JointEnum.J2);
			_udpState.qA17[2] = _jointPos.get(JointEnum.J3);
			_udpState.qA17[3] = _jointPos.get(JointEnum.J4);
			_udpState.qA17[4] = _jointPos.get(JointEnum.J5);
			_udpState.qA17[5] = _jointPos.get(JointEnum.J6);
			_udpState.qA17[6] = _jointPos.get(JointEnum.J7);
    	}
    		
    }
    
    // Some UDP action

    private boolean startUdpCommunication()
    {
        if (null == _udpHelper)
        {
            _udpHelper = new UdpHelper2(LOCAL_PORT, "DefaultApplication", _logger);
            if (!_udpHelper.initUDP())
            {
                return false;
            }

            // create additional thread to receive UDP packages the code of the thread is defined in UdpHelper.run()
            _udpReceiverThread = Executors.newSingleThreadExecutor();
            _udpReceiverThread.execute(_udpHelper);
        }

        return true;
    }
    
    private boolean startUdpState()
    {
        if (null == _udpState)
        {
     	   _udpState = new UdpState(LOCAL_PORT_COPPELIA, "DefaultApplication", _logger);
            if (!_udpState.initUDP())
            {
                return false;
            }

            // create additional thread to receive UDP packages the code of the thread is defined in UdpHelper.run()
            _udpStateThread = Executors.newSingleThreadExecutor();
            _udpStateThread.execute(_udpState);
        }

        return true;
    }
 /*   private void updateUdp()
    {
    	// This only updates UDP variables
    	for (int i = 0; i<7; i++)
    	{
    		_udpHelper.qA17[i] = Math.toDegrees(qA[i]);
    	}
   	
    } */
    public void recordManequinn()
    {
    	// Local variables
    	double[] measuredTorques;
    	double[] externalTorques;
    	TorqueSensorData measuredData, externalData;
    	double tA1, tA2, tA3, tA4, tA5, tA6, tA7;
    	double[] tA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	JointPosition qA17;
    	double[] qA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	ForceSensorData measurements;
    	String temp_qA;
    	int splNr = 2;

    	forName = "C:\\Users\\KukaUser\\Downloads\\collectedMannequin\\free\\allForces" + (String.valueOf(targetCnt)) + ".txt";
    	forFile.doStuff(startFile,forName);
    	
    	posName = "C:\\Users\\KukaUser\\Downloads\\collectedMannequin\\free\\eePosition" + (String.valueOf(targetCnt)) + ".txt";
    	posFile.doStuff(startFile,posName);
    	

    	getLogger().info("Started files: " + posName + ", " + forName);
    	
    	// Spline to go to all positions    	
    	ObjectFrame[] Pi = new ObjectFrame[9];
    	Pi[0] = getFrame("/ManUpperLeg/P1");
    	Pi[1] = getFrame("/ManUpperLeg/P2");
    	Pi[2] = getFrame("/ManUpperLeg/P3");
    	Pi[3] = getFrame("/ManUpperLeg/P4");
    	Pi[4] = getFrame("/ManUpperLeg/P5");
    	Pi[5] = getFrame("/ManUpperLeg/P4");
    	Pi[6] = getFrame("/ManUpperLeg/P3");
    	Pi[7] = getFrame("/ManUpperLeg/P2");
    	Pi[8] = getFrame("/ManUpperLeg/P1");
    	// P1, P2, P3 = circular; P4, P5 -> Lin
    	Spline splineForward = new Spline(lin(Pi[0]),circ(Pi[1],Pi[2]),lin(Pi[3]),lin(Pi[4]));
    	Spline splineBackward = new Spline(lin(Pi[4]),lin(Pi[3]),circ(Pi[2],Pi[1]),lin(Pi[0]));
    	
    	// Trajectory loop
    	for (int i = 0; i<=splNr ; i++)
    	{

        	switch(i)
        	{
	        	case 0:
	            	getLogger().info("Moving  robot forward");
	        		targetMC = _lbr.moveAsync(splineForward); 
	        		break;
	        	case 1:
	            	getLogger().info("Moving  robot backward");
	        		targetMC = _lbr.moveAsync(splineBackward);
	        		break;	
        	}
        	
        	// In the meantime, get the EE and Q forces every 50ms
        	while (!targetMC.isFinished())
        	{
        		// Positions
				qA17 = _lbr.getCurrentJointPosition();
				targetMs = _lbr.getFlange();
				qA[0] = qA17.get(JointEnum.J1);
				qA[1] = qA17.get(JointEnum.J2);
				qA[2] = qA17.get(JointEnum.J3);
				qA[3] = qA17.get(JointEnum.J4);
				qA[4] = qA17.get(JointEnum.J5);
				qA[5] = qA17.get(JointEnum.J6);
				qA[6] = qA17.get(JointEnum.J7);
				// Forces
				// From the whole body 
				measuredData = _lbr.getMeasuredTorque();
				externalData = _lbr.getExternalTorque();
				// Individually.
				measuredTorques = measuredData.getTorqueValues();
				externalTorques = externalData.getTorqueValues();
				
				// Joint torques
				tA1 = measuredData.getSingleTorqueValue(JointEnum.J1);
				tA2 = measuredData.getSingleTorqueValue(JointEnum.J2);
				tA3 = measuredData.getSingleTorqueValue(JointEnum.J3);
				tA4 = measuredData.getSingleTorqueValue(JointEnum.J4);
				tA5 = measuredData.getSingleTorqueValue(JointEnum.J5);
				tA6 = measuredData.getSingleTorqueValue(JointEnum.J6);
				tA7 = measuredData.getSingleTorqueValue(JointEnum.J7);
				
				// End-effector forces
				measurements = _lbr.getExternalForceTorque(_lbr.getFlange(), World.Current.getRootFrame());
				//getLogger().info("Postion =" + qA17);
				
				// dt 
				ThreadUtil.milliSleep(5);
				
				// Print to file(s)
				temp_qA = "";
				for(int k=0; k<7; k++){
					temp_qA += (String.valueOf(qA[k])) + ",";
				}
		    	posFile.doStuff(saveFile, temp_qA + (String.valueOf(targetMs)) + ";\n");
		    	
				message = String.valueOf(tA1)+","+String.valueOf(tA2)+String.valueOf(tA3)+","+String.valueOf(tA4)+","+String.valueOf(tA5)+","+String.valueOf(tA6)+","+String.valueOf(tA7)+",";
		    	message += "F "+ String.valueOf(measurements)+";\n";
		    	forFile.doStuff(saveFile, message);
		    	
				    			
        	} // While (moving to a point)
        	
        	ThreadUtil.milliSleep(5); // Break time between points
    	} // For loop (trajectory points)

    	posFile.doStuff(endFile, posName);
    	forFile.doStuff(endFile, forName);
} // Function

    
    public void followTwithNullS()
    {
    	
    	// Trajectory Points
    	ObjectFrame[] Pi = new ObjectFrame[9];
    	Pi[0] = getFrame("/ManUpperLeg/P1");
    	Pi[1] = getFrame("/ManUpperLeg/P2");
    	Pi[2] = getFrame("/ManUpperLeg/P3");
    	Pi[3] = getFrame("/ManUpperLeg/P4");
    	Pi[4] = getFrame("/ManUpperLeg/P5");
    	Pi[5] = getFrame("/ManUpperLeg/P4");
    	Pi[6] = getFrame("/ManUpperLeg/P3");
    	Pi[7] = getFrame("/ManUpperLeg/P2");
    	Pi[8] = getFrame("/ManUpperLeg/P1");
    	// Local variables
    	Frame current;
    	if(!stopRobot)
    	{
    		if(targetCnt<8){
        		target = Pi[targetCnt].copyWithRedundancy();
    		}else{    			
        		target = Pi[8].copyWithRedundancy();
    		}
    		
    		targetMC = _lbr.moveAsync(ptp(target).setMode(cartImpCtrlMode)); 
    		// Only temporarily:
    		//if(!nullUpdated)
    		//{
        	//	target = getFrame("/Recording/Center").copyWithRedundancy();
            //	targetMC = _lbr.moveAsync(ptp(target).setMode(cartImpCtrlMode));    			
        		//getLogger().info("Moving to point " + target);
    		//}else{
    		//	targetMC = _lbr.moveAsync(new PTP(nullJoints).setMode(cartImpCtrlMode));  
        		//getLogger().info("Moving to updated point ");
    		//}


        	while (!targetMC.isFinished())
        	{
    			// ----------------- Measuring forces -----------------
    			// From the whole body 
    			measuredData = _lbr.getMeasuredTorque();
    			externalData = _lbr.getExternalTorque();
    			// Taking it individually
    			measuredTorques = measuredData.getTorqueValues();
    			externalTorques = externalData.getTorqueValues();
    			
    			// For the UDP, take the full torque values
    			teA[0] = measuredData.getSingleTorqueValue(JointEnum.J1);
    			teA[1] = measuredData.getSingleTorqueValue(JointEnum.J2);
    			teA[2] = measuredData.getSingleTorqueValue(JointEnum.J3);
    			teA[3] = measuredData.getSingleTorqueValue(JointEnum.J4);
    			teA[4] = measuredData.getSingleTorqueValue(JointEnum.J5);
    			teA[5] = measuredData.getSingleTorqueValue(JointEnum.J6);
    			teA[6] = measuredData.getSingleTorqueValue(JointEnum.J7);

		    	// For the stop motion it looks on the external torques
    			tA[0] = externalData.getSingleTorqueValue(JointEnum.J1);
    			tA[1] = externalData.getSingleTorqueValue(JointEnum.J2);
    			tA[2] = externalData.getSingleTorqueValue(JointEnum.J3);
    			tA[3] = externalData.getSingleTorqueValue(JointEnum.J4);
    			tA[4] = externalData.getSingleTorqueValue(JointEnum.J5);
    			tA[5] = externalData.getSingleTorqueValue(JointEnum.J6);
    			tA[6] = externalData.getSingleTorqueValue(JointEnum.J7);
    			
    			// At the end-effector
    			measurements = _lbr.getExternalForceTorque(_lbr.getFlange());
    			force = measurements.getForce();
    			fEE[0] = force.getX();
    			fEE[1] = force.getY();
    			fEE[2] = force.getZ();
    			force = measurements.getTorque();
    			fEE[3] = force.getX();
    			fEE[4] = force.getY();
    			fEE[5] = force.getZ();
    			
    			// ----------------- Measuring positions -----------------
    			// Null space joints: current vs desired
    			qA17 = _lbr.getCurrentJointPosition();
				qA[0] = qA17.get(JointEnum.J1);
				qA[1] = qA17.get(JointEnum.J2);
				qA[2] = qA17.get(JointEnum.J3);
				qA[3] = qA17.get(JointEnum.J4);
				qA[4] = qA17.get(JointEnum.J5);
				qA[5] = qA17.get(JointEnum.J6);
				qA[6] = qA17.get(JointEnum.J7);
				
				qA17d = _lbr.getCommandedJointPosition();
				qAd[0] = qA17d.get(JointEnum.J1);
				qAd[1] = qA17d.get(JointEnum.J2);
				qAd[2] = qA17d.get(JointEnum.J3);

				// At the end-effector
        		current = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"), getFrame("/Recording/Center"));
        		
        		pEE[0] = current.getX();
        		pEE[1] = current.getY();
        		pEE[2] = current.getZ();
        		pEE[3] = current.getAlphaRad();
        		pEE[4] = current.getBetaRad();
        		pEE[5] = current.getGammaRad(); 
        		
		    	// This only updates UDP variables
		    	for (int i = 0; i<7; i++)
		    	{
		    		_udpHelper.qA17[i] = qA[i];
		    		_udpHelper.tA17[i] = teA[i] - tA[i];
		    		//getLogger().info("Updating UDP");
		    		if(i<6){
		    			_udpHelper.pEE[i] = pEE[i];
		    			_udpHelper.fEE[i] = fEE[i];
		    		}
		    	}
		    	
				// Stop motion condition
    			for (int k=0;k<7;k++)
    			{
    				stopRobot = stopRobot || (tA[k]>LIMIT);
    				/*if(tA[k]>LIMIT)
    				{
    					getLogger().info("Force exceeded in joint " + k);
    				}*/
    			}
    			for (int k=0;k<6;k++)
    			{
    				stopRobot = stopRobot || (fEE[k]>LIMIT);
    				/*if(fEE[k]>LIMIT)
    				{
    					getLogger().info("Force exceeded in EE-axis " + k);
    				}*/
    			}
    			
    			// Stop motion if necessary
    			if(stopRobot)
    			{
    				//getLogger().info("Stopping robot");
    				targetMC.cancel();
    		    	IOlinkage.setLEDLightGreen(false);
    		    	IOlinkage.setLEDLightRed(true);
    				break;	// get out of the while loop
    			}
    			
        	} // while (!targetMC.isFinished())    		
    	} // move if !stopRobot

    	// End of the motion
    	if(stopRobot)
    	{
    	ThreadUtil.milliSleep(500);
    	stopRobot = false;
    	IOlinkage.setLEDLightGreen(true);
    	IOlinkage.setLEDLightRed(false);
    	}else{
    		if(targetCnt==8){
    			increment = -1;	// Go forward
    		}else if (targetCnt==0)
    		{
    			increment = 1;	 // Go backwards
    		}    		
    		// If we reach the end and the null space joints differ too much, we update it
    		boolean tempUpdate = false;
    		for(int k=0; k<3; k++)
    		{
    			dqA[k] = qAd[k]-qA[k];
    			tempUpdate = tempUpdate || ((dqA[k]>=DELTAQ) && (Math.abs(qA[k])<qAL[k]));
    			if((dqA[k]>=DELTAQ) && !(Math.abs(qA[k])<qAL[k]))
    			{
    				getLogger().info("Null Joint "+k+" close to its limits");
    			}
    		}

    		if(tempUpdate)
    		{
    			getLogger().info("Difference in a null Joint over 20 degrees");
        		// Only update if the distance between the current frame and the goal frame are OK
        		boolean tempCartesian = false;
        		Frame goal = _lbr.getCommandedCartesianPosition(_linkage.getFrame("/TCP"), getFrame("/Recording/Center"));
        		//Frame next = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"), getFrame("/Recording/Center"));
        		// Original : double disY = Math.abs(goal.getY()-next.getY());
        		double disX = Math.abs(goal.getX()-pEE[0]);
        		double disY = Math.abs(goal.getY()-pEE[1]);
        		double disZ = Math.abs(goal.getZ()-pEE[2]);
        		
        		tempCartesian = (disX<DELTAC) && (disY<DELTAC) && (disZ<=DELTAC);

        		// Actual update
        		if(tempCartesian)
        		{
        			nullJoints =new JointPosition(qA[0],qA[1],qA[2],qA[3],qA[4],qA[5],qA[6]);
        			getLogger().info("Updated");
        			if(!nullUpdated)
        			{
        				nullUpdated = true;
        			}        			
        		}else{
        			getLogger().info("Current Cartesian point too far for update");
        		}

    		}
    		
    		targetCnt+=increment;
    		//ThreadUtil.milliSleep(250);
    	}
    	
} // void end
    
    
    public void followTrajectoryPTP_async()
    {
    	
    	// Trajectory Points
    	ObjectFrame[] Pi = new ObjectFrame[9];
    	Pi[0] = getFrame("/ManUpperLeg/P1");
    	Pi[1] = getFrame("/ManUpperLeg/P2");
    	Pi[2] = getFrame("/ManUpperLeg/P3");
    	Pi[3] = getFrame("/ManUpperLeg/P4");
    	Pi[4] = getFrame("/ManUpperLeg/P5");
    	Pi[5] = getFrame("/ManUpperLeg/P4");
    	Pi[6] = getFrame("/ManUpperLeg/P3");
    	Pi[7] = getFrame("/ManUpperLeg/P2");
    	Pi[8] = getFrame("/ManUpperLeg/P1");
    	
    	if(!stopRobot)
    	{
    		if(targetCnt<8){
        		target = Pi[targetCnt].copyWithRedundancy();
    		}else{    			
        		target = Pi[8].copyWithRedundancy();
    		}
        	getLogger().info("Moving to point " + target);
        	targetMC = _lbr.moveAsync(ptp(target).setMode(cartImpCtrlMode));
        	while (!targetMC.isFinished())
        	{
    			// ----------------- Measuring forces ---------------------------
    			// From the whole body 
    			//measuredData = _lbr.getMeasuredTorque();
    			externalData = _lbr.getExternalTorque();
    			// Taking it individually
    			//measuredTorques = measuredData.getTorqueValues();
    			externalTorques = externalData.getTorqueValues();
    			tA[0] = externalData.getSingleTorqueValue(JointEnum.J1);
    			tA[1] = externalData.getSingleTorqueValue(JointEnum.J2);
    			tA[2] = externalData.getSingleTorqueValue(JointEnum.J3);
    			tA[3] = externalData.getSingleTorqueValue(JointEnum.J4);
    			tA[4] = externalData.getSingleTorqueValue(JointEnum.J5);
    			tA[5] = externalData.getSingleTorqueValue(JointEnum.J6);
    			tA[6] = externalData.getSingleTorqueValue(JointEnum.J7);

    			// At the end-effector
    			measurements = _lbr.getExternalForceTorque(_lbr.getFlange());
    			force = measurements.getForce();
    			fEE[0] = force.getX();
    			fEE[1] = force.getY();
    			fEE[2] = force.getZ();
    			force = measurements.getTorque();
    			fEE[3] = force.getX();
    			fEE[4] = force.getY();
    			fEE[5] = force.getZ();
    			
    			// Stop motion condition
    			for (int k=0;k<7;k++)
    			{
    				stopRobot = stopRobot || (tA[k]>LIMIT);
    				if(tA[k]>LIMIT)
    				{
    					getLogger().info("Force exceeded in joint " + k);
    				}
    			}
    			for (int k=0;k<6;k++)
    			{
    				stopRobot = stopRobot || (fEE[k]>LIMIT);
    				if(fEE[k]>LIMIT)
    				{
    					getLogger().info("Force exceeded in EE-axis " + k);
    				}
    			}
    			
    			// Stop motion if necessary
    			if(stopRobot)
    			{
    				getLogger().info("Stopping robot");
    				targetMC.cancel();
    		    	IOlinkage.setLEDLightGreen(false);
    		    	IOlinkage.setLEDLightRed(true);
    				break;	// get out of the while loop
    			}
    			
        	} // while (!targetMC.isFinished())    		
    	} // move if !stopRobot

    	// End of the motion
    	if(stopRobot)
    	{
    	ThreadUtil.milliSleep(5000);
    	stopRobot = false;
    	IOlinkage.setLEDLightGreen(true);
    	IOlinkage.setLEDLightRed(false);
    	}else{
    		if(targetCnt==8){
    			increment = -1;	// Go forward
    		}else if (targetCnt==0)
    		{
    			increment = 1;	 // Go backwards
    		}
    		targetCnt+=increment;
    	}
    	
} // void end
    
    // ------------------------- Carol testing ends here -------------------------
    /**
     * Implementation of the callback method to listen for monitor state events. An additional single thread is required
     * to use dialog boxes for user feedback without blocking the listener callback.
     * */
    @Override
    public void onMonitoringStateChanged(BrakeTestMonitorEvent event)
    {
        if (!event.getCurrentCyclicMonitorState().equals(_currentStatus))
        {
            _currentStatus = event.getCurrentCyclicMonitorState();
            _isPostponementAcceptable = event.isPostponementAcceptable();
            _isBrakeTestExecutable = event.isBrakeTestExecutable();
            _diskStatus = event.getStateOfFreeDiskSpace();

            _logger.info("BrakeTestState changed to " + _currentStatus);

            // Run the handling of the event with dialog boxes in an additional Thread
            _workerToHandleEvent.execute(new Runnable()
            {
                @Override
                public void run()
                {
                    handleCyclicMonitoring();
                }
            });
        }
    }

    private void handleInitialBraketestExecution()
    {
        String text = "";
        if (_isBrakeTestExecutable)
        {
            text = "Do you want to start a brake test execution now?";
            if (0 == getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, text, "Yes", "No"))
            {
                _action = UserAction.START_BRAKETEST;
            }
        }
        else
        {
            text = "The brake test is not executable! The system is in " + _currentStatus.toString() + " state. "
                    + (_currentStatus.equals(CyclicMonitoringState.FATAL_ERROR) ? "All robot motions from the "
                            + "application are locked. Please contact customer support!" : "");
            if (0 == getApplicationUI().displayModalDialog(ApplicationDialogType.ERROR, text, "Exit application"))
            {
                // terminate application
                _action = UserAction.EXIT_APPLICATION;
            }
        }
    }

    private void handleCyclicMonitoring()
    {
        String text = "";
        if (0 < _diskStatus.compareTo(BrakeTestDiskSpaceState.ENOUGH_SPACE))
        {
            text = "The state of the free disk space is " + _diskStatus + ". What would you like to do?";
            if (0 != getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, text, "continue"))
            {
                _logger.error("That should not be possible");
            }
        }

        if (_currentStatus.equals(CyclicMonitoringState.FEEDBACK_REQUIRED))
        {
            handleFeedbackRequiredState();
        }
        else if (_currentStatus.equals(CyclicMonitoringState.ERROR))
        {
            handleErrorState();
        }
        else if (_currentStatus.equals(CyclicMonitoringState.FATAL_ERROR))
        {
            text = "The system is in FATAL_ERROR state. All robot motions from the application are locked. Please "
                    + "contact customer support!";
            if (0 == getApplicationUI().displayModalDialog(ApplicationDialogType.ERROR, text, "Exit application"))
            {
                // terminate application
                _action = UserAction.EXIT_APPLICATION;
            }
        }
    }

    private void handleErrorState()
    {
        String text = "";
        if (_diskStatus.equals(BrakeTestDiskSpaceState.NOT_ENOUGH_SPACE))
        {
            text = "A brake test is overdue, but the disk space is not enough! Please free disk space on hard "
                    + "disk or contact Service Support!";
            if (0 == getApplicationUI().displayModalDialog(ApplicationDialogType.ERROR, text, "Exit application"))
            {
                // terminate application
                _action = UserAction.EXIT_APPLICATION;
            }
        }
        else
        {
            text = "A brake test is overdue! The robot can only be used after a successful brake test. "
                    + "Do you want to start it now?";
            if (0 == getApplicationUI().displayModalDialog(ApplicationDialogType.ERROR, text, "Yes"))
            {
                _action = UserAction.START_BRAKETEST;
            }
        }
    }

    private void handleFeedbackRequiredState()
    {
        boolean postponeUnacceptable = false;
        String text = "";
        text = "A brake test is due in " + _brakeTestMonitor.getTimeTillBraketestOverdue() + "seconds. "
                + "What would you like to do?";
        int response = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
                text, "start", "postpone");
        if (0 == response)
        {
            _action = UserAction.START_BRAKETEST;
        }
        else if (1 == response)
        {
            _action = UserAction.POSTPONE_BRAKETEST;
            if (_isPostponementAcceptable)
            {
                _brakeTestMonitor.postponeBrakeTest();
            }
            else
            {
                postponeUnacceptable = true;
            }
        }
        else
        {
            _logger.error("That should not be possible (2)");
        }
        if (postponeUnacceptable)
        {
            text = "A postponement is not possible anymore! Do you want to start the brake test now?";
            if (0 == getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
                    text, "Yes"))
            {
                _action = UserAction.START_BRAKETEST;
            }
        }
    }

    @Override
    public void dispose()
    {
        _brakeTestMonitor.removeMonitoringListener(this);

        // Dispose the additional single thread used to collect user feedback with dialog boxes.
        if (_workerToHandleEvent != null)
        {
            try
            {
                _workerToHandleEvent.shutdown();
                _workerToHandleEvent.awaitTermination(5, TimeUnit.SECONDS);
            }
            catch (InterruptedException e)
            {
                _logger.warn("Interruption in shutting down the event worker!", e);
            }
            _workerToHandleEvent.shutdownNow();
        }

		 _udpHelper.dispose();

	        if (_udpReceiverThread != null)
	        {
	            try
	            {
	                _udpReceiverThread.shutdown();
	                _udpReceiverThread.awaitTermination(5, TimeUnit.SECONDS);
	            }
	            catch (InterruptedException e)
	            {
	                _logger.warn("Interruption in shutting down the event worker!", e);
	            }
	            _udpReceiverThread.shutdownNow();
	        }
	        
        super.dispose();
    }
}
