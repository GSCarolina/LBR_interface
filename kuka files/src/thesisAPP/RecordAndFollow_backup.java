package thesisAPP;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.handGuiding;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

import thesisAPP.BrakeTestMonitorSampleApp.UserAction;
import thesisMethods.*;
import thesisUtilities.*;
//import thesisUtilities.UdpState;
//import thesisUtilities.WriteToFile;
//import thesisUtilities.WriteToFile2;


import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.LinkageIOGroup;
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
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.PositionInformation;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.ErrorHandlingAction;
import com.kuka.roboticsAPI.motionModel.IErrorHandler;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.task.ITaskLogger;

/**
 * Implementation of a sample application handling the cyclic brake test. As soon as the application is started, the
 * user is asked to manage the initial state of the cyclic monitoring, e.g. start the brake test execution or continue
 * with the application. Then, a continuous motion of the robot between two joint configurations is started to simulate
 * the normal application workflow. The IBrakeTestMonitorListener is used to receive events about state changes of the
 * cyclic brake test monitoring and react accordingly.
 */
public class RecordAndFollow_backup extends RoboticsAPIApplication implements IBrakeTestMonitorListener
{

    /**
     * User action for interaction with application workflow.
     * */
    public enum UserAction
    {
        NO_ACTION,
        START_BRAKETEST,
        POSTPONE_BRAKETEST,
        EXIT_APPLICATION;
    }

    private MedController _controller;

    // --------- Necessary stuff -------------
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
	
	// --------------------------------------
    private BrakeTestMonitor _brakeTestMonitor;
    private CyclicMonitoringState _currentStatus = CyclicMonitoringState.OK;
    private boolean _isPostponementAcceptable = false;
    private boolean _isBrakeTestExecutable = true;
    private BrakeTestDiskSpaceState _diskStatus = BrakeTestDiskSpaceState.UNKNOWN;

    private int _responseTime = 60;
    private double _vel = 0.2;
    private IMotionContainer _mc;

    /** User commanded action in the application dataflow. */
    private UserAction _action = UserAction.NO_ACTION;

    /** Additional thread to handle BrakeTestMonitorEvents with dialog boxes for user feedback. */
    private ExecutorService _workerToHandleEvent = Executors.newSingleThreadExecutor();
    
    // --------- UDP action --------- 
    private UdpHelper2 _udpHelper;
	private static final int LOCAL_PORT_MATLAB = 30002;	// This local port is for matlab
	private ExecutorService _udpHelperThread;

	private UdpState _udpState;
	private static final int LOCAL_PORT_COPPELIA = 30007;	// This local port is for the simulator
	private ExecutorService _udpStateThread;

    // --------- Trajectory --------------------
	TorqueSensorData measuredData, externalData;
	ForceSensorData measurements;
	Vector force;
	// velocity follow-up
	VelClass velSettings;
	// recording process
    private JointPosition _jointPos, _jointPos_old, _jointPos_udp;
	// trajectory limit values
	public final double[] Q_LIM = {Math.toRadians(150.0), Math.toRadians(110.0), Math.toRadians(150.0), Math.toRadians(110.0), Math.toRadians(150.0), Math.toRadians(110.0), Math.toRadians(155.0)};
	private double[] qA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	
    // --------- Trajectory PahClass (because of the null pointer issue) --------------------
	public double[] qA_old = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double diffqA = 0.0;
	
	public final double[] qA_diff = {0.2, 0.075, 0.2, 0.075, 0.075, 0.075, 0.075}; // max difference to consider the robot being misplaced
	public double[] PEE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
	
	public double[] measuredTorques, externalTorques;
	public double[] tA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] tA_old = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] tAext = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] tAext_old = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] tA_diff = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] tAext_diff = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	public double[] teeA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] fEE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
	final double LIMIT = 10;	// Constant value
	
	// recording paths using hand-guiding
	public List<JointPosition> recordedPathJS, correctedPathJS; 
	public List<Frame> recordedPathCS; 
	// unused
	public List<TorqueSensorData>  recordedTorqueJS, recordedExtJS;
	public List<ForceSensorData>  recordedForce;	
	
	// Files handling 
	public WriteToFile jointFile;
	public WriteToFile2 posFile;
	public WriteToFile3 forFile;
	public ReadFile recording;
	public String message, jointName, posName;
	public String forName;
	public String recName;
	
    // --------- Lambda and Beta --------------------
	RatioClass impGains;
	double energyMargin = 0.1;
	boolean dEt = false;
	public int cycle = 0;
	ObjectFrame fWall, fCenter;
	Frame fX, fGoal, fVC;
	double[] xVC = {0.0, 0.0, 0.0};
	/////////////////////////////////
	

    // --------- Others --------------------
	// => State Machine <=
	public final int ST_INIT = 0, ST_RECORD = 1, ST_WAIT = 2, ST_RUN = 3, ST_STOP = 4;
	public int state = ST_INIT;	
    // => Controller <= 
	CartesianImpedanceControlMode cartImpCtrlMode;
	// controller parameters
	public float KX_init, KY_init, KZ_init, KROT_init, DAll_init;
	// => Motion <=
	public ObjectFrame t1;
	public Frame target;
	public int targetCnt;
	public IMotionContainer targetMC;
	private AbstractFrame targetMs;
	public double velJS = 0.2;
	

 	public double override;

	// Robot app handling
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
        
        _jointPos_udp = new JointPosition(
                Math.toRadians(30),
                Math.toRadians(30),
                Math.toRadians(0),
                Math.toRadians(0),
                Math.toRadians(0),
                Math.toRadians(0),
                Math.toRadians(0));
        
    	// --------------- modifications ------------------
        // Start end-effector
		_linkage.attachTo(_lbr.getFlange());
		_linkage.getLoadData().setMass(0.975);
		
        // Impedance controller 
        cartImpCtrlMode = new CartesianImpedanceControlMode();
		KX_init = 200;
		KY_init = 200;
		KZ_init = 100;
		KROT_init = 50;
		DAll_init = (float)1.0;
		
		IOlinkage.setLEDLightGreen(false);
		IOlinkage.setLEDLightRed(false);
		
		override = 0.5;
		
		
		// Objects
		impGains = new RatioClass();
		fCenter = getFrame("/Recording/Center");
		int tStep = 100;
		impGains.init(fCenter.copyWithRedundancy(),tStep);
		
		// trajectories start up
		recordedPathJS = new ArrayList<JointPosition>();
		recordedPathCS = new ArrayList<Frame>();
		correctedPathJS = new ArrayList<JointPosition>();
		//recordedPathJST = new ArrayList<JointPosition>();
		recordedTorqueJS = new ArrayList<TorqueSensorData>();
		recordedExtJS = new ArrayList<TorqueSensorData>();
		recordedForce  = new ArrayList<ForceSensorData>();
		
    	recordedPathJS.clear();
    	correctedPathJS.clear();
    	recordedPathCS.clear();
    	
    	velSettings = new VelClass();
    	fVC = new Frame();
    }

    @Override
    public void run()
    {
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

        // Other stuff ---- carol was here ---- 
        boolean runOnce = false;
        
        while (!_action.equals(UserAction.EXIT_APPLICATION))
        {
            // ******************* Execution of a brake test commanded via user interaction *******************
            if (_action.equals(UserAction.START_BRAKETEST) && _isBrakeTestExecutable)
            {
                // if another motion is active, cancel it first
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

            // ******************* Initialize controller settings *******************
            if (null == _mc || _mc.isFinished() && !runOnce)
            {
            	getLogger().info("Setting initial values");
            	getApplicationControl().setApplicationOverride(override);
            	_lbr.move(ptp(getFrame("/Recording/Center")));
            	// Do once
            	runOnce = true;
            	setNullSpaceImp(KX_init,KY_init,KZ_init,KROT_init,DAll_init);
            	_lbr.move(ptp(getFrame("/Recording/Center")).setMode(cartImpCtrlMode));
            	
            	// start the UDP communication         	
            	startUdpHelper();
            	getLogger().info("UDP socket avaliable for MATLAB on port " + LOCAL_PORT_MATLAB);
            	startUdpState();
            	getLogger().info("UDP socket avaliable for CoppeliaSim on port " + LOCAL_PORT_COPPELIA);
            	
            	
            	
            }
            // Normal run: state machine
            if (null == _mc || _mc.isFinished() && runOnce)
            {
            	// local variables
            	double[] qA = {0,0,0,0,0,0,0};
            	boolean valid_qA = true, updateController = false, updateVelocity = false, initPoint = false;
            	double q_temp = 0.0;
            	int validCnt = 0;
            	getLogger().info("Running state machine: status = " + state);
            	
            	// main loop
            	while(true){
            		switch(state)
            		{
            		case ST_INIT:
        		    	// internal variables
        		    	_udpState.isMoving = 0;
        		    	_udpState.JS_checked = false;
        		    	_udpState.sim_finished = false;
        		    	_udpState.sim_status = false;
        		    	updateController = false;
        		    	_udpState.recordedPathJS.clear();
        		    	
            			// record a trajectory with handguiding
        				getLogger().info("=== 1. HANDGUIDING PATH ===");
            			if (selectRecord())
            			{
            				getLogger().info("--> 1.a. Recording trajectory <--");
                			// initial state: record trajectory
                			String recordName = new String("exercise");
                			recordHandGuiding(recordName);
            			}
            			// load trajectory from a file
            			else
            			{
            				getLogger().info("--> 1.a Loading trajectory from file <--");
                			String exercise = new String("exercise");
                			loadTrajectory(exercise);
                			
                			if(selectCorrection())
                			{
                				getLogger().info("--> 1.b Correct loaded trajectory <--");
                    			updateRecordedPath();
                			}
                			else
                			{
                				// take the path as a it is
                				correctedPathJS.clear();
                				for(int index=0; index<recordedPathJS.size();index++)
                				{
                					correctedPathJS.add(recordedPathJS.get(index));
                				}
                			}
                			
            			}
            			
            			// Transfer to the simulator socket all the recorded trajectory.
            			_udpState.JS_size = correctedPathJS.size();
            	    	for(int i=0;i<correctedPathJS.size();i++)
            	    	{
            	    		// Joint-Space
            	    		_udpState.recordedPathJS.add(correctedPathJS.get(i));
            	    	}
            	    	getLogger().info("Trajectory recorded with " + _udpState.recordedPathJS.size() + " points.");
            	    	
        				getLogger().info("=== 2. COLLECT HANDGUIDING ===");
        				getLogger().info("--> Waiting for simulator. <---");

            	    	
            			state = ST_WAIT;
            			break;
            			
            		case ST_WAIT:
            			// wait state: wait for release signal from CoppeliaSim
        		    	// internal variables
        		    	_udpState.isMoving = 0;
        		    	//_udpState.JS_checked = false;
        		    	_udpState.sim_finished = false;
        		    	_udpState.sim_status = false;
        		    	updateController = false;
        		    	
            			if(_udpState.JS_checked)
            			{
            				getLogger().info("=== 3. MOVING ROBOT AROUND ===");
            				getLogger().info("Now ready for receiving position from CoppeliaSim");
            				// update UDP var for simulator
                    		_jointPos_udp = _lbr.getCurrentJointPosition();
                    		_udpState.qA17[0] = _jointPos_udp.get(JointEnum.J1);
                    		_udpState.qA17[1] = _jointPos_udp.get(JointEnum.J2);
                    		_udpState.qA17[2] = _jointPos_udp.get(JointEnum.J3);
                    		_udpState.qA17[3] = _jointPos_udp.get(JointEnum.J4);
                    		_udpState.qA17[4] = _jointPos_udp.get(JointEnum.J5);
                    		_udpState.qA17[5] = _jointPos_udp.get(JointEnum.J6);
                    		_udpState.qA17[6] = _jointPos_udp.get(JointEnum.J7);
                    		
            				state = ST_RUN;
            				_udpState.pathCnt = 1;
            				velJS = velSettings.velJS;
            				
            				// keep info updated on Matlab socket
                    		_udpHelper.stiffnessAxis = impGains.Caxis[0];
                    		_udpHelper.stiffnessAngles = impGains.Crot;
                    		_udpHelper.damping = impGains.Dtotal;
                    		_udpHelper.velJS = velSettings.velJS;
                    		_udpHelper.force = velSettings.fCS;
                    		
            			}
            			break;
            		case ST_RUN:
            			// follow CoppeliaSim :D 
            			//_jointPos = followingSimulator();
            			final double maxdC = 0.1, maxdD = 0.1; // fin values
            			
            			if(!_udpState.sim_finished && !_udpState.sim_status)
            			{
            				_jointPos = followingSimulator();
            				targetMC = _lbr.moveAsync(ptp(_jointPos).setMode(cartImpCtrlMode).setJointVelocityRel(velJS)); 
                        	
                        	IOlinkage.setLEDLightGreen(true);
            		    	IOlinkage.setLEDLightRed(false);
            		    	
                        	while (!targetMC.isFinished())
                        	{                        		
                        		//getLogger().info(_udpState.testMsg);
                        		//ThreadUtil.milliSleep(1000);
                        		// ---> IMPEDANCE CONTROLLER GAINS <---
                        		//fWall = getFrame("/Wall");
                        		//fVC = fWall.copyWithRedundancy();
                        		//xVC[0] = fVC.getX();
                        		//xVC[1] = fVC.getY();
                        		//xVC[2] = fVC.getZ();
                        		//fCenter = getFrame("/Recording/Center");
                        		
                        		// Using world frame
                        		if(!initPoint)
                        		{
                            		fGoal = _lbr.getCommandedCartesianPosition(_linkage.getFrame("/TCP"), World.Current.getRootFrame());//
                            		initPoint = true;
                        		}
                        		fX = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"), World.Current.getRootFrame());
                        		
                        		// Using reference frame
                        		//fGoal = _lbr.getCommandedCartesianPosition(_linkage.getFrame("/TCP"), getFrame("/tRef"));
                        		//fX = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"), getFrame("/tRef"));
                        		
                        		// ---> CLOSEST VIRTUAL CONSTRAINT POINT <---.
                        		//xVC[0] = _udpState.vcPoint[0];
                        		//xVC[1] = _udpState.vcPoint[1];
                        		//xVC[2] = _udpState.vcPoint[2];
                        		
                        		xVC = setVCfromCoppelia(fX);
                        		
                        		// impedance gains
                        		//updateController = false;

                        		updateController = impGains.updateGains(xVC, fGoal, fX, maxdC, maxdD);
                        		//getLogger().info("RHOs " + impGains.dummy[1] + " , " + impGains.dummy[2] + " , " + impGains.dummy[3]);
                        		//getLogger().info("xC " + fGoal.getX() + " , " + fGoal.getY() + " , " + fGoal.getZ());
                        		//getLogger().info("VW " + xVC[0] + " , " + xVC[1] + " , " + xVC[2]);
                        		                        		  
                        		// update UDP var for simulator
                        		_jointPos_udp = _lbr.getCurrentJointPosition();
                        		_udpState.qA17[0] = _jointPos_udp.get(JointEnum.J1);
                        		_udpState.qA17[1] = _jointPos_udp.get(JointEnum.J2);
                        		_udpState.qA17[2] = _jointPos_udp.get(JointEnum.J3);
                        		_udpState.qA17[3] = _jointPos_udp.get(JointEnum.J4);
                        		_udpState.qA17[4] = _jointPos_udp.get(JointEnum.J5);
                        		_udpState.qA17[5] = _jointPos_udp.get(JointEnum.J6);
                        		_udpState.qA17[6] = _jointPos_udp.get(JointEnum.J7);
                        		
                        		// ---> MOVEMENT TIME-DEPENDANT VARIABLES <---
                        		force = _lbr.getExternalForceTorque(_lbr.getFlange()).getForce();
                        		updateVelocity = velSettings.updateVelocity(force);
                        		
                        		
                        		// ---> KEEP OR UPDATE CURRENT MOTION <---
                        		// Cancel movement if the increment is enough to update the controller
                        		if(updateController || updateVelocity)
                        		{
                        			targetMC.cancel();
                        			// display message
                        			//getLogger().info(cycle + " - Robot pushed. New Vel = " +velSettings.velJS + ". CAxis = " +impGains.Caxis[0] + ", CAngles = "+impGains.Crot + ", D = "+impGains.Dtotal);
                        			//getLogger().info(cycle + " -xC = "+fGoal.getX()+", "+fGoal.getY()+", "+fGoal.getZ());
                        			//getLogger().info("VC = "+xVC[0]+", "+xVC[1]+", "+xVC[2]);
                        			//getLogger().info("X = "+fX.getX()+", "+fX.getY()+", "+fX.getZ());
                        			
                        			getLogger().info(cycle + "- RHO "+impGains.APF.rho0+", "+impGains.APF.rho1+","+impGains.APF.rho);
                        			
                        			cycle++;
                        			IOlinkage.setLEDLightGreen(true);
                    		    	IOlinkage.setLEDLightRed(true);
                    		    	
                    		    	// keep info updated on Matlab socket
                            		_udpHelper.stiffnessAxis = impGains.Caxis[0];
                            		_udpHelper.stiffnessAngles = impGains.Crot;
                            		_udpHelper.damping = impGains.Dtotal;
                            		_udpHelper.velJS = velSettings.velJS;
                            		_udpHelper.force = velSettings.fCS;
                            		
                            		
                    		    	ThreadUtil.milliSleep(100);
                        		} 
                        		ThreadUtil.milliSleep(100);
                        	}    
                        	
                        	// Scenario 1: motion properly finished
                        	if (_udpState.pathPointer >= _udpState.pathCnt && !updateController && !updateVelocity)
                        	{
                        		//getLogger().info("Reached path point"); //
                        		_udpState.isMoving = 1;
                        		_udpState.pathCnt++;
                        		initPoint = false;
                        		getLogger().info("initPoint " + _udpState.pathCnt);
                        	}
                        	// Scenario 2: motion aborted to update velocity
                        	//if(updateVelocity)
                        	//{
                        		//velJS = velSettings.velJS;
                        		//updateVelocity = false;
                        		/*
                        		if(velSettings.updateCheck)
                        		{
                        			getLogger().info("updateCheck");
                        		}
                        		if(velSettings.updateVel)
                        		{
                        			getLogger().info("updateVel " + velSettings.velJS_diff + ", "+ velSettings.vLimit);
                        		}
                        		*/
                        	//}                        	
                        	// Scenario 3: motion aborted to update controller
                        	if(updateController || updateVelocity)
                        	{
                        		velJS = velSettings.velJS;
                        		//getLogger().info("Updating controller");
                            	setNullSpaceImp((float)impGains.Caxis[0],(float)impGains.Caxis[1],(float)impGains.Caxis[2],(float)impGains.Crot,(float)impGains.Dtotal);
                            	// set new settings with the current position and move back
                            	//_jointPos = _lbr.getCurrentJointPosition();
                            	//targetMC = _lbr.move(ptp(_jointPos).setMode(cartImpCtrlMode).setJointVelocityRel(velJS));   
                            	updateController = false;
                            	updateVelocity = false;
                            	_udpState.isMoving = 0;
                        	}

                        	
            			}
            			else
            			{
                			getLogger().info("--> Requested to go back to init <--");
            				state = ST_STOP;
            				velJS = 0.2;
            			}
                    	
                    	
            			break;
            		case ST_STOP:
            			// stop: trajectory finished, wait for beginning again?
            			IOlinkage.setLEDLightGreen(false);
        		    	IOlinkage.setLEDLightRed(true);
        				getLogger().info("=== 4. END PATH ===");
        				if(_udpState.sim_finished)
        				{
        					getLogger().info("--> End point reached <--");
        				}
        				else if (_udpState.sim_status)
        				{
        					getLogger().info("Simulation scene on start. Aborted trajectory execution.");
        				}
        				target = getFrame("/Recording/Center").copyWithRedundancy();
        		    	targetMC = _lbr.move(ptp(target).setMode(cartImpCtrlMode).setJointVelocityRel(0.2)); 
        		    	
        		    	_udpState.JS_checked = false;
        		    	
        		    	IOlinkage.setLEDLightGreen(false);
        		    	IOlinkage.setLEDLightRed(false);
        		    	
        		    	
        		    	state = ST_INIT;
            			break;
            		}

                	// constant updates
            		_udpState.status = state;
            		_udpHelper.status = state;
            		ThreadUtil.milliSleep(10);
            	} //while(true)


            }
        }
    }
    
 // ----------------------------------------------------------------------------------------------------------
    public void setNullSpaceImp(float stifX, float stifY, float stifZ, float stiffROT, float dampAll)
    {
    	cartImpCtrlMode.parametrize(CartDOF.X).setStiffness(stifX);
    	cartImpCtrlMode.parametrize(CartDOF.Y).setStiffness(stifY);
    	cartImpCtrlMode.parametrize(CartDOF.Z).setStiffness(stifZ);
    	cartImpCtrlMode.parametrize(CartDOF.ROT).setStiffness(stiffROT);
    	
    	if (dampAll<(float)0.1 || dampAll>(float)1.0)
    	{
    		cartImpCtrlMode.parametrize(CartDOF.ALL).setDamping(1.0);
    	}
    	else
    	{
    		cartImpCtrlMode.parametrize(CartDOF.ALL).setDamping(1.0);
    	}
    	
    	cartImpCtrlMode.setNullSpaceStiffness(1.0);
    	cartImpCtrlMode.setNullSpaceDamping(0.7);
    	//ThreadUtil.milliSleep(1000);
    }

// ---------------------------------------- Energy methods --------------------------------------
// See ParamClass and RatioClass
// ---------------------------------------- Simulator methods --------------------------------------
    public JointPosition followingSimulator()
    {
		//getLogger().info("from UDP update");---
    	boolean valid_qA = false;
    	double q_temp = 0.0;
    	int validCnt = 0;
    	JointPosition _target = new JointPosition(_lbr.getCurrentJointPosition());
		
		for(int k=0; k<7; k++)
		{
			q_temp = Math.abs(_udpState.sim_qA[k]);
			if (q_temp <= Q_LIM[k])
			{
				validCnt++;
				qA[k] = _udpState.sim_qA[k];
			}
			//else // old
			//{
			//	getLogger().info("not valid position on joint " + k);
			//}
			        			
		}
		//getLogger().info("validCnt = " + validCnt + ", error = " + _udpState.sim_qA_error);
		valid_qA = (validCnt >=6) && !_udpState.sim_qA_error;
		
		// if target position is invalid, keep going to wherever this was commanded previously
		if(valid_qA)
		{
			//getLogger().info("position valid");
			JointPosition coppeliaJS = new JointPosition(qA[0],qA[1],qA[2],qA[3],qA[4],qA[5],qA[6]);  
			_target = coppeliaJS;
			//getLogger().info("Using a position from Coppelia");
		}

		return _target;
    }
    
    // setting virtual constraint wall
    public double[] setVCfromCoppelia(Frame currentX)
    {
    	double[] simVC = {0.0,0.0,0.0};
    	
    	switch(_udpState.type)
    	{
    	case 1:
    		simVC[0] = currentX.getX();
    		simVC[1] = currentX.getY();
    		simVC[2] = _udpState.vcPoint[2];
    	default:
    		simVC[0] = currentX.getX();
    		simVC[1] = currentX.getY();
    		simVC[2] = _udpState.vcPoint[2];
    	}
    	
    	return simVC;
    }
// ---------------------------------------- Trajectory methods --------------------------------------
    
    public void recordHandGuiding(String exName)
    {
    	// local variables
    	boolean stopRobot = false, updateFile = false;
    	double diffqA = 0.0, velJS = 0.3;
    	int updateCnt = 0;
    	Frame current;
    	
    	recordedPathJS.clear();
    	correctedPathJS.clear();
    	
    	// 1. Move to start point
    	IOlinkage.setLEDLightGreen(false);
    	IOlinkage.setLEDLightRed(false);
    	_lbr.setESMState("1");
    	target = getFrame("/Recording/KE_start").copyWithRedundancy();
    	targetMC = _lbr.move(ptp(target).setMode(cartImpCtrlMode).setJointVelocityRel(velJS)); 
		// update values
		_jointPos = _lbr.getCurrentJointPosition();
		_jointPos_old = _lbr.getCurrentJointPosition();
		qA_old[0] = _jointPos.get(JointEnum.J1);
		qA_old[1] = _jointPos.get(JointEnum.J2);
		qA_old[2] = _jointPos.get(JointEnum.J3);
		qA_old[3] = _jointPos.get(JointEnum.J4);
		qA_old[4] = _jointPos.get(JointEnum.J5);
		qA_old[5] = _jointPos.get(JointEnum.J6);
		qA_old[6] = _jointPos.get(JointEnum.J7);
		recordedPathJS.add(_jointPos);			// save the start position
		
		// 2. Record hand guiding
    	IOlinkage.setLEDLightGreen(true);
    	IOlinkage.setLEDLightRed(true);
    	recordedPathJS.clear();
    	recordedPathCS.clear();
		_lbr.setESMState("2");
		getLogger().info("Hand guiding active");
    	IOlinkage.setLEDLightGreen(true);
    	IOlinkage.setLEDLightRed(false);
    	
		targetMC = _lbr.moveAsync(handGuiding());

    	while(!targetMC.isFinished())
    	{				
    		// Joint-space
    		_jointPos = _lbr.getCurrentJointPosition();
			qA[0] = _jointPos.get(JointEnum.J1);
			qA[1] = _jointPos.get(JointEnum.J2);
			qA[2] = _jointPos.get(JointEnum.J3);
			qA[3] = _jointPos.get(JointEnum.J4);
			qA[4] = _jointPos.get(JointEnum.J5);
			qA[5] = _jointPos.get(JointEnum.J6);
			qA[6] = _jointPos.get(JointEnum.J7);
			
			updateFile = false;
			// a new position has been reached if at least 1 joint has been moved 0.075 rad
			for(int j=0;j<7;j++)
			{
				diffqA = Math.abs(qA[j]-qA_old[j]);
				if(diffqA >= 0.075)
				{
					updateFile = true;
				}
			}
			// If a new position has been reached, add it to the recordings
			if(updateFile)
			{
				//getLogger().info("Updating path nr. " + updateCnt);
				// Joint-space
				recordedPathJS.add(_jointPos);
				correctedPathJS.add(_jointPos);
				// Cartesian-space
				current = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"), World.Current.getRootFrame());
				//current = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"));
				recordedPathCS.add(current);
				
				// update joints values
				for(int j=0;j<7;j++)
				{
					qA_old[j] = qA[j];
				}
				
				// save torques as well
				measuredData = _lbr.getMeasuredTorque();
				recordedTorqueJS.add(measuredData);
				
				externalData = _lbr.getExternalTorque();
				recordedExtJS.add(externalData);				
				//
				updateCnt++;
			}// if(updateFile)
			
			
    	} // while(!targetMC.isFinished()) --
    	
    	// Save recorded paths in files
    	getLogger().info("Printing recorded path for exercise " +exName+ " with " + updateCnt + " points.");  	

    	//getLogger().info("File(s) started. Saving JS.");
    	
    	// Saving Joint-space path
    	jointName = "C:\\Users\\KukaUser\\Downloads\\thesisData\\pos\\JS_" +exName+".txt";
    	jointFile.init(jointName);
    	String qMsg = new String();
    	for(int i=0;i<recordedPathJS.size();i++)
    	{
    		// Joint-Space
    		_jointPos = recordedPathJS.get(i);
			qA[0] = _jointPos.get(JointEnum.J1);
			qA[1] = _jointPos.get(JointEnum.J2);
			qA[2] = _jointPos.get(JointEnum.J3);
			qA[3] = _jointPos.get(JointEnum.J4);
			qA[4] = _jointPos.get(JointEnum.J5);
			qA[5] = _jointPos.get(JointEnum.J6);
			qA[6] = _jointPos.get(JointEnum.J7);
			qMsg = new String(qA[0]+","+qA[1]+","+qA[2]+","+qA[3]+","+qA[4]+","+qA[5]+","+qA[6]);
			
			//jointFile.doStuff(writeFile,qMsg);
			jointFile.saveLine(qMsg);
    	}
    	jointFile.end();
    	
    	// Saving Cartesian-space path 
    	posName = "C:\\Users\\KukaUser\\Downloads\\thesisData\\pos\\CS_" + exName + ".txt";
    	posFile.init(posName);
    	for(int i=0;i<recordedPathCS.size();i++)
    	{
    		// Cartesian-Space
			current = recordedPathCS.get(i);
			PEE[0] = current.getX();
			PEE[1] = current.getY();
			PEE[2] = current.getZ();
			PEE[3] = current.getAlphaRad();
			PEE[4] = current.getBetaRad();
			PEE[5] = current.getGammaRad();
			qMsg = new String(PEE[0]+","+PEE[1]+","+PEE[2]+","+PEE[3]+","+PEE[4]+","+PEE[5]);
			
			posFile.saveLine(qMsg);
    	}
    	
    	
    	// Saving forces
    	forName = "C:\\Users\\KukaUser\\Downloads\\thesisData\\pos\\FORCE_" +exName+".txt";
    	forFile.init(forName);
    	String tMsg = new String(), tExtMsg = new String();
    	
    	for(int i=0;i<recordedTorqueJS.size();i++)
    	{
    		// From the whole body 
    		measuredData = recordedTorqueJS.get(i);		
    		externalData = recordedExtJS.get(i);
    		
    		// Joint torques
    		tA[0] = measuredData.getSingleTorqueValue(JointEnum.J1);
    		tA[1] = measuredData.getSingleTorqueValue(JointEnum.J2);
    		tA[2] = measuredData.getSingleTorqueValue(JointEnum.J3);
    		tA[3] = measuredData.getSingleTorqueValue(JointEnum.J4);
    		tA[4] = measuredData.getSingleTorqueValue(JointEnum.J5);
    		tA[5] = measuredData.getSingleTorqueValue(JointEnum.J6);
    		tA[6] = measuredData.getSingleTorqueValue(JointEnum.J7);
    		tMsg = new String(tA[0]+","+tA[1]+","+tA[2]+","+tA[3]+","+tA[4]+","+tA[5]+ "," + tA[6] + ";");
    		
    		tAext[0] = externalData.getSingleTorqueValue(JointEnum.J1);
    		tAext[1] = externalData.getSingleTorqueValue(JointEnum.J2);
    		tAext[2] = externalData.getSingleTorqueValue(JointEnum.J3);
    		tAext[3] = externalData.getSingleTorqueValue(JointEnum.J4);
    		tAext[4] = externalData.getSingleTorqueValue(JointEnum.J5);
    		tAext[5] = externalData.getSingleTorqueValue(JointEnum.J6);
    		tAext[6] = externalData.getSingleTorqueValue(JointEnum.J7);
    		
    		tExtMsg = new String(tAext[0]+","+tAext[1]+","+tAext[2]+","+tAext[3]+","+tAext[4]+","+tAext[5]+ "," + tAext[6] + ";");
    		
    		qMsg = tMsg + tExtMsg;
    		posFile.saveLine(qMsg);
    	}		
		forFile.end();
		
    	getLogger().info("Path saved. See file: " + jointName);
    	
    	
    	
    	
		// 3. Move to end point
    	IOlinkage.setLEDLightGreen(false);
    	IOlinkage.setLEDLightRed(false);
    	_lbr.setESMState("1");
    	getLogger().info("Moving to Center");
    	target = getFrame("/Recording/Center").copyWithRedundancy();
    	targetMC = _lbr.move(ptp(target).setMode(cartImpCtrlMode).setJointVelocityRel(velJS)); 
    	
    }

	
    
    public void updateRecordedPath()
    {
    	// local variables
    	boolean stopRobot = false, updateFile = false;
    	double diffqA = 0.0, velJS = 0.3;
    	int updateCnt = 0;
    	Frame current;
    	
    	// 1. Move to start point
    	_lbr.setESMState("1");
    	target = getFrame("/Recording/KE_start").copyWithRedundancy();
    	targetMC = _lbr.move(ptp(target).setMode(cartImpCtrlMode).setJointVelocityRel(velJS));  
		// update values
		_jointPos = _lbr.getCurrentJointPosition();
		_jointPos_old = _lbr.getCurrentJointPosition();
		qA_old[0] = _jointPos.get(JointEnum.J1);
		qA_old[1] = _jointPos.get(JointEnum.J2);
		qA_old[2] = _jointPos.get(JointEnum.J3);
		qA_old[3] = _jointPos.get(JointEnum.J4);
		qA_old[4] = _jointPos.get(JointEnum.J5);
		qA_old[5] = _jointPos.get(JointEnum.J6);
		qA_old[6] = _jointPos.get(JointEnum.J7);
		
		// 2. Play recorded path
    	IOlinkage.setLEDLightGreen(true);
    	IOlinkage.setLEDLightRed(true);
		getLogger().info("Now you can push the robot while moving");
		int posCnt = 0, Pi_index = 0, Pi_limit = 0;
		//final int delta = 10, limit = recordedPathJS.size();
		final int delta = 10, limit = recordedPathCS.size();
		Frame[] Pi = new Frame[delta];
		JointPosition[] Ji = new JointPosition[delta];
		velJS = 1.0;
	
		stopRobot = false;
		boolean stopRobotext = false;
		//////////////// old: JS
		
		correctedPathJS.clear();
    	for(int i=0;i<recordedPathJS.size();i++)
    	{
    		_jointPos = recordedPathJS.get(i);
			qA_old[0] = _jointPos.get(JointEnum.J1);
			qA_old[1] = _jointPos.get(JointEnum.J2);
			qA_old[2] = _jointPos.get(JointEnum.J3);
			qA_old[3] = _jointPos.get(JointEnum.J4);
			qA_old[4] = _jointPos.get(JointEnum.J5);
			qA_old[5] = _jointPos.get(JointEnum.J6);
			qA_old[6] = _jointPos.get(JointEnum.J7);   
			
			IOlinkage.setLEDLightGreen(true);
	    	IOlinkage.setLEDLightRed(false);
	    	
    		//targetMC = _lbr.moveAsync(ptp(_jointPos).setMode(cartImpCtrlMode).setJointVelocityRel(velJS)); 
    		targetMC = _lbr.move(ptp(_jointPos).setMode(cartImpCtrlMode).setJointVelocityRel(velJS)); 
        	while(!targetMC.isFinished())
        	{
        		ThreadUtil.milliSleep(10);
        	}
        	IOlinkage.setLEDLightGreen(false);
	    	IOlinkage.setLEDLightRed(false);
        	ThreadUtil.milliSleep(100);
    		
    		// Check JS displacement
    		_jointPos = _lbr.getCurrentJointPosition();
			qA[0] = _jointPos.get(JointEnum.J1);
			qA[1] = _jointPos.get(JointEnum.J2);
			qA[2] = _jointPos.get(JointEnum.J3);
			qA[3] = _jointPos.get(JointEnum.J4);
			qA[4] = _jointPos.get(JointEnum.J5);
			qA[5] = _jointPos.get(JointEnum.J6);
			qA[6] = _jointPos.get(JointEnum.J7);    		
			
			for(int j=0;j<7;j++)
			{
				diffqA = Math.abs(qA[j]-qA_old[j]);
				if(diffqA >= qA_diff[j])
				{
					stopRobot = true;
					IOlinkage.setLEDLightGreen(true);
			    	IOlinkage.setLEDLightRed(true);
			    	//getLogger().info("New position");
			    	
				}
			}
			if(stopRobot){
				// udpate path
				correctedPathJS.add(_jointPos);
				//ThreadUtil.milliSleep(1000);
			}
			else
			{
				// keep path
				correctedPathJS.add(recordedPathJS.get(i));
			}
				    	
	    	ThreadUtil.milliSleep(50);
	    	IOlinkage.setLEDLightGreen(false);
	    	IOlinkage.setLEDLightRed(false);
	    	stopRobot = false;
	    	stopRobotext = false;
	    	
    	}
    	getLogger().info("Corrected Path with "+ correctedPathJS.size() +" points " );

    	////////////////
    	IOlinkage.setLEDLightGreen(true);
    	IOlinkage.setLEDLightRed(false);
    	
		// 3. Move to end point
    	velJS = 0.3; 
    	getLogger().info("Moving to Center");
    	target = getFrame("/Recording/Center").copyWithRedundancy();
    	targetMC = _lbr.move(ptp(target).setMode(cartImpCtrlMode).setJointVelocityRel(velJS));    	
    }
    
	public void loadTrajectory(String exName)
    {
		boolean errorLoading = false;
		boolean errorOpening = false;
    	// load recording
    	recording = new ReadFile();        
    	recName = "C:\\Users\\KukaUser\\Downloads\\thesisData\\pos\\JS_" +exName+".txt";
    	recording.init(recName); 
    	recording.readContent();
    	
    	if(recording.error)
    	{
    		errorLoading = true;
    	}
 	
    	recordedPathJS.clear();
    	recordedPathCS.clear();
    	
        // process data loop
    	String del = new String(",");
        String[] dataLine;
        int dataNr = 0;
        for(int i=0; i<recording.linesNr;i++){
            dataLine = recording.splitData(del,i);
            dataNr = dataLine.length;
            for(int j=0;j<dataNr;j++){
                qA[j] = Double.parseDouble(dataLine[j]);
            }
            
            if(dataNr == 7)
            {
            	_jointPos = new JointPosition(qA[0],qA[1],qA[2],qA[3],qA[4],qA[5],qA[6]);
                recordedPathJS.add(_jointPos);
            }
            
        }
        
    	if(!errorLoading||dataNr!=6)
    	{
    		getLogger().info("Trajectory sucessfully loaded with " + recording.linesNr + " points, from file: " + recName);
    	}
    	else if (errorLoading)
    	{
    		getLogger().info("Error while reading or opening file");
    	}
    	else
    	{
    		getLogger().info("Error on the path parameters");
    	}
    	
        recording.end();
        
    }
// ---------------------------------------- UDP sockets methods --------------------------------------
   private boolean startUdpHelper()
   {
       if (null == _udpHelper)
       {
           _udpHelper = new UdpHelper2(LOCAL_PORT_MATLAB, "DefaultApplication", _logger);
           if (!_udpHelper.initUDP())
           {
               return false;
           }

           // create additional thread to receive UDP packages the code of the thread is defined in UdpHelper.run()
           _udpHelperThread = Executors.newSingleThreadExecutor();
           _udpHelperThread.execute(_udpHelper);
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
   
   // ---------------------------------------- Others --------------------------------------
   public boolean selectRecord()
   {
       String text = new String("Do you want to record a trajectory or load it?");
       int result = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, text, "RECORD", "LOAD");
       //getLogger().info("Result " + result);
       if (result == 0)
       {
    	   getLogger().info("You have chosen RECORD");
       }
       else
       {
    	   getLogger().info("You have chosen LOAD ");
       }
       boolean output = (result == 0);
       return output;
   }
   
   public boolean selectCorrection()
   {
       String text = new String("Do you want to correct the loaded trajectory?");
       int result = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, text, "YES", "NO");
       //getLogger().info("Result " + result);
       if (result == 0)
       {
    	   getLogger().info("You have chosen YES");
       }
       else
       {
    	   getLogger().info("You have chosen NO");
       }
       boolean output = (result == 0);
       return output;
   }
// ----------------------------------------------------------------------------------------------------------

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

        super.dispose();
    }
}
