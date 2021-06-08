package thesisAPP;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.circ;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.spl;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import shared.Training;
import thesisUtilities.WriteToFile;
import thesisUtilities.WriteToFile2;


import com.kuka.med.controllerModel.MedController;
import com.kuka.med.cyclicBrakeTest.BrakeTestDiskSpaceState;
import com.kuka.med.cyclicBrakeTest.BrakeTestMonitor;
import com.kuka.med.cyclicBrakeTest.BrakeTestMonitorEvent;
import com.kuka.med.cyclicBrakeTest.BrakeTestOutcome;
import com.kuka.med.cyclicBrakeTest.CyclicMonitoringState;
import com.kuka.med.cyclicBrakeTest.IBrakeTestMonitorListener;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.Spline;
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
public class Trajectory extends RoboticsAPIApplication implements IBrakeTestMonitorListener
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

    @Inject
    private ITaskLogger _logger;

    @Inject
    private LBR _lbr;	// Robot

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
 	
 	// I/O Groups
 	private LinkageIOGroup linkage;
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
        override = 0.1;
        runOnce = false;
    	// Messing around
    	getLogger().info("Start up");
    	t1 = getFrame("/TrajectoryFrame/TrajP1");
    	target = t1.copyWithRedundancy();
    	targetCnt = 0;
    	targetMs = _lbr.getFlange();
    	getLogger().info("d " + targetMs);
    	
    	
    	getLogger().info("Starting file");
    	startFile = 0;
    	saveFile = 1;
    	endFile = 2;
    	
    	// How does this work
    	//fileName = "C:\\Users\\KukaUser\\Downloads\\filename4.txt";
    	//message = "Wassuuuuuuuuuuuuuuuup";
    	//myFile.doStuff(startFile, fileName);
    	//myFile.doStuff(saveFile, message);
    	//myFile.doStuff(endFile, message);
    	
    }

    @Override
    public void run()
    {
    	//linkage.setLEDLightGreen(true);
    	//linkage.setLEDLightRed(true);
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
            if (null == _mc || _mc.isFinished() && !runOnce)
            {
            	getLogger().info("Hello world. Moving to start position.");
            	_lbr.move(ptp(getFrame("/ManUpperLeg/P3")));
            	// Do once
            	getApplicationControl().setApplicationOverride(override);
            	runOnce = true;
            	getLogger().info("Do this once.");
            }
            if ((null == _mc || _mc.isFinished()) && runOnce && (targetCnt<=9))
            {
            	/*Trying different trajectories
            	 * Point-to-Point:
            	 * followTrajectoryPTP_async();
            	 * Linear:
            	 * followTrajectoryLIN();
            	 * Record a trajectory positions and forces in files
            	 * recordTrajectory();
            	 * Record an upper-leg routine with the mannequin (position control)
            	 * recordManequinn();
            	 * For (almost) all trajectories:
            	 * targetCnt++;
            	 * */
            	getLogger().info("Manequinn testing");
            	recordManequinn();
            	targetCnt++;
            }
        }
    }
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
				ThreadUtil.milliSleep(50);
				
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
        	
        	ThreadUtil.milliSleep(2000); // Break time between points
    	} // For loop (trajectory points)

    	posFile.doStuff(endFile, posName);
    	forFile.doStuff(endFile, forName);
} // Function

    public void recordTrajectory()
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
    	int trajPoints = 9;
    	//int dt = 0; 	// Increment time between steps

    	forName = "C:\\Users\\KukaUser\\Downloads\\collectedData\\allForces" + (String.valueOf(targetCnt)) + ".txt";
    	forFile.doStuff(startFile,forName);
    	
    	posName = "C:\\Users\\KukaUser\\Downloads\\collectedData\\eePosition" + (String.valueOf(targetCnt)) + ".txt";
    	posFile.doStuff(startFile,posName);
    	

    	getLogger().info("Started files: " + posName + ", " + forName);
    	
    	// Trajectory loop
    	for (int i = 0; i<=trajPoints ; i++)
    	{
        	// Select a target position
    		// The + trajectory
        	switch(i)
        	{
	        	case 0:
	        		t1 = getFrame("/Recording/Center");
	        		break;
	        	case 1:
	        		t1 = getFrame("/Recording/Xbackward");
	        		break;
	        	case 2:
	        		t1 = getFrame("/Recording/Xforward");
	        		break;
	        	case 3:
	        		t1 = getFrame("/Recording/Center");
	        		break;
	        	case 4:
	        		t1 = getFrame("/Recording/Yleft");
	        		break;
	        	case 5:
	        		t1 = getFrame("/Recording/Yright");
	        		break;
	        	case 6:
	        		t1 = getFrame("/Recording/Center");
	        		break;
	        	case 7:
	        		t1 = getFrame("/Recording/Zup");
	        		break;
	        	case 8:
	        		t1 = getFrame("/Recording/Zdown");
	        		break;	
        	}
        	// Mannequin trajectory
        	target = t1.copyWithRedundancy();

        	// Move the robot to that target
        	getLogger().info("Moving to point " + i);
        	targetMC = _lbr.moveAsync(lin(target)); 
        	
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
				ThreadUtil.milliSleep(50);
				
				// Print to file(s)
				temp_qA = "";
				for(int k=0; k<7; k++){
					temp_qA += (String.valueOf(qA[k])) + ",";
				}
		    	posFile.doStuff(saveFile, temp_qA + (String.valueOf(targetMs)) + ";\n");
		    	
				message = String.valueOf(tA1)+","+String.valueOf(tA2)+String.valueOf(tA3)+","+String.valueOf(tA4)+","+String.valueOf(tA5)+","+String.valueOf(tA6)+","+String.valueOf(tA7)+",";
		    	message += "F "+ String.valueOf(measurements)+";\n";
		    	forFile.doStuff(saveFile, message);
		    	
		    	
		    	//myFile.doStuff(startFile, fileName);
		    	//myFile.doStuff(saveFile, message);
		    	//myFile.doStuff(endFile, message);
				    			
        	} // While (moving to a point)
        	
        	ThreadUtil.milliSleep(2000); // Break time between points
    	} // For loop (trajectory points)

    	posFile.doStuff(endFile, posName);
    	forFile.doStuff(endFile, forName);
} // Function
    
    public void followTrajectoryPTP_async()
    {
    	// Follow trajectory
    	getLogger().info("Moving to point " + targetCnt);
    	targetMC = _lbr.moveAsync(ptp(target));  
    	while (!targetMC.isFinished())
    	{
    		getLogger().info("Robot moving - " + targetCnt);
			ThreadUtil.milliSleep(1000);
			if(targetCnt>=5)
			{
				// ----------------- Measuring forces ---------------------------
				// From the whole body 
				TorqueSensorData measuredData = _lbr.getMeasuredTorque();
				TorqueSensorData externalData = _lbr.getExternalTorque();
				// Taking it individually
				double[] measuredTorques = measuredData.getTorqueValues();
				double[] externalTorques = externalData.getTorqueValues();
				
				double torqueA2 = measuredData.getSingleTorqueValue(JointEnum.J2);
				getLogger().info("Currently measured torque for Joint2 [Nm]:" +	torqueA2);
				// At the end-effector
				ForceSensorData measurements = _lbr.getExternalForceTorque(_lbr.getFlange(), World.Current.getRootFrame());
				//
				targetMC.cancel();
				targetCnt = 0;
				getLogger().info("Cancel executed. Forces: " + measurements);
			}
			else
			{
				targetCnt++;
			}
    	}
    	
    	}
    
    public void followTrajectoryPTP()
    {
    	// Follow trajectory
    	getLogger().info("Moving to point 1");
    	_lbr.move(ptp(getFrame("/TrajectoryFrame/TrajP1")));
    	getLogger().info("Moving to point 2");
    	_lbr.move(ptp(getFrame("/TrajectoryFrame/TrajP2")));
    	getLogger().info("Moving to point 3");
    	_lbr.move(ptp(getFrame("/TrajectoryFrame/TrajP3")));
    	getLogger().info("Moving to point 4");
    	_lbr.move(ptp(getFrame("/TrajectoryFrame/TrajP4")));   	
    	getLogger().info("Moving to point 4");
    	_lbr.move(ptp(getFrame("/TrajectoryFrame/TrajP5")));   
    	}
    public void followTrajectoryLIN()
    {
    	// Follow trajectory
    	getLogger().info("Moving to point 1");
    	_lbr.move(lin(getFrame("/TrajectoryFrame/TrajP1")));
    	getLogger().info("Moving to point 2");
    	_lbr.move(lin(getFrame("/TrajectoryFrame/TrajP2")));
    	getLogger().info("Moving to point 3");
    	_lbr.move(lin(getFrame("/TrajectoryFrame/TrajP3")));
    	getLogger().info("Moving to point 4");
    	_lbr.move(lin(getFrame("/TrajectoryFrame/TrajP4")));   	
    	getLogger().info("Moving to point 4");
    	_lbr.move(lin(getFrame("/TrajectoryFrame/TrajP5")));   
    	}
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

        super.dispose();
    }
}
