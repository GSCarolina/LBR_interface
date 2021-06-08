package thesisAPP;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

import thesisMethods.ParamClass;


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
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.PositionInformation;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.ErrorHandlingAction;
import com.kuka.roboticsAPI.motionModel.IErrorHandler;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.task.ITaskLogger;

/**
 * Implementation of a sample application handling the cyclic brake test. As soon as the application is started, the
 * user is asked to manage the initial state of the cyclic monitoring, e.g. start the brake test execution or continue
 * with the application. Then, a continuous motion of the robot between two joint configurations is started to simulate
 * the normal application workflow. The IBrakeTestMonitorListener is used to receive events about state changes of the
 * cyclic brake test monitoring and react accordingly.
 */
public class VirtualWall extends RoboticsAPIApplication implements IBrakeTestMonitorListener
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
    private JointPosition _jointPos;
    private IMotionContainer _mc;

    /** User commanded action in the application dataflow. */
    private UserAction _action = UserAction.NO_ACTION;

    /** Additional thread to handle BrakeTestMonitorEvents with dialog boxes for user feedback. */
    private ExecutorService _workerToHandleEvent = Executors.newSingleThreadExecutor();
    
    // --------- Others --------------------
    // => Controller <= 
	CartesianImpedanceControlMode cartImpCtrlMode;
	// controller parameters
	public float KX, KY, KZ, KROT, DAll;
	public float KX_init, KY_init, KZ_init, KROT_init, DAll_init;
	// => Motion <=
	public ObjectFrame t1;
	public Frame target;
	public int targetCnt;
	public IMotionContainer targetMC;
	private AbstractFrame targetMs;

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
        
    	// --------------- modifications ------------------
        // Start end-effector
		_linkage.attachTo(_lbr.getFlange());
		_linkage.getLoadData().setMass(0.975);
		
        // Impedance controller 
        cartImpCtrlMode = new CartesianImpedanceControlMode();
		KX = 200;
		KY = 200;
		KZ = 100;
		KROT = 300;
		DAll = (float)1.0;
		KX_init = 200;
		KY_init = 200;
		KZ_init = 100;
		KROT_init = 300;
		DAll_init = (float)1.0;
		
		IOlinkage.setLEDLightGreen(false);
		IOlinkage.setLEDLightRed(false);
		
		override = 0.5;
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
        		KX = 200;
        		KY = 200;
        		KZ = 100;
        		KROT = 300;
        		DAll = (float)0.1;
            	setNullSpaceImp(KX,KY,KZ,KROT,DAll);
            	_lbr.move(ptp(getFrame("/Recording/Center")).setMode(cartImpCtrlMode));
            	
            	getPotentialEnergy();
            }
            // Modify the controller gains overtime
            if (null == _mc || _mc.isFinished() && runOnce)
            {
            	// Do something
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
    	ThreadUtil.milliSleep(1000);
    }

// ---------------------------------------- Energy methods --------------------------------------
    public double[] getKineticEnergy()
    {
    	// This functions returns an array with the Energy values for every axis and rotation
    	
    	double energy[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	// calculate distance between current point and where it should go
    	double[] pEE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, pEE_goal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
    	double[] dpEE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	
    	Frame Pi = _lbr.getCommandedCartesianPosition(_linkage.getFrame("/TCP"), getFrame("/Recording/Center"));
    	pEE_goal[0] = Pi.getX();
    	pEE_goal[1] = Pi.getY();
    	pEE_goal[2] = Pi.getZ();
    	pEE_goal[3] = Pi.getAlphaRad();
    	pEE_goal[4] = Pi.getBetaRad();
    	pEE_goal[5] = Pi.getGammaRad(); 
		
    	Frame Preal = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"), getFrame("/Recording/Center"));
		pEE[0] = Preal.getX();
		pEE[1] = Preal.getY();
		pEE[2] = Preal.getZ();
		pEE[3] = Preal.getAlphaRad();
		pEE[4] = Preal.getBetaRad();
		pEE[5] = Preal.getGammaRad(); 
		
		for (int i = 0; i<6; i++)
		{
			dpEE[i] = Math.abs(pEE_goal[i]-pEE[i]);
		}
		
		// distance -> energy in every axis
		double k = 1.0;
		for (int i = 0; i<6; i++)
		{
			energy[i] = 0.5*k*(dpEE[i]*dpEE[i]);
		}		
		
    	return energy;
    }
    
    public double[] getPotentialEnergy()
    {
    	// This functions returns an array with the Energy values for every axis and rotation 	
    	double energy[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	// calculate distance between current point and the virtual wall
    	double[] pEE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, pEE_wall = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
    	double[] dpEE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	ObjectFrame virtual_wall = getFrame("/Wall");
    	pEE_wall[0] = virtual_wall.getX();
    	pEE_wall[1] = virtual_wall.getY();
    	pEE_wall[2] = virtual_wall.getZ();
    	pEE_wall[3] = virtual_wall.getAlphaRad();
    	pEE_wall[4] = virtual_wall.getBetaRad();
    	pEE_wall[5] = virtual_wall.getGammaRad(); 
    	
    	Frame Preal = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"), getFrame("/Recording/Center"));
		pEE[0] = Preal.getX();
		pEE[1] = Preal.getY();
		pEE[2] = Preal.getZ();
		pEE[3] = Preal.getAlphaRad();
		pEE[4] = Preal.getBetaRad();
		pEE[5] = Preal.getGammaRad(); 
		
		for (int i = 0; i<6; i++)
		{
			dpEE[i] = Math.abs(pEE_wall[i]-pEE[i]);
		}
		
		getLogger().info("Frame info: X = " + pEE_wall[0] + ", Y = " + pEE_wall[1] + ", Z = " + pEE_wall[2] + ", A = " + pEE_wall[3] + ", B = " + pEE_wall[4] + ", C = " + pEE_wall[5]);
		// distance -> energy in every axis
		double k = 1.0;
		// potential energy in high school = mgh, here some weird stuff 
		for (int i = 0; i<6; i++)
		{
			energy[i] = 0.5*k*(dpEE[i]);
		}		
		
    	return energy;
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
