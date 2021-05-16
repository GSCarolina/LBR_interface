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

import thesisMethods.ParamClass;
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
public class RecordAndFollow extends RoboticsAPIApplication implements IBrakeTestMonitorListener
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
	// recording process
	public double[] qA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] qA_old = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    private JointPosition _jointPos, _jointPos_old;
	List<JointPosition> recordedPathJS; 
	List<Frame> recordedPathCS; 
	// trajectory limit values
	public final double[] Q_LIM = {Math.toRadians(150.0), Math.toRadians(110.0), Math.toRadians(150.0), Math.toRadians(110.0), Math.toRadians(150.0), Math.toRadians(110.0), Math.toRadians(155.0)};
	
    // --------- Energy --------------------
	// energy and parameters
	double[] Uee = {0,0,0,0,0,0};
	double Loff = 0.5;
	double Ekee = 0.0;
	double energyMargin = 0.1;
	// gains
	ParamClass parameter = new ParamClass();
	double[] Caxis = {0.0, 0.0, 0.0};
	double[] Daxis = {0.0, 0.0, 0.0};
	double Dtotal = 0.0;
	int cycle = 0;
	// For the kinetic energy:
	double[] x_old = {0.0,0.0,0.0,0.0,0.0,0.0};
	// to keep on track on the increments
	double[] dUee = {0,0,0,0,0,0}, Uee_old = {0,0,0,0,0,0};
	double dEkee = 0.0, Ekee_old = 0.0;
	boolean dEt = false;
	
	// --------- Write to file ---------- 
	WriteToFile jointFile;
	WriteToFile2 posFile;
	String message, jointName, posName;
	// constant var for handling the files
	private final int startFile = 0, endFile = 2, writeFile = 1;
	
    // --------- Others --------------------
	// => State Machine <=
	public final int ST_INIT = 0, ST_RECORD = 1, ST_WAIT = 2, ST_RUN = 3, ST_STOP = 4;
	public int state = ST_INIT;	
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
		KROT_init = 50;
		DAll_init = (float)1.0;
		
		IOlinkage.setLEDLightGreen(false);
		IOlinkage.setLEDLightRed(false);
		
		override = 0.5;
		
		// trajectories start up
		recordedPathJS = new ArrayList<JointPosition>();
		recordedPathCS = new ArrayList<Frame>();
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
            	// start the first value for the kinetic energy
            	ObjectFrame fX_old = getFrame("/Recording/Center");
            	x_old[0] = fX_old.getX();
            	x_old[1] = fX_old.getY();
            	x_old[2] = fX_old.getZ();
            	x_old[3] = fX_old.getAlphaRad();
            	x_old[4] = fX_old.getBetaRad();
            	x_old[5] = fX_old.getGammaRad(); 
            	
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
            	boolean valid_qA = true, updateController = false;
            	double q_temp = 0.0;
            	int validCnt = 0;
            	getLogger().info("Running state machine: status = " + state);
            	
            	// main loop
            	while(true){
            		switch(state)
            		{
            		case ST_INIT:
            			getLogger().info("Recording trajectory");
            			// initial state: record trajectory
            			String recordName = new String("exercise");
            			recordHandGuiding(recordName);
            			
            			// Transfer to the simulator socket all the recorded trajectory
            			_udpState.JS_size = recordedPathJS.size();
            	    	for(int i=0;i<recordedPathJS.size();i++)
            	    	{
            	    		// Joint-Space
            	    		_udpState.recordedPathJS.add(recordedPathJS.get(i));
            	    	}
            	    	getLogger().info("Trajectory recorded with " + _udpState.recordedPathJS.size() + " points. Now waiting"); 
            			state = ST_WAIT;
            			break;
            			
            		case ST_WAIT:
            			// wait state: wait for release signal from Coppelia Sim
            			if(_udpState.JS_checked)
            			{
            				getLogger().info("Now ready for receiving position from CoppeliaSim");
            				state = ST_RUN;
            			}
            			break;
            		case ST_RUN:
            			// follow CoppeliaSim
            			_jointPos = followingSimulator();
                    	targetMC = _lbr.moveAsync(ptp(_jointPos).setMode(cartImpCtrlMode).setJointVelocityRel(velJS)); 
                    	
                    	while (!targetMC.isFinished())
                    	{
                    		// energy gains
                    		updateController = getEnergyAndGains(energyMargin);
                    		// keep updated online energy on Matlab socket
                    		for(int j = 0; j<6;j++)
                    		{
                    			_udpHelper.Uee[j] = Uee[j];
                    			if(j<3)
                    			{
                    				_udpHelper.Caxis[j] = Caxis[j];
                    			}
                    		}
                    		_udpHelper.Ekee = Ekee;
                    		_udpHelper.D = Dtotal;
                    		
                    		// Cancel movement if the energy difference is enough to update the controller--
                    		if(updateController)
                    		{
                    			targetMC.cancel();
                    			getLogger().info(cycle + " - Robot pushed. Updating gains. Uy = " + Uee[1] + "Ekee = " + Ekee);
                    			cycle++;
                    		}                    		
                        	// do something here to prevent the robot crashing?
                    		
                    	}    
                    	
                    	// Update the controller if needed
                    	if(updateController)
                    	{
                        	setNullSpaceImp((float)Caxis[0],(float)Caxis[1],(float)Caxis[2],KROT_init,(float)Dtotal);
                        	// set new settings with the current position and move back
                        	//_jointPos = _lbr.getCurrentJointPosition();
                        	targetMC = _lbr.move(ptp(_jointPos).setMode(cartImpCtrlMode).setJointVelocityRel(velJS));   
                        	updateController = false;
                    	}
                    	
            			break;
            		case ST_STOP:
            			// stop: trajectory finished, wait for begining again?
            			
            			break;
            		}

                	// constant updates
            		_udpState.status = state;
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
    public double getKineticEnergy()
    {
    	// This functions returns one single value 
    	double Ek = 0.0;
    	// calculate distance between current point and where it should go
    	double[] xCenter = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
    	double[] xWall = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    	double[] x = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
    	double[] dxee = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	double dxee_max = 0.0, dxee_total = 0.0, k = 1.0;
    	
    	// --- THIS NEEDS TO BE MODIFIED ---
    	// calculate speed: dxee = sqrt( dX^2 + dY^2 + dZ^2) when dxee = (x_center - xwall)
    	ObjectFrame fWall = getFrame("/Wall");
    	xWall[0] = fWall.getX();
    	xWall[1] = fWall.getY();
    	xWall[2] = fWall.getZ();
    	xWall[3] = fWall.getAlphaRad();
    	xWall[4] = fWall.getBetaRad();
    	xWall[5] = fWall.getGammaRad(); 
    	
    	ObjectFrame fCenter = getFrame("/Recording/Center");
    	xCenter[0] = fCenter.getX();
    	xCenter[1] = fCenter.getY();
    	xCenter[2] = fCenter.getZ();
    	xCenter[3] = fCenter.getAlphaRad();
    	xCenter[4] = fCenter.getBetaRad();
		xCenter[5] = fCenter.getGammaRad(); 
		// --- THIS NEEDS TO BE MODIFIED ---
		
		for (int i = 0; i<3; i++)
		{
			// maximum square velocity
			dxee[i] =(xCenter[i]-xWall[i])*(xCenter[i]-xWall[i]);
		}
		dxee_max = Math.sqrt(dxee[0]+dxee[1]+dxee[2]);
		if (dxee_max > 0.0)
		{
			k = 2/(Math.pow(dxee_max, 2));
		}

		
		// calculate speed: dxee = sqrt( dX^2 + dY^2 + dZ^2)
		Frame fX = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"), getFrame("/Recording/Center"));
    	x[0] = fX.getX();
    	x[1] = fX.getY();
    	x[2] = fX.getZ();
    	x[3] = fX.getAlphaRad();
    	x[4] = fX.getBetaRad();
		x[5] = fX.getGammaRad(); 
		
		for (int i = 0; i<3; i++)
		{
			dxee[i] =(x[i]-x_old[i])*(x[i]-x_old[i]);
		}
		dxee_total = Math.sqrt(dxee[0]+dxee[1]+dxee[2]);
		
		// how to set up kinetic energy in the axis
		Ek = 0.5*k*dxee_total*dxee_total;
		if(Ek>1)
		{
			Ek=1;
		}
		else if (Ek<0)
		{
			Ek = 0;
		}
		//for (int i = 0; i<3; i++)
		//{
		//	Ek[i] = k*dxee[i]*dxee[i];
		//}	
		
		// Update values
		for (int i = 0; i<6; i++)
		{
			x_old[i] =x[i];
		}
		
    	return Ek;
    }
    
    public double[] getPotentialEnergy(double Loff)
    {
    	// This functions returns an array with the Energy values for every axis and rotation 	
    	double U[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	double xOut[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	// calculate distance between current point and the virtual wall
    	double[] xCenter = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
    	double[] xWall = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    	double[] x = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	// parameters arrays
    	double[] m_x = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	double[] n_x = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	
    	// --- THIS NEEDS TO BE MODIFIED ---
    	// getting values
    	ObjectFrame fWall = getFrame("/Wall");
    	xWall[0] = fWall.getX();
    	xWall[1] = fWall.getY();
    	xWall[2] = fWall.getZ();
    	xWall[3] = fWall.getAlphaRad();
    	xWall[4] = fWall.getBetaRad();
    	xWall[5] = fWall.getGammaRad(); 
    	
    	ObjectFrame fCenter = getFrame("/Recording/Center");
    	xCenter[0] = fCenter.getX();
    	xCenter[1] = fCenter.getY();
    	xCenter[2] = fCenter.getZ();
    	xCenter[3] = fCenter.getAlphaRad();
    	xCenter[4] = fCenter.getBetaRad();
		xCenter[5] = fCenter.getGammaRad(); 
		// --- THIS NEEDS TO BE MODIFIED ---
		
		Frame fX = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"), getFrame("/Recording/Center"));
    	x[0] = fX.getX();
    	x[1] = fX.getY();
    	x[2] = fX.getZ();
    	x[3] = fX.getAlphaRad();
    	x[4] = fX.getBetaRad();
		x[5] = fX.getGammaRad(); 
		
		// calculate m, n parameters
		for (int i = 0; i<3; i++)
		{
			m_x[i] = 1/(xWall[i]-Loff-xCenter[i]);
			if (xCenter[i]>xWall[i])
			{
				m_x[i] = 1/(xWall[i]+Loff-xCenter[i]);
			}
			n_x[i] = -m_x[i]*xCenter[i];			
		}

		
		// calculate potential energy
		for (int i = 0; i<3; i++)
		{
			U[i]= m_x[i]*x[i]+n_x[i];	
			// check boundaries
			if(U[i]<=1.0)
			{
				xOut[i]=x[i];
				if(U[i]<0.0)
				{
					U[i] = 0.0;
				}
			}
			else
			{
				// Rebounce x inside Wall+-Loff
				if (xWall[i]>=xCenter[i])
				{
					xOut[i]=xWall[i]-Loff;
				}
				else
				{
					xOut[i]=xWall[i]+Loff;
				}
				U[i] = 1.0;
			}
		}
		
    	return U;
    }
    
    public boolean getEnergyAndGains(double increment)
    {  	
    	// Get potential energy
    	Uee = getPotentialEnergy(Loff);
    	
    	// Get kinetic energy
    	Ekee = getKineticEnergy();
    	
    	// Set parameters accordingly
    	for (int j = 0; j<3; j++)
    	{
    		Caxis[j] = parameter.getKAxis(Uee[j]);
    		// the getKaxis needs to be fixed:
    		if (Caxis[j]>5000)
    		{
    			Caxis[j] = 5000;
    		}
    	}
    	
    	for (int j = 0; j<3; j++)
    	{
    		Daxis[j] = parameter.getDamp(Ekee);
    	}
    	Dtotal = (Daxis[0]+Daxis[1]+Daxis[2])/3;
    	
		// keep on track on the energy
		for(int j = 0; j<6;j++)
		{
			dUee[j] = Math.abs(Uee[j]-Uee_old[j]);
			Uee_old[j] = Uee[j];
			
			dEt = dEt || (dUee[j] >= 0.03);
		}
		dEkee = Math.abs(Ekee-Ekee_old);
		Ekee_old = Ekee;
		dEt = dEt || (dEkee >= increment);
		
		return dEt;
		
    }
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
// ---------------------------------------- Trajectory methods --------------------------------------
    public void recordHandGuiding(String exName)
    {
    	// local variables
    	boolean stopRobot = false, updateFile = false;
    	double diffqA = 0.0;
    	int updateCnt = 0;
    	Frame current;
    	
    	// 1. Move to start point
    	getLogger().info("Moving to start point");
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
		
		// 2. Record hand guiding
		getLogger().info("Setting handguiding");
    	IOlinkage.setLEDLightGreen(true);
    	IOlinkage.setLEDLightRed(true);
    	recordedPathJS.clear();
    	recordedPathCS.clear();
		_lbr.setESMState("2");
		getLogger().info("Hand guiding active");
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
			for(int j=0;j<7;j++)
			{
				diffqA = Math.abs(qA[j]-qA_old[j]);
				if(diffqA >= 0.075)
				{
					updateFile = true;
				}
			}
			// If robot has been moved, update file
			if(updateFile)
			{
				//getLogger().info("Updating path nr. " + updateCnt);
				recordedPathJS.add(_jointPos);
				// Cartesian-space
				current = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"));
				recordedPathCS.add(current);
				
				// update JS--
				for(int j=0;j<7;j++)
				{
					qA_old[j] = qA[j];
				}
				updateCnt++;
			}// if(updateFile)
			
			
    	} // while(!targetMC.isFinished())
    	/*
    	// Save recorded trajectory in files
    	getLogger().info("Printing recorded path for exercise " +exName+ " with " + updateCnt + "points.");
    	// start file(s)
    	jointName = "C:\\Users\\KukaUser\\Downloads\\thesisData\\pos\\JS_" +exName+".txt";
    	jointFile.doStuff(startFile,jointName);
    	posName = "C:\\Users\\KukaUser\\Downloads\\thesisData\\pos\\CS_" + exName + ".txt";
    	posFile.doStuff(startFile,posName);
    	//getLogger().info("File(s) started. Saving JS.");
    	
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
			qMsg = new String(qA[0]+","+qA[1]+","+qA[2]+","+qA[3]+","+qA[4]+","+qA[5]+","+qA[6]+";");
			
			jointFile.doStuff(writeFile,qMsg);
    	}
		
    	getLogger().info("Path saved.");
    	jointFile.doStuff(endFile,qMsg);
    	posFile.doStuff(endFile,qMsg);
    	*/
    	
		// 3. Move to end point
    	IOlinkage.setLEDLightGreen(false);
    	IOlinkage.setLEDLightRed(false);
    	_lbr.setESMState("1");
    	getLogger().info("Moving to Center");
    	target = getFrame("/Recording/Center").copyWithRedundancy();
    	targetMC = _lbr.move(ptp(target).setMode(cartImpCtrlMode).setJointVelocityRel(velJS)); 
    	
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
