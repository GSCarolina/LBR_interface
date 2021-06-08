package thesisAPP;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.circ;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.spl;
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
//import thesisUtilities.WriteToFile;
//import thesisUtilities.WriteToFile2;
//import thesisUtilities.WriteToFile3;


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
import com.kuka.roboticsAPI.motionModel.Spline;
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
public class RecordTrajectory extends RoboticsAPIApplication implements IBrakeTestMonitorListener
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
    
    // --------- Some UDP action --------- 
    private UdpHelper2 _udpHelper;
	private static final int LOCAL_PORT = 30007;	// This local port is for matlab
	private ExecutorService _udpReceiverThread;
	
	// --------- Write to file --------- 
	WriteToFile jointFile;
	WriteToFile2 posFile;
	WriteToFile3 forFile;
	WriteTest testFile;
	ReadFile recording;
	String message, jointName, posName, forName, recName, testName;
	
	int testCnt = 0;
	final int tStep = 100;
	// --------- For trajectories --------- 
	// Local variables
	TorqueSensorData measuredData, externalData;
	ForceSensorData measurements;
	Vector force;
	double[] qA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double[] qA_old = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	final double[] qA_diff = {0.2, 0.075, 0.2, 0.075, 0.075, 0.075, 0.075}; // max difference to consider the robot being misplaced
	double[] PEE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
	
	double[] measuredTorques, externalTorques;
	double[] tA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double[] tA_old = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double[] tAext = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double[] tAext_old = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double[] tA_diff = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double[] tAext_diff = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	double[] teeA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double[] fEE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
	final double LIMIT = 10;	// Constant value
	
	// recording paths using hand-guiding
	List<JointPosition> recordedPathJS, correctedPathJS; 
	List<Frame> recordedPathCS; 
	// unused
	List<TorqueSensorData>  recordedTorqueJS, recordedExtJS;
	List<ForceSensorData>  recordedForce;
	
	private JointPosition _jointPos_old;	
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
		KROT_init = 50;
		DAll_init = (float)1.0;
		
		IOlinkage.setLEDLightGreen(false);
		IOlinkage.setLEDLightRed(false);
		
		override = 0.8;
		// trajectories start up
		recordedPathJS = new ArrayList<JointPosition>();
		recordedPathCS = new ArrayList<Frame>();
		correctedPathJS = new ArrayList<JointPosition>();
		//recordedPathJST = new ArrayList<JointPosition>();
		recordedTorqueJS = new ArrayList<TorqueSensorData>();
		recordedExtJS = new ArrayList<TorqueSensorData>();
		recordedForce  = new ArrayList<ForceSensorData>();
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
		    	IOlinkage.setLEDLightGreen(false);
		    	IOlinkage.setLEDLightRed(false);
		    	
            	getLogger().info("Setting initial values");
            	getApplicationControl().setApplicationOverride(override);
            	_lbr.move(ptp(getFrame("/Recording/Center")));
            	// Do once
            	runOnce = true;
        		KX = 200;
        		KY = 200;
        		KZ = 200;
        		KROT = 300;
        		DAll = (float)0.3;
            	setNullSpaceImp(KX,KY,KZ,KROT,DAll);
            	_lbr.move(ptp(getFrame("/Recording/Center")).setMode(cartImpCtrlMode));
            	
            	// start the UDP server         	
            	//startUdpCommunication();
            	//getLogger().info("UDP started.");
            	
            }
            // Modify the controller gains overtime
            if (null == _mc || _mc.isFinished() && runOnce)
            {

            	
            	while(true){
            		String exercise = new String();
            		for(int k=0; k<4; k++)
            		{
            			switch(k)
            			{
            			case 0:
            				exercise = "knee_ext_0";
            				break;
            			case 1:
            				exercise = "knee_ext_1";
            				break;
            			case 2:
            				exercise = "dorso_flex_0";
            				break;
            			case 3:
            				exercise = "dorso_flex_1";
            				break;
            			}
            			exercise = "exercise";
                    	targetMC = _lbr.move(ptp(getFrame("/Recording/Center")).setMode(cartImpCtrlMode));  
                    	//getLogger().info("Recording and playing exercise: " + exercise);
                    	//getLogger().info(" =========== 1. RECORD PATH ===========");
                    	//getLogger().info(" Exercise: " + exercise);
                		//recordHandGuiding(exercise);
                		
                		getLogger().info(" =========== 1. LOAD PATH ===========");
                		getLogger().info(" Exercise: " + exercise + ", cycle " + testCnt);
                		loadTrajectory(exercise);
                		
                		//getLogger().info(" =========== 2. RECORDED PATH ===========");
                		//runRecordedPath(recordedPathJS);
                		
                		getLogger().info(" =========== 3. USING VEL ===========");
                		runTrajectory(recordedPathJS);
                		testCnt++;
                		/*
                		for (int kk = 0; kk<1; kk++)
                		{
                			getLogger().info(" =========== 2. RECORDED PATH ===========");
                    		runRecordedPath(recordedPathJS);
                    		
                    		getLogger().info(" =========== 3. CORRECT PATH ===========");
                    		correctRecordedPath();                			
                    		//
                    		
                    		IOlinkage.setLEDLightGreen(false);
            		    	IOlinkage.setLEDLightRed(false);
            		    	
            		    	getLogger().info(" =========== 4. NEW PATH ===========");
                    		runRecordedPath(correctedPathJS);
                    		
                    		ThreadUtil.milliSleep(20);
                    		
                		}
						*/

                		          			
            		}

            	}//while(true)


            }// if (null == _mc || _mc.isFinished() && runOnce)
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

// ---------------------------------------- Trajectory methods --------------------------------------
    public void loadTrajectory(String exName)
    {
    	// load recording
    	recording = new ReadFile();        
    	recName = "C:\\Users\\KukaUser\\Downloads\\thesisData\\pos\\JS_" +exName+".txt";
    	recording.init(recName); 
    	recording.readContent();
    	
    	if(!recording.error)
    	{
    		getLogger().info("Found trajectory with " + recording.linesNr + " points");
    	}
    	else
    	{
    		getLogger().info("Error while reading or opening file");
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
        
    	if(!recording.error||dataNr!=6)
    	{
    		getLogger().info("Trajectory sucessfully loaded");
    	}
    	else
    	{
    		getLogger().info("Error while reading file");
    	}
    	
        recording.end();
        
    }
    
    public void recordHandGuiding(String exName)
    {
    	// local variables
    	boolean stopRobot = false, updateFile = false;
    	double diffqA = 0.0, velJS = 0.3;
    	int updateCnt = 0;
    	Frame current;
    	
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
    	posFile.end();
    	
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
    		tMsg = new String(tA[0]+","+tA[1]+","+tA[2]+","+tA[3]+","+tA[4]+","+tA[5]+ "," + tA[6]);
    		
    		tAext[0] = externalData.getSingleTorqueValue(JointEnum.J1);
    		tAext[1] = externalData.getSingleTorqueValue(JointEnum.J2);
    		tAext[2] = externalData.getSingleTorqueValue(JointEnum.J3);
    		tAext[3] = externalData.getSingleTorqueValue(JointEnum.J4);
    		tAext[4] = externalData.getSingleTorqueValue(JointEnum.J5);
    		tAext[5] = externalData.getSingleTorqueValue(JointEnum.J6);
    		tAext[6] = externalData.getSingleTorqueValue(JointEnum.J7);
    		
    		tExtMsg = new String(tAext[0]+","+tAext[1]+","+tAext[2]+","+tAext[3]+","+tAext[4]+","+tAext[5]+ "," + tAext[6]);
    		
    		qMsg = tMsg + ", " + tExtMsg;
    		forFile.saveLine(qMsg);
    	}		
		forFile.end();
		
    	getLogger().info("Path saved.");
    	
    	
    	
    	
		// 3. Move to end point
    	IOlinkage.setLEDLightGreen(false);
    	IOlinkage.setLEDLightRed(false);
    	_lbr.setESMState("1");
    	getLogger().info("Moving to Center");
    	target = getFrame("/Recording/Center").copyWithRedundancy();
    	targetMC = _lbr.move(ptp(target).setMode(cartImpCtrlMode).setJointVelocityRel(velJS)); 
    	
    }

    public void runRecordedPath(List<JointPosition> recording)
    {
    	// local variables
    	boolean stopRobot = false, updateFile = false;
    	double diffqA = 0.0, velJS = 0.125;
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
    	IOlinkage.setLEDLightGreen(false);
    	IOlinkage.setLEDLightRed(true);
		getLogger().info("Playing recorded path");
		int posCnt = 0, Pi_index = 0, Pi_limit = 0;
		//final int delta = 10, limit = recordedPathJS.size();
		final int delta = 10, limit = recording.size();
		Frame[] Pi = new Frame[delta];
		JointPosition[] Ji = new JointPosition[delta];
		velJS = 1.0;
		/*
		while(posCnt<limit)
		{
			for(int j=0;j<delta;j++)
			{
				Pi_index = j+posCnt;
				if(Pi_index < limit)
				{
					//Ji[j] = recordedPathJS.get(Pi_index);
					Pi[j] = recordedPathCS.get(Pi_index);;
				}	
				else if (Pi_limit == 0)
				{
					Pi_limit = j;
					getLogger().info("Pi_limit = " + Pi_limit);
				}
			}
			if (Pi_limit == 0)
			{
				getLogger().info("Normal move " + posCnt);
				Spline splineForward = new Spline(spl(Pi[0]),spl(Pi[1]),spl(Pi[2]),spl(Pi[3]),spl(Pi[4]),spl(Pi[5]),spl(Pi[6]),spl(Pi[7]),spl(Pi[8]),spl(Pi[9]));
				targetMC = _lbr.moveAsync(splineForward.setMode(cartImpCtrlMode).setJointVelocityRel(velJS));				
			}
			else
			{
				getLogger().info("Pi_limit move ");
				Spline splineForward = new Spline(spl(Pi[0]));
				switch (Pi_limit)
				{
				case 1:
					splineForward = new Spline(spl(Pi[0]));
					break;
				case 2:
					splineForward = new Spline(spl(Pi[0]),spl(Pi[1]));
					break;
				case 3:
					splineForward = new Spline(spl(Pi[0]),spl(Pi[1]),spl(Pi[2]));
					break;
				case 4:
					splineForward = new Spline(spl(Pi[0]),spl(Pi[1]),spl(Pi[2]),spl(Pi[3]));
					break;
				case 5:
					splineForward = new Spline(spl(Pi[0]),spl(Pi[1]),spl(Pi[2]),spl(Pi[3]),spl(Pi[4]));
					break;
				case 6:
					splineForward = new Spline(spl(Pi[0]),spl(Pi[1]),spl(Pi[2]),spl(Pi[3]),spl(Pi[4]),spl(Pi[5]));
					break;
				case 7:
					splineForward = new Spline(spl(Pi[0]),spl(Pi[1]),spl(Pi[2]),spl(Pi[3]),spl(Pi[4]),spl(Pi[5]),spl(Pi[6]));
					break;
				case 8:
					splineForward = new Spline(spl(Pi[0]),spl(Pi[1]),spl(Pi[2]),spl(Pi[3]),spl(Pi[4]),spl(Pi[5]),spl(Pi[6]),spl(Pi[7]));
					break;
				case 9:
					splineForward = new Spline(spl(Pi[0]),spl(Pi[1]),spl(Pi[2]),spl(Pi[3]),spl(Pi[4]),spl(Pi[5]),spl(Pi[6]),spl(Pi[7]),spl(Pi[8]));
					break;
					
				}
				targetMC = _lbr.moveAsync(splineForward.setMode(cartImpCtrlMode).setJointVelocityRel(velJS));
			}
			
			//Spline splineForward = new Spline(spl(Pi[0]),spl(Pi[1]),spl(Pi[2]),spl(Pi[3]),spl(Pi[4]),spl(Pi[5]),spl(Pi[6]),spl(Pi[7]),spl(Pi[8]),spl(Pi[9]));
			//targetMC = _lbr.moveAsync(splineForward.setMode(cartImpCtrlMode).setJointVelocityRel(velJS));
			
        	while(!targetMC.isFinished())
        	{
        		// do something
        		ThreadUtil.milliSleep(10);
        	}
        	
			posCnt = posCnt + delta;
		}
		*/
		

		
		//////////////// good: JS
		
    	for(int i=0;i<recording.size();i++)
    	{
    		_jointPos = recording.get(i);
    		targetMC = _lbr.moveAsync(ptp(_jointPos).setMode(cartImpCtrlMode).setJointVelocityRel(velJS)); 
    		
        	while(!targetMC.isFinished())
        	{
        		// do something
        		ThreadUtil.milliSleep(10);
        	}
    	}
    	
		//////////////// bad: CS
		/*
    	for(int i=0;i<recordedPathCS.size();i++)
    	{
    		Pi[0] = recordedPathCS.get(i);
    		targetMC = _lbr.moveAsync(ptp(Pi[0]).setMode(cartImpCtrlMode).setJointVelocityRel(velJS)); 
    		
        	while(!targetMC.isFinished())
        	{
        		// do something
        		ThreadUtil.milliSleep(10);
        	}
    	}
    	*/
    	////////////////
    	IOlinkage.setLEDLightGreen(false);
    	IOlinkage.setLEDLightRed(false);
    	
		// 3. Move to end point
    	velJS = 0.3; 
    	getLogger().info("Moving to Center");
    	target = getFrame("/Recording/Center").copyWithRedundancy();
    	targetMC = _lbr.move(ptp(target).setMode(cartImpCtrlMode).setJointVelocityRel(velJS));
    	
    }
    
    public void correctRecordedPath()
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
    
    public double getVelocity_old(double force)
    {
    	// maxVel = 0.6 // old value
    	final double maxVel = 0.99, minVel = 0.00001;
    	final double maxForce = 20, minForce = 10;
    	double output = 0.0;
    	
    	double temp = minVel + (maxVel-minVel)*Math.abs((force-minForce)/(maxForce));
    	
    	if(temp >= maxVel)
    	{
    		output = maxVel;
    	}
    	else if (temp <= minVel)
    	{
    		output = minVel;
    	}
    	else
    	{
    		output = temp;
    	}
    	
    	return output;
    }
    
    public double getVelocity(double force)
    {
    	// Internal variables
    	final double maxVel = 0.99, minVel = 0.00001;
    	final double maxForce = 27, minForce = 20;
    	final double m = (maxVel-minVel)/(Math.exp(maxForce-minForce)-1);
    	final double n = minVel - m;
    	
    	double dForce = force - minForce;
    	if(force < minForce)
    	{
    		dForce = 0;
    	}
    	else if (force > maxForce)
    	{
    		dForce = maxForce-minForce;
    	}
    	    	
    	double output = n + (m*Math.exp(dForce));
    	
    	if (output<minVel)
    	{
    		output = minVel;
    	}
    	else if(output > maxVel)
    	{
    		output = maxVel;
    	}
    	
    	return output;

    }
    
    public void runTrajectory(List<JointPosition> recording)
    {
    	// local variables
    	boolean stopRobot = false, updateFile = false;
    	double diffqA = 0.0, velJS = 0.125;
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
    	IOlinkage.setLEDLightGreen(false);
    	IOlinkage.setLEDLightRed(false);
		getLogger().info("Playing recorded path");
		int posCnt = 0, Pi_index = 0, Pi_limit = 0;
		//final int delta = 10, limit = recordedPathJS.size();
		final int delta = 10, limit = recording.size();
		Frame[] Pi = new Frame[delta];
		JointPosition[] Ji = new JointPosition[delta];

		int i = 0, fCheck = 0;
		boolean updateVel = false;
		velJS = 0.00001;
		
		double fX, fY, fZ, fCS, tempCS, velJS_old, velJS_temp, velJS_diff, fCS_old, f_temp, f_diff;
		final double fLimit = 5, vLimit = 0.1;
		fCS_old = 0.0;
		velJS_old = 0.0;
		
		// For testing: saving stuff on files
    	testName = "C:\\Users\\KukaUser\\Downloads\\thesisData\\pos\\fee20_10_" +testCnt+".txt";
    	testFile.init(testName);
		String fileLine = new String();
		
		while(i<limit)
		{
			IOlinkage.setLEDLightGreen(false);
        	IOlinkage.setLEDLightRed(false);
			// make movement
			_jointPos = recording.get(i);
    		targetMC = _lbr.moveAsync(ptp(_jointPos).setMode(cartImpCtrlMode).setJointVelocityRel(velJS)); 
    		
        	while(!targetMC.isFinished())
        	{            	
        		Vector forceVector = _lbr.getExternalForceTorque(_lbr.getFlange()).getForce();
				//measurements = _lbr.getExternalForceTorque(_lbr.getFlange(), World.Current.getRootFrame());
				fX = forceVector.getX();
				fY = forceVector.getY();
				fZ = forceVector.getZ();
				
				tempCS = Math.pow(fX,2)+Math.pow(fY,2)+Math.pow(fZ,2);
				fCS = Math.sqrt(tempCS);
				
				//updateVel = (fCS >= fLimit);				
				velJS_temp = getVelocity(fCS);
        		
    			// update velocity value
				//velJS_diff = Math.abs(velJS_temp -  velJS);
				f_diff = Math.abs(fCS - fCS_old);
				//f_diff = Math.abs(fCS);
				
				velJS_diff = Math.abs(velJS_temp - velJS_old);
				
				// update condition
				//updateVel = (fCheck>10 || f_diff>=fLimit || velJS_diff>=vLimit);
				updateVel = (fCheck>100 || velJS_diff>=vLimit);
				
				if(updateVel)
				{
					IOlinkage.setLEDLightGreen(true);
			    	IOlinkage.setLEDLightRed(true);
					updateVel = true;
					//velJS_old = velJS;
					fCS_old = fCS;
					velJS_old = velJS;
					velJS = velJS_temp;
					targetMC.cancel();
					fCheck = 0;
					/*
					if(f_diff>=fLimit)
					{
						getLogger().info("Updating due to force");
					}
					else if (velJS_diff>=vLimit)
					{
						getLogger().info("Updating due to velocity");
					}
					else
					{
						getLogger().info("Updating periodic");
					}
					*/
				}
				else
				{
					fCheck++;
				}

				//getLogger().info("velJS_temp = " + velJS_temp);
				//getLogger().info("fdiff= " +f_diff+ ", fCS= "+fCS+" , fCS_old= " +fCS_old);
				
        		ThreadUtil.milliSleep(tStep);
        		
        		// for testing: saving things in file
        		fileLine = fCS + "," + velJS_temp;
        		testFile.saveLine(fileLine);
        	}
        	
        	if(!updateVel)
        	{
        		i++;
        	}    	
        	
        	//else
        	//{
        	//	getLogger().info("Updating 2 ");
        	//}
        	
        	updateVel = false;
			
		}
		

    	IOlinkage.setLEDLightGreen(false);
    	IOlinkage.setLEDLightRed(false);
    	
    	testFile.end();
    	
		// 3. Move to end point
    	velJS = 0.3; 
    	getLogger().info("Moving to Center");
    	target = getFrame("/Recording/Center").copyWithRedundancy();
    	targetMC = _lbr.move(ptp(target).setMode(cartImpCtrlMode).setJointVelocityRel(velJS));
    	
    }
// ----------------------------------------------------------------------------------------------------------
   // Some UDP actionnnn

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
