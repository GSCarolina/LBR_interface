package main;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import guidedMode.Guided;
import guidedMode.PathDataRecorder;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javax.inject.Inject;
import javax.inject.Named;

import shared.ButtonTypeEnum;
import shared.ExcutionController;
import shared.Helper;
import shared.LedColor;
import shared.LedColor.Color;
import shared.LiftMode;
import shared.MotionMonitor;
import shared.NaviController;
import shared.TCPConnection;
import shared.Training;
import util.AppStateMachine;
import util.BrakeTestHelper;
import util.ReferencingHelper;
import util.UdpHelper;
import util.UserCmd;
import activeMode.Active;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.BodyIOGroup;
import com.kuka.generated.ioAccess.LinkageIOGroup;
import com.kuka.med.controllerModel.MedController;
import com.kuka.med.mastering.Mastering;
import com.kuka.med.robotState.FailedConditionForNormalUse;
import com.kuka.med.robotState.RobotStateAggregator;
import com.kuka.med.unmasteredApp.MedApplicationCategory;
import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPITask;
import com.kuka.roboticsAPI.applicationModel.tasks.UseRoboticsAPIContext;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ConditionObserver;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.IRisingEdgeListener;
import com.kuka.roboticsAPI.conditionModel.NotificationType;
import com.kuka.roboticsAPI.conditionModel.ObserverManager;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseExecutionService;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.EnablingDeviceState;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.SafetyStopType;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.ioModel.Input;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.task.ITaskLogger;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a
 * {@link RoboticsAPITask#run()} method, which will be called successively in
 * the application lifecycle. The application will terminate automatically after
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an
 * exception is thrown during initialization or run.
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the
 * {@link RoboticsAPITask#dispose()} method.</b>
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
@MedApplicationCategory(checkMastering = false)
public class Robert_2_3 extends RoboticsAPIApplication {
	@Inject
	private LBR _lbrMed;
	private MedController _medController;

	private SunriseExecutionService executionService;

	@Inject
	private Helper helper;

	@Inject
	@Named("Linkage")
	private Tool _linkage;

	private LedColor _ledColor;

	@Inject
	private ITaskLogger _logger;

	@Inject
	private LinkageIOGroup _linkageIOGroup;

	@Inject
	private BodyIOGroup _bodyIOGroup;

	/**
	 * Enumeration for controlling the number of clicks for each button
	 */
	@Inject
	private NaviController naviController;

	@Inject
	private ObserverManager observationManager;

	@Inject
	private LiftMode liftMode;

	@Inject
	private IApplicationControl appControl;

	public static boolean activeAttachButton;
	/**
	 * Members helping with the internal finite state machine in run().
	 */
	private static AppStateMachine _appStateMachine = AppStateMachine.START_STATE;
	private boolean _exitApplication = false;
	public static boolean system_Check_Needed;

	private static ITaskLogger staticLogger;
	private static TCPConnection staticTcpConnection;
	/**
	 * BrakeTestHelper handles the brake test and brake test events. Cyclic
	 * brake tests need to be performed each booting and after a certain cyclic
	 * time. The BrakeTestHelper listens to brake test event, informs if a brake
	 * test is due and allows to execute the test.
	 */
	private BrakeTestHelper _brakeTestHelper;

	private PTP ptpToReadyPosition;

	/**
	 * Ready position when the robot is ready to operate after system check
	 */
	public final static double[] readyPosition = new double[] { 0, 0, 0, 0 - Math.toRadians(90), 0, Math.toRadians(55), 0 };

	public static UserCmd userCommand;

	/**
	 * The incoming message for the TcpConnection to be handled in the
	 * application
	 */
	public static String cmdMsg = "-1;";
	/**
	 * Packet counter for messages that expect an answer. It allows the receiver
	 * to ignore duplicate messages requesting the same answer.
	 */
	private int msgCnt = -1;
	private ArrayList<ArrayList<Training>> motionsList;
	private ExcutionController excutionController;

	@Inject
	IApplicationData appData;
	private ArrayList<Frame> frameList;
	private TCPConnection tcpConnection;
	private MotionMonitor motionMonitor;
	private ArrayList<Frame> nextPosArray;
	private ArrayList<Frame> pathFrames;
	private ArrayList<TorqueSensorData> loggedTorques;
	private ArrayList<Double> loggedWeights;
	protected Frame startPosition;
	protected boolean isRecordPressed = false;
	private boolean exitRunning = false;
	private PathDataRecorder pathDataRecorder;
	private Runtime runtime;
	private Frame readyPositionFrame;
	private int overdueMastering;
	private Thread exControl;
	private String motionType;
	private int cycles;
	private int resistance;
	private int motionNo;
	protected Frame lowestPosition;
	private String max = "250";
	private String med = "200";
	private String min = "150";
	private UdpHelper _udpHelper;
	private ExecutorService _udpReceiverThread;
	private static final int _LOCAL_PORT = 30009;

	@Override
	public void initialize() {

		staticLogger = getLogger();
		staticTcpConnection = this.tcpConnection;
		// trainingType = TrainingTypeEnum.ACTIVE;

		_medController = (MedController) _lbrMed.getController();
		executionService = (SunriseExecutionService) _medController.getExecutionService();

		_brakeTestHelper = new BrakeTestHelper(_medController, _logger);

		_ledColor = new LedColor(getApplicationData(), _medController);

		_linkageIOGroup.setLEDLightGreen(false);
		_linkageIOGroup.setLEDLightGreen(false);

		_linkage.attachTo(_lbrMed.getFlange());
		_linkage.getLoadData().setMass(0.975);

		observationManager = new ObserverManager();
		recordBtObserver();
		// playBtObserver();
		// endBtObserver();

		// workspace monitoring
		// _lbrMed.setESMState("2");
		_logger.info("Min joint positions are:" + _lbrMed.getJointLimits().getMinJointPosition());
		_logger.info("Max joint positions are:" + _lbrMed.getJointLimits().getMaxJointPosition());

		ptpToReadyPosition = ptp(readyPosition).setJointVelocityRel(0.180).setJointAccelerationRel(0.03);

		NaviController.resetButtons(); 

		this.nextPosArray = new ArrayList<Frame>(4);
		this.motionsList = new ArrayList<ArrayList<Training>>(4);

		runtime = Runtime.getRuntime();
		// startPositionTorques = new ArrayList<TorqueSensorData>();
		helper.asyncErrorHandler(appControl);

	}

	@Override
	public void run() {

		while (!_exitApplication) {
			_logger.info("App State Machine: " + _appStateMachine);

			switch (_appStateMachine) {
			case START_STATE:
				runStartState();
				break;
			case MASTERING:
				runMasteringState();
				break;
			case BRAKE_TEST_OPTIONAL:
				runBrakeTestOptionalState();
				break;
			case BRAKE_TEST_HANDLING:
				runBrakeTestHandlingState();
				break;
			case CHECK_ROBOT_STATE:
				runCheckRobotStateState();
				break;
			case REFERENCING:
				runReferencingState();
				break;
			case RUNNING:
				runRunningState();
				break;
			case HANDGUIDING:
				// runHandguidingState();
				break;
			case APP_ERROR:
				_logger.error("Application state machine is in Error state. Closing application!");
				_exitApplication = true;
				break;
			case EXITING:
				_logger.info("Application state machine is in Exiting state. Closing application!");
				_exitApplication = true;
				break;
			default:
				_logger.info("Invalid application state reached! Closing application!");
				_exitApplication = true;
				break;
			}
		}
	}

	public static ITaskLogger getITaskLogger() {
		return staticLogger;
	}

	public static TCPConnection getTcpConnection() {

		return staticTcpConnection;
	}

	/**
	 * In this state it is checked if mastering or a brake test is necessary.
	 */
	private void runStartState() {
		// collision detection

		if (!startTCPCommunication()) {
			_appStateMachine = AppStateMachine.APP_ERROR;
		}

		if (!this._ledColor.isFlashing()) {
			this._ledColor.startFlashing(Color.YELLOW); 
		}

		if (!_lbrMed.isMastered()) {
			_appStateMachine = AppStateMachine.MASTERING;

			/**
			 * check if brake-test necessary
			 */
		} else {
			_lbrMed.setESMState("4");

			switch (_brakeTestHelper.getCurrentState()) {
			case SYSTEM_STARTED:
			case ERROR:
			case FEEDBACK_REQUIRED:

				if (!this._ledColor.isFlashing()) {
					this._ledColor.startFlashing(Color.YELLOW);
				}
				_appStateMachine = AppStateMachine.BRAKE_TEST_HANDLING;
				// _appStateMachine = AppStateMachine.RUNNING;
				break;
			case OK:
				// _appStateMachine = AppStateMachine.BRAKE_TEST_OPTIONAL;
				_appStateMachine = AppStateMachine.RUNNING;
				break;
			case FATAL_ERROR:
				_logger.error("Too many brake-tests failed! Robot movement stopped! Contact customer service!");
				_appStateMachine = AppStateMachine.APP_ERROR;
				break;
			default:
				_logger.error("Unexpected state of brake-test monitoring! Consider rebooting the robot!");
				_appStateMachine = AppStateMachine.APP_ERROR;
				break;
			}
		}
	}

	/**
	 * In this state all non-mastered axis can be mastered.
	 */
	private void runMasteringState() {
		if (!SafetyStopType.NOSTOP.equals(_lbrMed.getSafetyState().getSafetyStopSignal())) {
			_logger.error("Can't master, because a Safety-Stop is active! A reason could be that the SAFETY MUTING " + "BUTTON is not pressed.\n"
					+ "It is used to mute any Safety rows that would be violated if the robot " + "is not masterd.");
			_appStateMachine = AppStateMachine.APP_ERROR;
			return;
		}

		// this._ledColor.startFlashing(Color.YELLOW);
		overdueMastering = 0;
		if (!handleMastering()) {
			_appStateMachine = AppStateMachine.APP_ERROR;
			return;
		}
		_appStateMachine = AppStateMachine.START_STATE;

		// _appStateMachine = AppStateMachine.EXITING;
		;
	}

	/**
	 * @return True, if mastering was successful and false, otherwise.
	 */
	private boolean handleMastering() {
		Mastering mastering = new Mastering(_lbrMed);

		masterExceededLimit();

		overdueMastering++;

		for (int i = 0; i < _lbrMed.getJointCount(); i++) {
			if (!mastering.isAxisMastered(i) && !mastering.masterAxis(i)) {
				_logger.error("Mastering failed! in axis: " + i);
				if (overdueMastering == 5) {
					return false;
				}
				ThreadUtil.milliSleep(500);
				return handleMastering();
			}
		}
		return true;
	}

	private boolean masterExceededLimit() {
		Mastering mastering = new Mastering(_lbrMed);
		for (int i = 0; i < _lbrMed.getJointCount(); i++) {

			if (_lbrMed.getCurrentJointPosition().get(i) > _lbrMed.getJointLimits().getMaxJointPosition().get(i)
					|| _lbrMed.getCurrentJointPosition().get(i) < _lbrMed.getJointLimits().getMinJointPosition().get(i)) {

				if (!mastering.isAxisMastered(i) && !mastering.masterAxis(i)) {
					_logger.error("Mastering failed! in axis: " + i);
					return false;
				}

			}

		}

		return true;
	}

	/**
	 * In this state the brake test is only optional. It can be executed or
	 * ignored.
	 */
	private void runBrakeTestOptionalState() {
		_appStateMachine = AppStateMachine.BRAKE_TEST_HANDLING;
	}

	/**
	 * In this state the brake test can be executed or if possible also
	 * postponed.
	 */
	private void runBrakeTestHandlingState() {

		if (!_brakeTestHelper.handleBrakeTest(_lbrMed)) {
			_appStateMachine = AppStateMachine.APP_ERROR;
			return;
		}
		_appStateMachine = AppStateMachine.CHECK_ROBOT_STATE;

		/*
		 * if (!_brakeTestHelper.postponeBrakeTest()) { _appStateMachine =
		 * AppStateMachine.APP_ERROR; return; } _appStateMachine =
		 * AppStateMachine.CHECK_ROBOT_STATE;
		 */
	}

	/**
	 * In this state the robot's state is checked if all conditions for normal
	 * use are fulfilled.
	 */
	private void runCheckRobotStateState() {
		// check if robot is ready for running the main application steps
		RobotStateAggregator stateAggregator = new RobotStateAggregator(_lbrMed);
		if (stateAggregator.isReadyForNormalUse()) {
			_appStateMachine = AppStateMachine.RUNNING;
			return;
		}

		// go through failedConditions and react accordingly
		List<FailedConditionForNormalUse> failedConditions = stateAggregator.getFailedConditionsForNormalUse();

		if (failedConditions.contains(FailedConditionForNormalUse.NOT_MASTERED)) {
			_logger.error("ROBERT: Unexpected failed condition: Axes should be mastered at this point!");
			_appStateMachine = AppStateMachine.APP_ERROR;
			return;
		}

		if (failedConditions.contains(FailedConditionForNormalUse.NOT_POSITION_REFERENCED)
				|| failedConditions.contains(FailedConditionForNormalUse.NOT_GMS_REFERENCED)) {
			_lbrMed.setESMState("4");
			_appStateMachine = AppStateMachine.REFERENCING;
			return;
		}

		String text = "ROBERT: Robot not ready for normal use. Can't resolve: ";
		for (FailedConditionForNormalUse failedCond : stateAggregator.getFailedConditionsForNormalUse()) {
			text = text + failedCond.toString() + "; ";
		}
		_logger.error(text);
		_appStateMachine = AppStateMachine.APP_ERROR;
	}

	/**
	 * In this state the robot's position and torque sensors can be referenced.
	 */
	private void runReferencingState() {

		if (!handleReferencing()) {
			_logger.error("Position and GMS referencing failed!");
			_appStateMachine = AppStateMachine.APP_ERROR;
			return;
		}
		_logger.info("Position and GMS referencing successful!");
		_appStateMachine = AppStateMachine.CHECK_ROBOT_STATE;

		// test
		// _appStateMachine = AppStateMachine.RUNNING;

	}

	/**
	 * Execution of referencing motions for position and torque sensors.
	 * 
	 * @return True, if referencing was successful and false, otherwise.
	 */
	private boolean handleReferencing() {
		_lbrMed.setESMState("4");
		ReferencingHelper referencing = new ReferencingHelper(_lbrMed, _medController, _logger);

		return referencing.doPositionAndGMSReferencing();
	}

	private void runRunningState() {
		/* ************************************************************* */
		/* MAIN APPLICATION IS RUNNING IN THIS STATE AFTER SYSTEM CHECK */
		/* ************************************************************* */
		_linkage.move(ptpToReadyPosition);

		if (!startMotionMonitor()) {
			_appStateMachine = AppStateMachine.APP_ERROR;
		}
		this.readyPositionFrame = this._lbrMed.getCurrentCartesianPosition(this._linkage.getFrame("/RecTCP"), World.Current.getRootFrame());
		// Workspace monitoring is set back
		_lbrMed.setESMState("2");

		// turn off the lights
		if (_ledColor.isFlashing()) {
			_ledColor.stopFlashing();
		}
		try {
			tcpConnection.sendMessage(UserCmd.READY.toString());
		} catch (IOException e) {
			this._logger.info("Failed to send ready status to the control interface " + e);
		}

		activeAttachButton = true;

		while (!exitRunning) {

			if (activeAttachButton && _lbrMed.getSafetyState().getEnablingDeviceState() == EnablingDeviceState.HANDGUIDING && userCommand != UserCmd.SAFETY_STOP) {
				int cnt = 1;
				while (!_linkageIOGroup.getRecordButton() && _lbrMed.getSafetyState().getEnablingDeviceState() == EnablingDeviceState.HANDGUIDING) {
					ThreadUtil.milliSleep(10);

					if (cnt == 10) {
						this._logger.info("Attach pressed -- --- -- - -- - -");
						_linkage.getLoadData().setMass(0.975);

						_lbrMed.setESMState("2");
						if (_ledColor.isFlashing()) {
							_ledColor.stopFlashing();
						}

						_linkageIOGroup.setLEDLightGreen(false);
						_linkageIOGroup.setLEDLightRed(false);

						if (excutionController != null) {
							excutionController.end();

						}
						// if (pathDataRecorder != null) {
						// pathDataRecorder = null;
						// }
						/**
						 * reset everything 
						 */
						NaviController.resetButtons();

						this.motionsList.clear();
						nextPosArray.clear();

						_logger.info("ROBERT: all data has been reset ");
						/**
						 * sending attach pressed to the control interface
						 */
						try {
							tcpConnection.sendMessage(UserCmd.ATTACH_PRESSED.toString());
						} catch (IOException e) {
							this._logger.info("Failed to send REC_DONE to the control interface " + e);
						}

						helper.moveMe();

						/**
						 * check that status of robot
						 */

						// reset the userCmd
						userCommand = UserCmd.NONE;
					} else if (_lbrMed.getSafetyState().getEnablingDeviceState() != EnablingDeviceState.HANDGUIDING)
						break;
					cnt++;
				}

			}
			// handling the incoming command messages
			if (cmdMsg != null) {
				/**
				 * return if the command is a system check
				 */
				if (handleMessage(cmdMsg) == UserCmd.SYS_CHECK) {
					_appStateMachine = AppStateMachine.CHECK_ROBOT_STATE;
					userCommand = UserCmd.NONE;
					return;
				}
			}

			ThreadUtil.milliSleep(100);
		}

	}

	/**
	 * Handling the received message which has the following format: MSG_CNT :
	 * COMMAND : MODE : MOTION_NO : MOTION_TYPE : CYCLES : BREAK_TIMES :
	 * RESISTANCES Obsolete [MSG_CNT;COMMAND;MODE;EX_NO;CYCLE;RESISTANCE]
	 * 
	 * @param received
	 */
	private UserCmd handleMessage(String received) { 

		if (userCommand == UserCmd.ESTOP) {
			_logger.info("UserCommand: " + userCommand);
			userCommand = UserCmd.SAFETY_STOP;
			if (motionsList != null) {
				this.motionsList.clear();
			}
			if (nextPosArray != null) {
				nextPosArray.clear();
			}

			NaviController.resetButtons();
			
//			if (Guided.motionContainer != null) {
//				while (!Guided.motionContainer.isFinished()) {
//					Guided.motionContainer.cancel();
//					ThreadUtil.milliSleep(100);
//
//				}
//			}
//
//			if (Active.motionContainer != null) {
//				while (!Active.motionContainer.isFinished()) {
//					Active.motionContainer.cancel();
//					ThreadUtil.milliSleep(100);
//
//				}
//			}

			return userCommand;
		}

		String[] message = new String(received).split(";");

		int receivedMsgCnt = Integer.parseInt(message[0]);

		if (this.msgCnt != receivedMsgCnt) {
			this.msgCnt = receivedMsgCnt;
			return handleCommand(message);

		}
		return userCommand;
	}

	/**
	 * handle the commands from the external interface. MSG_CNT : COMMAND : MODE
	 * : MOTION_NO : MOTION_TYPE : CYCLES : BREAK_TIMES : RESISTANCES
	 * 
	 * @param message
	 */
	private UserCmd handleCommand(String[] message) {

		userCommand = UserCmd.fromString(message[1]);

		switch (userCommand) {

		case ADD:
			_linkageIOGroup.setLEDLightGreen(true);
			_linkageIOGroup.setLEDLightRed(false);
			/**
			 * add motion to the array only if motion number is >0
			 */

			if (Integer.parseInt(message[3]) > 0) { 

				/**
				 * set the kickoff position and the end position
				 */
				this.nextPosArray.add(frameList.get(0));
				// this.nextPosArray.add(frameList.get(frameList.size() - 1));

				/**
				 * create an object of a motion and connect the recording with
				 * the motion object
				 */
				createMotionArray();

			}
			break;
		case UPDATE: // TODO add Integer.parseInt(message[4]) for vein pump
			if (excutionController != null) {
				excutionController.update(Integer.parseInt(message[2]), Integer.parseInt(message[3])); 
			}

			break;

		case START:
			// MSG_CNT : COMMAND : MODE : MOTION_TYPE : CYCLES : RESISTANCES

			motionType = message[3].toString();
			cycles = Integer.parseInt(message[4]);
			resistance = Integer.parseInt(message[5]);

			excutionController = new ExcutionController(this.motionsList, this.nextPosArray, this.tcpConnection, this._linkageIOGroup,
					this._bodyIOGroup, _ledColor, this._logger, this._lbrMed, startPosition, _linkage, this.appData, this.lowestPosition);

			excutionController.setupTrainingType(1, motionType, cycles, resistance);

			exControl = new Thread(excutionController);
			this.tcpConnection.setExecutionController(excutionController);

			userCommand = UserCmd.NONE;
			excutionController.setTransitionBool(true);
			
			if (exControl != null) {
				exControl = new Thread(excutionController);
			}
			exControl.start();
			motionMonitor.setMotionControler(excutionController);
			break;
		case REMOVE:
			// MSG_CNT : COMMAND : MODE : MOTION_NO : MOTION_TYPE
			try {
				if (message[2].contains("R")) {
					this.motionsList.remove(Integer.parseInt(message[3])-1);
					this.nextPosArray.remove(Integer.parseInt(message[3])-1);
				}
			} catch (Exception e) {
				this._logger.info("ROBERT: Something went wrong when tried to remove a motion: " + e);
			}
			userCommand = UserCmd.ADD;
			break;
		case LOG:
			//TODO should this be add to be able to add a new motion after log where motion list < 4 motions
			userCommand = UserCmd.NONE;
			
			_linkageIOGroup.setLEDLightGreen(false);
			_linkageIOGroup.setLEDLightRed(false);
			// _ledColor.startFlashing(Color.YELLOW);
			/*
			 * duplicate the path for forth and back without changing the
			 * original
			 */
			// this.velArray = helper.velocityCalculation(frameList, 5.0);
			ArrayList<Frame> filtredTrac = helper.filterTrack(this.frameList, 5.0);

			ArrayList<Frame> logArray = new ArrayList<Frame>(filtredTrac);

			Collections.reverse(logArray);
			pathDataRecorder = new PathDataRecorder(logArray, this.appData, this._linkageIOGroup, this._linkage, this._ledColor, this._logger,
					this._lbrMed, this._bodyIOGroup, this.tcpConnection, appControl);
			Thread thread = new Thread(pathDataRecorder);
			thread.start();
			break;
		case PLAY:
			// MSG_CNT ; COMMAND ; MOTION_NO ; MOTION_TYPE ; CYCLES ;
			// RESISTANCES
			motionType = message[3].toString();
			cycles = Integer.parseInt(message[4]);
			resistance = Integer.parseInt(message[5]);
			motionNo = Integer.parseInt(message[2]);
			excutionController.setupTrainingType(motionNo, motionType, cycles, resistance);
			excutionController.setTransitionBool(false);

			if (exControl != null) {
				exControl = new Thread(excutionController);
			}
			exControl.start();
			motionMonitor.setMotionControler(excutionController);
			break;

		case PAUSE:
			if (pathDataRecorder != null) { 
				pathDataRecorder.pause();

			}
			if (excutionController != null) {
				excutionController.pause();
			}
			break;
		case RESUME:
			if (pathDataRecorder != null) {
				pathDataRecorder.resume();
			}
			if (excutionController != null) {
				excutionController.resume();
			}
			break;
		case END:
			if (excutionController != null) {
				excutionController.end(); 
			}

			 if (pathDataRecorder != null) {
			 pathDataRecorder.end();
			 }

			break;
		case SKIP:
			if (excutionController != null) { 
				excutionController.skipMotion(); 			
			}
			break;	
		case ROB_TRANSPORT:
			PTP robTransportPosition = ptp(0, Math.toRadians(55), 0, -Math.toRadians(90), Math.toRadians(90), 0, Math.toRadians(90));
			transportPosition(robTransportPosition);
			break;
		case KUKA_TRANSPORT:
			PTP kukaTransportPosition = ptp(0, Math.toRadians(25), 0, Math.toRadians(90), 0, 0, 0);
			transportPosition(kukaTransportPosition);
			break;
		case SYS_CHECK:
//			if (excutionController != null) {
//				excutionController.end(); 
//			}
//			
//			 if (pathDataRecorder != null) {
//			 pathDataRecorder.end();
//			 }			 
				_linkageIOGroup.setLEDLightGreen(false);
				_linkageIOGroup.setLEDLightRed(false);
				
				if (!_ledColor.isFlashing()) {
				_ledColor.startFlashing(Color.YELLOW);
				}
				this._lbrMed.setESMState("4");
			break;
		case EXERCISE_DONE:
			/**
			 * reactivate the recording button after exercise done
			 */
			userCommand = UserCmd.ADD;
			/**
			 * reset everything
			 */
			this.motionsList.clear();
			this.nextPosArray.clear();

			break;

		case RERECORD:
			userCommand = UserCmd.ADD;
			_ledColor.stopFlashing();
			_linkageIOGroup.setLEDLightRed(false);
			_linkageIOGroup.setLEDLightGreen(true);
			break;

		case REBOOT:
			String[] reboot = new String[] { "D:\\Programme\\reboot.cmd" };

			try {
				runtime.exec(reboot);
			} catch (IOException e) {
				_logger.info("ROBERT: Error happened, cannot reboot the ROBERT device" + e);
			}
			break;
		case SHUTDOWN:
			String[] shutdown = new String[] { "D:\\Programme\\shutdown.cmd" };
			try {
				runtime.exec(shutdown);
			} catch (IOException e) {
				_logger.info("ROBERT: Error happened, cannot shotdown the ROBERT device" + e);
			}
		case UNMASTER:
			invalidateMastering();
			try {
				this.tcpConnection.sendMessage(UserCmd.UNMASTERED.toString());
			} catch (IOException e) {
				this._logger.info("ROBERT: unable to send Unmastered infor to the CIU");
			}
			break;
		case SPEED:
			String speed = message[2].toString(); 
			if (speed.equals("MAX")) {
			this.appData.getProcessData("velo").setValue(this.max);	
			_logger.info("ROBERT: the speed level is set to: " + speed + " :" + this.max);
			}else if (speed.equals("MID")) {
			this.appData.getProcessData("velo").setValue(this.med);
			_logger.info("ROBERT: the speed level is set to: " + speed + " :" + this.med);
			}else if (speed.equals("MIN")){
			this.appData.getProcessData("velo").setValue(this.min);
			_logger.info("ROBERT: the speed level is set to: " + speed + " :" + this.min);
			}
			
			userCommand = UserCmd.ADD;
			break;

		case TRANSITION_START:
			motionNo = Integer.parseInt(message[2]);
			excutionController.transitionMotionTo(motionNo);
			excutionController.setTransitionBool(true);
			
			if (exControl != null) {
				exControl = new Thread(excutionController);
			}
			exControl.start();
			break;
		case REPEAT:
			userCommand = UserCmd.ADD;
			break;
		}

		return userCommand;

	}

	public void invalidateMastering() {
		Mastering mastering = new Mastering(this._lbrMed);

		for (int i = 0; i <= 6; i++) {
			if (mastering.isAxisMastered(i)) {
				mastering.invalidateMastering(i);
			}
		}
	}

	/**
	 * Transport position of ROBERT or KUKA
	 * 
	 * @param position
	 */
	private void transportPosition(PTP position) {
		this._lbrMed.setESMState("4");
		position.setJointVelocityRel(0.25);
		this._lbrMed.move(position);
		this._lbrMed.setESMState("2");
	}

	/**
	 * MSG_CNT : COMMAND : MODE: MOTION_NO : MOTION_TYPE : CYCLES : BREAK_TIMES
	 * : RESISTANCES
	 */
	private void createMotionArray() {
		ArrayList<Training> motionMode = new ArrayList<Training>();
		preparePathDataForGuided();

		Guided guided = new Guided(this._lbrMed, this.pathFrames, this.loggedTorques, this.loggedWeights, this.appData, this._linkageIOGroup,
				this._bodyIOGroup, this._ledColor, this._linkage, this._logger, appControl, this.tcpConnection);
		guided.setTcpConnection(tcpConnection);
		motionMode.add(guided);

		Active active = new Active(this.frameList, this._logger, this._lbrMed, this.appControl, this.appData, this._linkageIOGroup, _bodyIOGroup,
				this._ledColor, this._linkage, this.readyPositionFrame, this.tcpConnection);
		active.setlbrMed(_lbrMed);
		active.setTcpConnection(tcpConnection);
		motionMode.add(active);
		
		motionsList.add(motionMode); 
		this._logger.info("ROBERT: the motion size is: " + motionsList.size());

	}

	private void preparePathDataForGuided() {

		this.pathFrames = this.pathDataRecorder.getPathFrames();

		this.loggedTorques = this.pathDataRecorder.getPathRegisteredTorques();

		this.loggedWeights = this.pathDataRecorder.getPathWeights();
	}

	private void recordBtObserver() {

		if (observationManager != null) {
			observationManager = new ObserverManager();
		}

		IRisingEdgeListener risingEdgeListener = new IRisingEdgeListener() {

			@Override
			public void onRisingEdge(ConditionObserver conditionObserver, Date time, int missedEvents) {

				isRecordPressed = true;
				// ThreadUtil.milliSleep(100);

				if (naviController.getBtClickingState(ButtonTypeEnum.RECORD) == 0 && userCommand != UserCmd.SAFETY_STOP) {
					naviController.setBTClickingState(ButtonTypeEnum.RECORD, 1);

					while (_linkageIOGroup.getRecordButton()) {
						ThreadUtil.milliSleep(100);
					}
					/*
					 * Log the lowest position above the bad
					 */
					ObjectFrame recTCP = _linkage.getFrame("RecTCP");
					lowestPosition = _lbrMed.getCurrentCartesianPosition(recTCP);

					userCommand = UserCmd.NONE;

					boolean status = liftMode.liftup();
					startPosition = liftMode.getStartFrame();
					isRecordPressed = false;
					if (status) {

						if (_lbrMed.getSafetyState().getEnablingDeviceState() != EnablingDeviceState.HANDGUIDING || _linkageIOGroup.getRecordButton()) {
							_linkageIOGroup.setLEDLightGreen(true);

							// activating the recording
							userCommand = UserCmd.ADD;

						} else {
							_linkageIOGroup.setLEDLightGreen(false);
							_linkageIOGroup.setLEDLightRed(false);
						}

					} else {
						// returned -1, the Robert can not lift the load
						_logger.info("Robert: Either no load or overload detected");
						_linkageIOGroup.setLEDLightGreen(true);
						_linkageIOGroup.setLEDLightRed(true);
						isRecordPressed = false;
					}

				} else if (userCommand == UserCmd.ADD) {

					// torque_kickoff = _lbrMed.getExternalTorque();

					if (!_ledColor.isFlashing()) {
						_linkageIOGroup.setLEDLightGreen(true);
						_linkageIOGroup.setLEDLightRed(false);
					}

					// isRecordPressed = false;
					_lbrMed.setESMState("2");
					try {
						tcpConnection.sendMessage(UserCmd.REC_STARTED.toString());
					} catch (IOException e) {
						_logger.info("Failed to send REC_STARTED to the control interface " + e);
					}

					support();
					frameList = helper.pathRecorder();
					setFullWeight();

					_logger.info("ROBERT: The recorded path: " + frameList.toString());

					ThreadUtil.milliSleep(100);
					try {
						tcpConnection.sendMessage(UserCmd.REC_DONE.toString());
					} catch (IOException e) {
						_logger.info("Failed to send REC_DONE to the control interface " + e);
					}
					isRecordPressed = false;

					if (frameList.get(0).distanceTo(frameList.get(frameList.size() - 1)) < 10) {
						try {
							tcpConnection.sendMessage(UserCmd.INVALID_RECORDING.toString());
						} catch (IOException e) {
							_logger.info("ROBERT: Failed to send logging canceled to the control interface" + e);
						}
					}
				}

			}
		};

		Input recBt = _linkageIOGroup.getInput("RecordButton");

		ICondition recBtCondition = new BooleanIOCondition(recBt, true);

		ConditionObserver recBtObserver;
		try {
			recBtObserver = observationManager.createConditionObserver(recBtCondition, NotificationType.EdgesOnly, risingEdgeListener);
			recBtObserver.enable();
		} catch (Exception e) {
			_logger.info("ROBERT: Failed to start condition observer " + e);
		}

	}

	private void support() {
		double supportedMass = calcSupportedWeight(liftMode.getWeight());
		_linkage.getLoadData().setMass(supportedMass);
	}

	private double calcSupportedWeight(double weight) {
		// remove 20% of the weight
		double calcSupportedMass = weight * 0.80;
		// logger.info("Robert: actual weight: " + weight);
		_logger.info("Robert: actual weight: " + weight + "\n The supported weight(80%) = " + calcSupportedMass);
		return calcSupportedMass;
	}

	private void setFullWeight() {
		_linkage.getLoadData().setMass(liftMode.getWeight());
	}

	/**
	 * Start the TCP communication to the control interface
	 */
	private boolean startTCPCommunication() {
        
		if (!startUdpCommunication())
        {
        	return false;
        }
        
		if (tcpConnection == null) {

			tcpConnection = new TCPConnection(this._logger);

			if (!tcpConnection.initTCP()) { 

				return false;
			}

			Thread t = new Thread(tcpConnection);
			t.start();

			/**
			 * static variable to communicated with in the background task
			 */
			staticTcpConnection = tcpConnection;

		}

		return true;

	}

	/**
     * Start the UDP communication to external client (e.g. ExampleMedGUI)
     */
    private boolean startUdpCommunication()
    {
        if (null == _udpHelper)
        {
            _udpHelper = new UdpHelper(_LOCAL_PORT, "DefaultApplication", _logger, _bodyIOGroup);
            if (!_udpHelper.initUDP())
            {
                return false;
            }
            _logger.info("SAS: UDP connection established");
            // create additional thread to receive UDP packages the code of the thread is defined in UdpHelper.run()
            _udpReceiverThread = Executors.newSingleThreadExecutor();
            _udpReceiverThread.execute(_udpHelper);
            _logger.info("SAS: UDP thread started");
        }

        return true;
    }
	private boolean startMotionMonitor() {
		if (motionMonitor == null) {

			motionMonitor = new MotionMonitor(_logger, executionService, tcpConnection, _lbrMed, _medController, _linkageIOGroup, _ledColor);
			Thread t = new Thread(motionMonitor);
			t.start();
		}
		return true;
	}

	/**
	 * Dispose method called when this application is unloaded.
	 */
	@Override
	public void dispose() {

		if (tcpConnection != null) {
			tcpConnection.terminate();
		}

		if (excutionController != null) {
			excutionController.dispose();
		}

		if (_brakeTestHelper != null) {
			_brakeTestHelper.dispose();
		}

		exitRunning = true;
		_appStateMachine = AppStateMachine.APP_ERROR;

		super.dispose();
	}
}