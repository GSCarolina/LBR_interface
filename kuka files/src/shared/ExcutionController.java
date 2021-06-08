package shared;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.spl;
import guidedMode.Guided;

import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import main.Robert_2_3;
import shared.LedColor.Color;
import util.UserCmd;
import activeMode.Active;
import backgroundTask.StateHelper;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.BodyIOGroup;
import com.kuka.generated.ioAccess.LinkageIOGroup;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.IFiredConditionInfo;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.ioModel.Input;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineMotionCP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.task.ITaskLogger;

public class ExcutionController extends Training implements Runnable {

	private ArrayList<ArrayList<Training>> motionList;
	private Training aTraningType;
	private ExecutorService executorService;
	private ITaskLogger logger;
	private TCPConnection tcpConnection;
	private LinkageIOGroup linkageIOGroup;
	private ArrayList<Frame> nextPosArray;

	private LBR robot;
	private BodyIOGroup bodyIOGroup;

	private Torques recTorques;
	private ICondition bounce;

	private UserCmd userStatus;
	private int motionTransitionNo;
	private ArrayList<Training> traningSet;

	private int cycles;
	private int resistance;

	private Training lastMovType;

	private int lastMotionNo;
	private Frame lowestPosition;
	private boolean transitionBool;
	private boolean hasSkipped;

	/**
	 * Execution
	 * 
	 * @param motionsList
	 * @param _nextPosArray
	 * @param _tcpConnection
	 * @param _linkageIOGroup
	 * @param _bodyIOGroup
	 * @param _ledColor
	 * @param _logger
	 * @param _robot
	 * @param _startPosition
	 * @param _linkage
	 * @param _appData
	 * @param lowestPosition
	 */
	public ExcutionController(ArrayList<ArrayList<Training>> motionsList, ArrayList<Frame> _nextPosArray, TCPConnection _tcpConnection,
			LinkageIOGroup _linkageIOGroup, BodyIOGroup _bodyIOGroup, LedColor _ledColor, ITaskLogger _logger, LBR _robot, Frame _startPosition,
			Tool _linkage, IApplicationData _appData, Frame _lowestPosition) {
		this.logger = _logger;
		this.motionList = motionsList;
		this.tcpConnection = _tcpConnection;
		this.linkageIOGroup = _linkageIOGroup;
		this.bodyIOGroup = _bodyIOGroup;
		this.nextPosArray = _nextPosArray;
		this.robot = _robot;
		linkage = _linkage;
		ledColor = _ledColor;
		this.appData = _appData;
		this.lowestPosition = _lowestPosition;

		rotationalSoft = getRotationalSoft();
		softBounce = getSoftBounce();
		this.lastMotionNo = -1;
	}
	
	public ExcutionController(int _motionNo){
		this.executorService = Executors.newSingleThreadExecutor();
	}

	/**
	 * Setup the current motion with the motion parameters, and setup the
	 * transition criteria for the motion from these parameters
	 * 
	 * @param _motionNo
	 * @param _motionType
	 * @param _cycles
	 * @param _resistance
	 */
	public void setupTrainingType(int _motionNo, String _motionType, int _cycles, int _resistance) {
		this.executorService = Executors.newSingleThreadExecutor();
		/**
		 * current motion id can be 0 or 1 to indicate guided or active object
		 * in the array
		 */
		traningSet = motionList.get(_motionNo - 1); 

		if (this.motionList.get(_motionNo - 1).size() > 1) {
			if (_motionType.equals("G")) {
				this.aTraningType = traningSet.get(0);

			} else {
				this.aTraningType = traningSet.get(1);
			}
		} else {
			// only one motionType inside the current position
			this.aTraningType = traningSet.get(0);
		}

		this.motionTransitionNo = _motionNo;
		this.cycles = _cycles;
		this.resistance = _resistance;
		
		if (this.aTraningType instanceof Active) {
			Robert_2_3.activeAttachButton = true;
		}
		this.lastMovType = this.aTraningType;
	}

	@Override
	public void run() {
		
		double velo = this.appData.getProcessData("velo").getValue();
		
		this.robot.setDeviceCartesianVelocityLimit((int) velo);
		
		if (isTransition()) {
			logger.info("ROBERT: received command TRANSITION_START");
			if (!performMotionTransition(motionTransitionNo)) {
				//in case of safety stop
				this.robot.deactivateDeviceCartesianVelocityLimit();
				return;
			}

			try {
				tcpConnection.sendMessage(UserCmd.TRANSITION_DONE.toString());
			} catch (IOException e) {
				logger.info("ROBERT: Cannot sent the message Transition_Done to the CIU: " + e); 
			}
			this.robot.deactivateDeviceCartesianVelocityLimit();
		} else { 
			
		hasSkipped = false;
		StateHelper.isEstopPressed = false;

		recTCP = linkage.getFrame("/RecTCP");

		linkageIOGroup.setLEDLightGreen(false);
		linkageIOGroup.setLEDLightRed(false);

		// counter for all motion types running in the same motion

		double[] torques = new double[] { 220, 220, 150, 150, 90, 35, 35 };

		recTorques = new Torques(robot, logger, null);

		bounce = recTorques.prepareConditon(torques);
		
		bodyIOGroup.setPauseControl(false);
		bodyIOGroup.setEnd(false);

		linkageIOGroup.setLEDLightGreen(true);
		linkageIOGroup.setLEDLightRed(false);

		/**
		 * This methods return string after it has finished
		 */
		Callable<Integer> callable = new Callable<Integer>() {

			@Override
			public Integer call() throws Exception {
				/**
				 * returned form training when attach was pressed
				 */
				if (aTraningType.play(cycles, resistance) == 1) {
					// tcpConnection.sendMessage(UserCmd.ATTACH_PRESSED.toString());
					logger.info("ROBERT: motion is intrupted due to pressing either 'SKIP MOTION', 'END', 'ATTACH', or 'E-Stop'");
					linkageIOGroup.setLEDLightGreen(false);  					
					linkageIOGroup.setLEDLightRed(false); 
					return 1;
				}
				
				return 0;
			}
		};

		try {
			
		Future<Integer> future = executorService.submit(callable);

			if (future.get() == 0) {

				if (ledColor.isFlashing()) {
					ledColor.stopFlashingNow();
				}
				this.linkageIOGroup.setLEDLightGreen(false);
				this.linkageIOGroup.setLEDLightRed(false);

				helperFunction();

				this.logger.info("SET is Done");

				if (hasSkipped) {
					try {
						tcpConnection.sendMessage(UserCmd.SKIP_DONE.toString());
					} catch (IOException e) {
						this.logger.info("ROBERT: Cannot sent the message Transition_Done to the CIU: " + e);
					} 
				}
				else{
					 			
				/**
				 * send SET_done to the control interface
				 */
				try {
					tcpConnection.sendMessage(UserCmd.SET_DONE.toString());
				} catch (IOException e) {
					logger.info("ROBERT: failed to send SET_DONE to the CIU: " + e);
					e.printStackTrace();
				}
				
				}
				executorService.shutdown();
			} else if (future.get() == 1) {

				helperFunction();
				Robert_2_3.userCommand = UserCmd.NONE;
				//executorService.shutdown();
				
				if (hasSkipped) {
					try {
						tcpConnection.sendMessage(UserCmd.SKIP_DONE.toString());
					} catch (IOException e) {
						this.logger.info("ROBERT: Cannot sent the message Transition_Done to the CIU: " + e);
					} 
				}
				return;
			}
		} catch (InterruptedException e) {
			this.logger.info("ROBERT: Failed to execution controller is interrupted: " + e);
		} catch (ExecutionException e) {
			this.logger.info("ROBERT: Failed to execute --> " + e);
		}
		}
	}

	private void helperFunction() {

		if (this.aTraningType instanceof Guided) {
			/**
			 * end at the end position and send activity percentage to the
			 * control interface
			 */
			// this.training.returnToEndPosition();

			double persent = ((Guided) this.aTraningType).activePercentage();
			try {
				tcpConnection.sendMessage(UserCmd.ACTIVITY_PERCENT.toString() + ";" + persent);
			} catch (IOException e) {
				logger.info("ROBERT: Could not send percentage value to the external control" + e);
			}

		} else if (this.aTraningType instanceof Active) {
			String maxKgF = ((Active) this.aTraningType).getKgForce();
			logger.info("ROBERT: the max force and the average is : " + maxKgF);
			try {
				tcpConnection.sendMessage(maxKgF);
			} catch (IOException e) {
				logger.info("ROBERT: Could not send max pressure value to the external control" + e);
			}

			//TODO find out why it does not run this command after safety stop at the end of a cycle			
			
		}
		try {
			recTCP.move(positionHold(new PositionControlMode(), 100, TimeUnit.MILLISECONDS));
		} catch (Exception e) {
			logger.info("ROBERT: ExecutioinController failed to run position hold of the helper function" + e);
		}

		Robert_2_3.activeAttachButton = true;
		
		this.robot.deactivateDeviceCartesianVelocityLimit();
	}

	/**
	 * SkipMotion function does the same as end(), which stop the motion, but it
	 * does not reset the motion array.
	 */

	public void skipMotion() {

		if (this.aTraningType != null) {
			this.hasSkipped = true;
			this.aTraningType.skipMotion();
			
		}
	}

	public void end() {

		motionList.clear();
		logger.info("ROBERT: all data was reset");

		if (this.aTraningType != null) {
			this.aTraningType.end();
		}
		//executorService.shutdown();
		dispose();
	}
 
	public void update(int cycle, int resistance) {

		if (this.aTraningType != null) {
			this.aTraningType.update(cycle, resistance); 
		}
	}

	public void pause() {
		if (this.aTraningType != null) {
			this.aTraningType.pause();
		}

	}

	public void resume() {

		this.bodyIOGroup.setPauseControl(false);

		if (this.aTraningType != null) {
			this.aTraningType.resume();
		}
	}

	/**
	 * Reverse the array of frames and returns the spline motion
	 * 
	 * @param array
	 * @return {@link SplineMotionCP}
	 */
	protected Spline getReturningSplinePath(ArrayList<Frame> array) {

		Collections.reverse(array);
		ArrayList<IMotion> motions = new ArrayList<IMotion>();

		for (int i = 0; i < array.size(); i++) {

			motions.add(spl(array.get(i)));
		}

		return new Spline(motions.toArray(new SplineMotionCP[motions.size()]));
	}

	public void dispose() {

		if (executorService != null) {
			try {
				executorService.shutdown();
				executorService.awaitTermination(5, TimeUnit.SECONDS);
			} catch (InterruptedException e) {
				this.logger.warn("Interruption in shutting down the event worker (ExcutionController)!", e);
			}
			executorService.shutdownNow();
		}
	}


	/**
	 * Perform motion transition
	 * 
	 * @param _motionNo
	 * @return
	 */
	private boolean performMotionTransition(int _motionNo) {
		 motionTransitionNo = _motionNo;
		
		Input playBtn = bodyIOGroup.getInput("PlayPauseButton");
		BooleanIOCondition patientButton = new BooleanIOCondition(playBtn, true);
		
		if (motionTransitionNo != lastMotionNo || (lastMovType != null && lastMovType instanceof Active)) {
			lastMotionNo = motionTransitionNo;

			double velo = appData.getProcessData("velo").getValue();
			double acce = appData.getProcessData("acce").getValue();
			double jerk = appData.getProcessData("jerk").getValue();

			robot.setDeviceCartesianVelocityLimit(200);

			Frame motionFrame = nextPosArray.get(motionTransitionNo - 1);

			Spline splineMotion;
//			double distanceToMotionFrame = robot.getCurrentCartesianPosition(recTCP).copy().setZ(0).distanceTo(motionFrame.copy().setZ(0));
//			double distanceToReferenceFrame = robot.getCurrentCartesianPosition(recTCP).copy().setZ(0).distanceTo(this.lowestPosition.copy().setZ(0));

			logger.info("dis frame: " +  robot.getCurrentCartesianPosition(recTCP).getZ());
			logger.info("low frame: " +  lowestPosition.getZ());
			logger.info("motion frame: " +  motionFrame.getZ());
			
			if (motionFrame.getZ() < lowestPosition.getZ() && robot.getCurrentCartesianPosition(recTCP).getZ() > this.lowestPosition.getZ()) {

				logger.info("Lowest: " + lowestPosition.getZ() + "<> current: " + motionFrame.getZ()); 

				Frame transitionFrame;
				transitionFrame = motionFrame.copy();
				transitionFrame.setZ(robot.getCurrentCartesianPosition(recTCP).getZ());
				splineMotion = new Spline(spl(transitionFrame), spl(motionFrame));

			} else if (robot.getCurrentCartesianPosition(recTCP).getZ() < lowestPosition.getZ() 
					&& motionFrame.getZ() > lowestPosition.getZ()) {
				Frame transitionFrame;
				transitionFrame = robot.getCurrentCartesianPosition(recTCP).copy();
				transitionFrame.setZ(motionFrame.getZ());
				splineMotion = new Spline(spl(transitionFrame), spl(motionFrame));

			} /* else if (robot.getCurrentCartesianPosition(recTCP).getZ() < this.lowestPosition.getZ()
					&& motionFrame.getZ() < this.lowestPosition.getZ() && distanceToMotionFrame > distanceToReferenceFrame) {
				Frame transitionFrameOne;
				Frame transitionFrameTwo;
				transitionFrameOne = robot.getCurrentCartesianPosition(recTCP).copy();
				transitionFrameTwo = motionFrame.copy();

				transitionFrameOne.setZ(this.lowestPosition.getZ() + 50);
				transitionFrameTwo.setZ(this.lowestPosition.getZ() + 50);

				splineMotion = new Spline(spl(transitionFrameOne), spl(transitionFrameTwo), spl(motionFrame));

			} */ 
			else {
				splineMotion = new Spline(spl(motionFrame));
			}
			
//			splineMotion = new Spline(spl(motionFrame));

			do {
				Robert_2_3.activeAttachButton = false;
				
				robot.setESMState("3");

				//TODO why do we have here motionContainer.await(); it have been removed now
				motionContainer = recTCP.moveAsync(splineMotion.setBlendingCart(0.1).setCartVelocity(velo).setJointAccelerationRel(acce)
						.setJointJerkRel(jerk).setMode(rotationalSoft)
				 .breakWhen(patientButton)); 
				
				motionContainer.await();
				
				IFiredConditionInfo firedInfo = motionContainer.getFiredBreakConditionInfo();

				if (firedInfo != null) {
					position = robot.getCurrentJointPosition(); 
					
					Robert_2_3.activeAttachButton = true;
					robot.setESMState("5");
					bodyIOGroup.setPauseControl(true);
					softMc = robot.moveAsync(positionHold(softBounce, -1, TimeUnit.MILLISECONDS));
					
					ICondition firedCond = firedInfo.getFiredCondition();

					//TODO Counter reaction between motion might be removed ask for help
					if (firedCond.equals(patientButton)) {
						// if (firedCond.equals(patientButton)) {
						try {
							tcpConnection.sendMessage(UserCmd.PAUSED.toString());
						} catch (IOException e) {
							logger.info("ROBERT: Failed to send PAUSED message to the control interfase" + e);
						}
						// }

						linkageIOGroup.setLEDLightGreen(false);
						linkageIOGroup.setLEDLightRed(false); 
						ledColor.startFlashing(Color.GREEN);

						while (bodyIOGroup.getPlayPauseButton()) {
							ThreadUtil.milliSleep(100);
						}
						ThreadUtil.milliSleep(500); 
						motionContainer.cancel();

						// attach pressed or E-stop?
						if (waitForInteraction(robot, softMc, softBounce, rotationalSoft, position, recTCP, linkageIOGroup, bodyIOGroup, ledColor,
								logger, false)) {
							//userCmd = UserCmd.NONE;
							return false;
						}
						userStatus = UserCmd.RESUME;
						// continue exercise by finishing bounce mode
						while (!softMc.isFinished()) {
							softMc.cancel();
							ThreadUtil.milliSleep(100);
						}
						// remove finger from play button
						while (bodyIOGroup.getPlayPauseButton()) {
							ThreadUtil.milliSleep(100);
						}

						try {
							tcpConnection.sendMessage(UserCmd.RESUME.toString()); 

						} catch (IOException e) {
							logger.info("ROBERT: Failed to send PAUSED message to the control interfase" + e);
						}

						ledColor.stopFlashing();
					}

				} else {
					
					if (Robert_2_3.userCommand == UserCmd.SAFETY_STOP) { 
						Robert_2_3.userCommand = UserCmd.NONE;
						return false;
					}
					userStatus = UserCmd.NONE;
					return true;
				}

			} while (userStatus == UserCmd.RESUME);

			robot.deactivateDeviceCartesianVelocityLimit();

		}
	
		return true;

	}

	private boolean isTransition() {
		return transitionBool;
	}
	public boolean transitionMotionTo(int _motionNo) {
		this.motionTransitionNo = _motionNo;
		return transitionBool;		
	}
	public void setTransitionBool(boolean transitionBool) {
		this.transitionBool = transitionBool;
	}

}
