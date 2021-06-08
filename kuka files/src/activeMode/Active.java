package activeMode;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

import shared.LedColor;
import shared.LedColor.Color;
import shared.TCPConnection;
import shared.Training;
import util.UserCmd;
import backgroundTask.StateHelper;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.BodyIOGroup;
import com.kuka.generated.ioAccess.LinkageIOGroup;
import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.deviceModel.Device;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.IFiredConditionInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.ioModel.Input;
import com.kuka.roboticsAPI.ioModel.Output;
import com.kuka.roboticsAPI.motionModel.CartesianPTP;
import com.kuka.roboticsAPI.motionModel.ErrorHandlingAction;
import com.kuka.roboticsAPI.motionModel.IErrorHandler;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.task.ITaskLogger;

enum MoveDirct {
	XDIRCT, YDIRCT, ZDIRCT, MINUS_DIRECTION, PLUS_DIRECTION
}

public class Active extends Training {

	private int cycle;
	private int resistance;
	private double distance;
	private CartesianImpedanceControlMode controlMode;
	private MoveDirct direction;
	private MoveDirct direction_sign;
	private Frame endPosition;
	private final Frame kickOffPosition;
	private Frame kickOffPosShifted;
	private double[] velocity;
	public boolean isInterrupted;

	private boolean hasFinishedCycle;
	private boolean itHasRachedStartPos;
	private ITaskLogger logger;
	private ArrayList<Frame> switchToPathArray;
	private Frame toFrame;
	private int lastResistance = 0;
	private ArrayList<Frame> framesArr;
	private double maxKgF = 0;
	private double avgKgF = 0;
	private double kilo;
	private boolean rightDirectioin;
	private boolean safetyTriggered;
	private JointImpedanceControlMode jointImp;
	private double coefficient_t;
	private final double vienPumpDistance = 120;
	private int avgForceCnt = 0;
	private double donePercent = 80;
	private Spline splinePath;
	protected double[] returnPosition;
	private boolean returned;
	private BooleanIOCondition patientButton;
	private BooleanIOCondition endButton;
	private boolean isReady;
	private BooleanIOCondition pauseButton;
	private boolean cycleDone;
	private int i;
	public static boolean hasReachedEnd = false;
	public static double isVelocity = 0.0;
	private static final int LOCAL_PORT = 30007;
	private enum MOTION_NO {
		JOINTS_MOTION, TO_START_MOTION, PATH_MOTION
	}

	public void asyncErrorHandler(IApplicationControl applicationControl) {
		// Error handling of asynchronous motion
		IErrorHandler errorHandler = new IErrorHandler() {
			@Override
			public ErrorHandlingAction handleError(Device device, IMotionContainer failedContainer, List<IMotionContainer> canceledContainers) {
				logger.warn("Robert: the following motion command has failed: " + failedContainer.getErrorMessage());
				for (int i = 0; i < canceledContainers.size(); i++) {
					logger.info("I cannot execute the following commands: " + canceledContainers.get(i).toString());
				}
				// check if motion canceled due to pressing E-stop or an error.
				StateHelper.cancelMotion = true;
				return ErrorHandlingAction.Ignore;
			}
		};
		applicationControl.registerMoveAsyncErrorHandler(errorHandler);

	}

	public Active(ArrayList<Frame> frameList, ITaskLogger _logger, LBR _robot, IApplicationControl _appControl, IApplicationData _appData,
			LinkageIOGroup _linkageIOGroup, BodyIOGroup _bodyIOGroup, LedColor _ledColor, Tool _linkage, Frame _readyPositionFrame, TCPConnection _tcpConnection) {

		asyncErrorHandler(_appControl);
		this.linkageIOGroup = _linkageIOGroup;
		this.bodyIOGroup = _bodyIOGroup;
		this.logger = _logger;
		this.robot = _robot;
		this.ledColor = _ledColor;
		this.appData = _appData;
		this.linkage = _linkage;
		this.tcpConnection = _tcpConnection;
		/**
		 * copy the arrayList
		 */
		this.framesArr = new ArrayList<Frame>();
		for (int i = 0; i < frameList.size(); i++) {
			this.framesArr.add(frameList.get(i));
		}

		Collections.reverse(this.framesArr);

		this.kickOffPosition = frameList.get(0);

		this.endPosition = frameList.get(frameList.size() - 1);

		// this.distance = endPosition.distanceTo(this.kickOffPosition);

		this.velocity = new double[] { 0.2, 0.2, 0.2, 0.3, 0.2, 0.2, 0.2 };
		this.controlMode = new CartesianImpedanceControlMode();
		this.controlMode.parametrize(CartDOF.ALL).setDamping(1.0);

		findDirectionAndDistance(this.endPosition, this.kickOffPosition);

		Input playBtn = this.bodyIOGroup.getInput("PlayPauseButton");
		patientButton = new BooleanIOCondition(playBtn, true);

		Output pause = this.bodyIOGroup.getOutput("PauseControl");
		pauseButton = new BooleanIOCondition(pause, true);

		Output end = this.bodyIOGroup.getOutput("End");
		endButton = new BooleanIOCondition(end, true);

		bodyIOGroup.setEnd(false);
		bodyIOGroup.setPauseControl(false);
	}

	/**
	 * Playing the exercise. The play methods runs on its own thread
	 */
	@Override
	public int play(int _cycles, int _resistance) {
		this.cycle = _cycles;
		this.resistance = _resistance;

		rotationalSoft = getRotationalSoft();
		softBounce = getSoftBounce();
		hardImpedance = getHardImpedance();

		jointImp = new JointImpedanceControlMode(3000.0, 3000.0, 1000.0, 3000.0, 100.0, 100.0, 100.0);
		jointImp.setDampingForAllJoints(1.0);
		/**
		 * set the impedance according to direction of the recorded path
		 */
		setImpedanceForXYZ();

		isInterrupted = false;
		returned = true;
		boolean isOverTheDistance = false;
		onExtendedArm();
		// onRunningActiveMode();

		robot.setESMState("5");

		StateHelper.ESM_5_STOP = false;

		/*
		 * start the active mode even when the start position under the 10%
		 */
		isReady = true;
		startUdpCommunication(LOCAL_PORT);

		for ( i = 1; i <= this.cycle; i++) {

			double maxCycleKilo = 0;
			hasFinishedCycle = false;
			itHasRachedStartPos = true;

			/**
			 * activate the impedance again after returning to start position
			 */

			do {
				/**
				 * interrupt if end, attach or E-stop pressed.
				 */
				if (isInterrupted || StateHelper.isEstopPressed) {

					/**
					 * if attach was pressed then set isInterrupted to true
					 * anyway
					 */
					
					return 1;
				}

				Frame currentFrame = getlbrMed().getCurrentCartesianPosition(recTCP, World.Current.getRootFrame());
				ThreadUtil.milliSleep(100);
				isVelocity = currentFrame.distanceTo(robot.getCurrentCartesianPosition(recTCP));
				hasReachedEnd = false;
				
				/**
				 * update status variable itHasRachedStartPos, is set to true
				 */

				/**
				 * calculate the distance in the direction of the movement
				 */
				double newDistance = 0;

				switch (this.direction) {
				case XDIRCT:
					newDistance = currentFrame.copy().setY(0).setZ(0).distanceTo(this.kickOffPosition.copy().setY(0).setZ(0));

					if ((kickOffPosition.getX() - currentFrame.getX() > 0) && (direction_sign == MoveDirct.MINUS_DIRECTION)) {
						this.rightDirectioin = true;
					} else if ((kickOffPosition.getX() - currentFrame.getX() < 0) && (direction_sign == MoveDirct.PLUS_DIRECTION)) {
						this.rightDirectioin = true;
					} else {
						this.rightDirectioin = false;
					}
					break;
				case YDIRCT:
					newDistance = currentFrame.copy().setX(0).setZ(0).distanceTo(this.kickOffPosition.copy().setX(0).setZ(0));

					if ((kickOffPosition.getY() - currentFrame.getY() > 0) && (direction_sign == MoveDirct.MINUS_DIRECTION)) {
						this.rightDirectioin = true;
					} else if ((kickOffPosition.getY() - currentFrame.getY() < 0) && (direction_sign == MoveDirct.PLUS_DIRECTION)) {
						this.rightDirectioin = true;
					} else {
						this.rightDirectioin = false;
					}
					break;
				case ZDIRCT:
					newDistance = currentFrame.copy().setX(0).setY(0).distanceTo(this.kickOffPosition.copy().setX(0).setY(0));

					if ((kickOffPosition.getZ() - currentFrame.getZ() > 0) && (direction_sign == MoveDirct.MINUS_DIRECTION)) {
						this.rightDirectioin = true;
					} else if ((kickOffPosition.getZ() - currentFrame.getZ() < 0) && (direction_sign == MoveDirct.PLUS_DIRECTION)) {
						this.rightDirectioin = true;
					} else {
						this.rightDirectioin = false;
					}
					break;
				}
				double disPercent = (newDistance * 100) / this.distance; 
				
				if (rightDirectioin) {			
				_udpHelper.sendInfoMessage(UserCmd.PROGRESS.toString() + ";" + disPercent);
				} else{
				_udpHelper.sendInfoMessage(UserCmd.PROGRESS.toString() + ";" + 0.00);	
				}


				/*
				 * Active the mode if one of the following condition is met.
				 */
				if ((this.resistance != lastResistance && this.rightDirectioin) && (disPercent) < 40 || (StateHelper.ESM_5_STOP && !safetyTriggered)
						|| this.isReady) {
					this.isReady = false;
					lastResistance = this.resistance;
					logger.info("activate position control");

					if (ledColor.isFlashing()) {
						ledColor.stopFlashingNow();
						linkageIOGroup.setLEDLightGreen(true);
						linkageIOGroup.setLEDLightRed(false);
					}

					StateHelper.ESM_5_STOP = false;
					robot.setESMState("5");
					logger.info("Active Mode");
					
					motionContainer = recTCP.moveAsync(ptp(this.kickOffPosShifted).setMode(this.controlMode));

				}
				
				/*
				 * The cycle return to start position
				 */
				if ((disPercent) < 10 || !this.rightDirectioin) {

					itHasRachedStartPos = true;

					/**
					 * controlling the cycle
					 */
					if (isOverTheDistance) {
						isOverTheDistance = false;
						hasFinishedCycle = true;
						logger.info("ROBERT: below 10% of the cycle distance");
						sendMsg(UserCmd.CYCLE_START.toString());
					}
				}

				/**
				 * get over the 90% of the path
				 */
				else if (((newDistance * 100) / this.distance) > donePercent && this.rightDirectioin) {

					kilo = forceToKilo();

					/**
					 * finding the absolute max and keep it
					 */
					if (kilo > maxKgF && !safetyTriggered) {
						maxKgF = kilo;
					}

					/**
					 * counting all max value and use it
					 */
					if (kilo > maxCycleKilo) {
						maxCycleKilo = kilo;
					}

					if (itHasRachedStartPos) {
						logger.info("ROBERT: over " + donePercent + "% of the cycle distance");
						sendMsg(UserCmd.CYCLE_DONE.toString());
						itHasRachedStartPos = false;

						isOverTheDistance = true;

						// this.avgForceCnt++;
					}

					/**
					 * finish if it was the last cycle in this motion
					 */
					if (i == this.cycle) {
						linkageIOGroup.setLEDLightGreen(false);
						linkageIOGroup.setLEDLightRed(false);
						/**
						 * make the impedance more stiff
						 */
						motionContainer = recTCP.moveAsync(positionHold(new PositionControlMode(), 100, TimeUnit.MILLISECONDS));
						motionContainer.await();

						while (safetyTriggered) {
							ThreadUtil.milliSleep(100);
						}
						isInterrupted = true;
  
						if (kilo > 0) {
							avgKgF = avgKgF + maxCycleKilo;
							avgForceCnt++;
							// logger.info("AVG : " + avgKgF);
						}
						
						stopUdpComunication();
						
						return 0;
					}

					int cnt = 0;
					/**
					 * as long as the velocity is less than 8mm/seconds: or more
					 * precisely 0.8mm/0.1s
					 */
					do {
						/**
						 * start counting to 2 seconds
						 */
						cnt++;
						/**
						 * update the frames to check if the velocity is still
						 * below 8mm/s
						 */
						currentFrame = getlbrMed().getCurrentCartesianPosition(recTCP, World.Current.getRootFrame());
						ThreadUtil.milliSleep(100);
						toFrame = getlbrMed().getCurrentCartesianPosition(recTCP, World.Current.getRootFrame());
						

						isVelocity = currentFrame.distanceTo(toFrame);
						hasReachedEnd = true;
						
						if (cnt == 3) {							
							hasReachedEnd = true;
							/**
							 * finish if it was the last cycle in this motion
							 */
							if (i == this.cycle) {
								linkageIOGroup.setLEDLightGreen(false);
								linkageIOGroup.setLEDLightRed(false);
								motionContainer = recTCP.moveAsync(positionHold(new PositionControlMode(), 100, TimeUnit.MILLISECONDS)); 
								motionContainer.await();

								while (safetyTriggered) {
									ThreadUtil.milliSleep(100);
								}

								isInterrupted = true;
								
								stopUdpComunication();
								
								return 0;
							}

							// E-Stop was pressed, cancel everything
							if (StateHelper.isEstopPressed) {
								logger.info("ROBERT: all data was reset, due to pressing E-stop");
								this.framesArr.clear();
								return 1;
							}

							cycleDone = true;
							logger.info("rigth direction " + this.rightDirectioin + " and returned " + returned);

							try {
								if (!this.rightDirectioin) {
									break;
								}
								while (returned == false) {
									if (isInterrupted) {
										return 1;
									}
									ThreadUtil.milliSleep(100);
								}

								logger.info("ROBERT: starting to return");

								if (returnMotionControl(MOTION_NO.PATH_MOTION, framesArr.get(framesArr.size() - 1)) == 1) {
									isInterrupted = true;
									return 1;
								}

								robot.setESMState("5");
								logger.info("ROBERT: moving to start done");
								hasFinishedCycle = true;
								isOverTheDistance = false;
								lastResistance = 0;
								cycleDone = false;
								sendMsg(UserCmd.CYCLE_START.toString());

							} catch (Exception e) {
								logger.info("ROBERT: cannot move to start position" + e);
							}

							break;
						}

						// robot.setESMState("5");

					} while (currentFrame.distanceTo(toFrame) <= 0.8);

				}


			} while (!hasFinishedCycle);

			if (kilo > 0) {
				avgKgF = avgKgF + maxCycleKilo;
				avgForceCnt++;
				// logger.info("AVG : " + avgKgF);
			}
		}

		stopUdpComunication();
		
		while (!motionContainer.isFinished()) {
			motionContainer.cancel();
			ThreadUtil.milliSleep(100);
		}

		if (ledColor.isFlashing()) {
			ledColor.stopFlashingNow();
			linkageIOGroup.setLEDLightGreen(false);
			linkageIOGroup.setLEDLightRed(false);
		}

		return 0;

	}

	

	private void onExtendedArm() {

		try {
			new Thread(new Runnable() {

				@Override
				public void run() {
					Frame distinationFrame;

					returnPosition = robot.getCurrentJointPosition().get();
					distinationFrame = robot.getCurrentCartesianPosition(recTCP); 

					while (!isInterrupted) {
						// double[] currentPos = getJointsDegrees();
						double init_A4 = Math.toDegrees(robot.getCurrentJointPosition().get(JointEnum.J4));
						double init_A6 = Math.toDegrees(robot.getCurrentJointPosition().get(JointEnum.J6));
						double init_A7 = Math.toDegrees(robot.getCurrentJointPosition().get(JointEnum.J7));

						if (Math.abs(init_A4) < 5 || Math.abs(init_A4) > 110 || Math.abs(init_A6) > 110 || Math.abs(init_A6) < 5
								|| Math.abs(init_A7) > 165) {
							returned = false;
							safetyTriggered = true;
							
							robot.setESMState("6");
							ThreadUtil.milliSleep(300); 
							logger.info("ROBERT: safety stop because a joint position is below 10 degree");
							robot.setESMState("5");
							
							while (!motionContainer.isFinished()) {
								motionContainer.cancel();
								ThreadUtil.milliSleep(100);
							}
							try {
								motionContainer = recTCP.move(positionHold(hardImpedance, 1, TimeUnit.SECONDS));
							} catch (Exception e) {
								logger.info("ROBERT: failed to hold position: " + e);
							}
							
//							motionContainer.await();
//							
							while (!motionContainer.isFinished()) {
								motionContainer.cancel();
								ThreadUtil.milliSleep(100);
							}
							
							if (ledColor.isFlashing()) {
								linkageIOGroup.setLEDLightGreen(false);
								linkageIOGroup.setLEDLightRed(false);
								ledColor.stopFlashing();
							}
							ThreadUtil.milliSleep(100);
							ledColor.startFlashing(Color.GREEN);

							logger.info("ROBERT: below has been inverted");

							if (returnMotionControl(MOTION_NO.JOINTS_MOTION, distinationFrame) == 1) {
								return;
							}

							if (!rightDirectioin || cycleDone == false) {

								if (returnMotionControl(MOTION_NO.TO_START_MOTION, kickOffPosShifted) == 1) {
									return;
								}
							}

							returned = true;

							safetyTriggered = false;

						} else if (Math.abs(init_A4) > 50 && Math.abs(init_A4) < 100) {
							returnPosition = robot.getCurrentJointPosition().get();
							distinationFrame = robot.getCurrentCartesianPosition(recTCP);

						}
						ThreadUtil.milliSleep(10);
					}
				}
			}).start();
		} catch (Exception e) {
			logger.info("ROBERT: failed thread for controlling axis during active mode" + e);
		}
	}

	private int returnMotionControl(MOTION_NO motionType, Frame distinationFrame) {

		Frame current;
		double dis = 0;

		if (ledColor != null && !ledColor.isFlashing() && StateHelper.redLight == false) {
			linkageIOGroup.setLEDLightGreen(true);
			linkageIOGroup.setLEDLightRed(false);
			ledColor.startFlashing(Color.GREEN);
		}

		do {

			bodyIOGroup.setPauseControl(false);

			switch (motionType) {

			case JOINTS_MOTION:
				PTP ptpToReadyPosition = ptp(returnPosition).setJointVelocityRel(0.220).setJointAccelerationRel(0.03);
				motionContainer = recTCP.moveAsync(ptpToReadyPosition.setMode(jointImp).breakWhen(patientButton).breakWhen(pauseButton)
						.breakWhen(endButton));
				motionContainer.await();
				break;

			case PATH_MOTION:

				while (null != motionContainer && !motionContainer.isFinished()) {
					motionContainer.cancel();
					ThreadUtil.milliSleep(100);
				}
				try {
					this.splinePath = getReturningPath(framesArr, robot.getCurrentCartesianPosition(recTCP));
				} catch (Exception e) {
					logger.info("ROBERT: failed to return to start position" + e);
				}
				/* if (this.distance <= this.vienPumpDistance) { */
				motionContainer = recTCP.moveAsync(this.splinePath.setJointVelocityRel(this.velocity).setJointAccelerationRel(0.01)
						.breakWhen(patientButton).breakWhen(pauseButton).breakWhen(endButton));
				motionContainer.await();
				this.isReady = true;
				break;

			case TO_START_MOTION:
				CartesianPTP ptpToStartPosition = ptp(kickOffPosShifted).setJointVelocityRel(0.100).setJointAccelerationRel(0.03);
				motionContainer = recTCP.moveAsync(ptpToStartPosition.breakWhen(patientButton).breakWhen(pauseButton).breakWhen(endButton)
						.setMode(jointImp));
				motionContainer.await();
				this.isReady = true;
				break;
			}

			IFiredConditionInfo firedInfo = motionContainer.getFiredBreakConditionInfo();

			if (firedInfo != null && robot.isReadyToMove()) {

				bodyIOGroup.setPauseControl(true);
				ICondition firedCond = firedInfo.getFiredCondition();

				if (firedCond.equals(endButton)) {

					// ledColor.stopFlashing();
					//
					bodyIOGroup.setEnd(false);
					bodyIOGroup.setPauseControl(false);

					while (!motionContainer.isFinished()) {
						motionContainer.cancel();
						ThreadUtil.milliSleep(100);
					}

					return 1;
				}

				while (bodyIOGroup.getPlayPauseButton()) {
					ThreadUtil.milliSleep(100);
				}
				ThreadUtil.milliSleep(500);

				this.position = robot.getCurrentJointPosition();
				// logger.info("Robert: bounce mode is active");
				softMc = robot.moveAsync(positionHold(softBounce, -1, TimeUnit.MILLISECONDS));

				if (firedCond.equals(patientButton) || firedCond.equals(pauseButton)) {

					sendMsg(UserCmd.PAUSED.toString());

					// remove finger from play button

					while (bodyIOGroup.getPlayPauseButton()) {
						ThreadUtil.milliSleep(100);
					}

					if (waitForInteraction(robot, softMc, softBounce, rotationalSoft, position, recTCP, linkageIOGroup, bodyIOGroup, ledColor,
							logger, false)) {
						bodyIOGroup.setPauseControl(false);
						bodyIOGroup.setEnd(false);
						return 1;
					}
					bodyIOGroup.setPauseControl(false);
					bodyIOGroup.setEnd(false);

					sendMsg(UserCmd.RESUME.toString()); 

					// continue exercise by finishing bounce mode
					while (!softMc.isFinished()) {
						softMc.cancel();
						ThreadUtil.milliSleep(100);
					}
					// remove finger from play button
					while (bodyIOGroup.getPlayPauseButton()) {
						ThreadUtil.milliSleep(100);
					}

				} else if (firedCond.equals(endButton)) {

					return 1;
				}
				current = robot.getCurrentCartesianPosition(recTCP, World.Current.getRootFrame());
				dis = distinationFrame.distanceTo(current);
			} else { 
				dis = 0;
			}
		} while (dis > 3);
		return 0;
	}


	private synchronized void sendMsg(String _userCmd) {

		try {
			this.tcpConnection.sendMessage(_userCmd); 
		} catch (IOException e) {
			logger.info("ROBERT: Failed to send the message: " + _userCmd + "in the active mode" + e); 
		}
	}

	/**
	 * updating the cycle number and the resistance level
	 */
	@Override
	public void update(int _cycle, int _resistance) {
		this.cycle = _cycle;
		if (this.resistance != _resistance) {
			this.resistance = _resistance;
			setImpedanceForXYZ();
			lastResistance = 0;
		}
	}

	@Override
	public void pause() {

		bodyIOGroup.setPauseControl(true);
	}

	/**
	 * resume the exercise by setting the light indicator to green
	 */
	@Override
	public void resume() {
		bodyIOGroup.setPauseControl(false);

	}

	/**
	 * The end methods ends the current exercise and it runs on the class level.
	 */
	@Override
	public void end() {
		bodyIOGroup.setEnd(true);
		isInterrupted = true; 
	}
	
	@Override
	public void skipMotion() {
		bodyIOGroup.setEnd(true);
		isInterrupted = true;
	}

	/**
	 * Finding the direction of the movement based on the start and end
	 * positions, setting the class property (direction) to the founded
	 * direction
	 * 
	 * @param endPos
	 * @param kickOffPos
	 */
	private void findDirectionAndDistance(Frame endPos, Frame kickOffPos) {

		Frame xFirstPosition = endPos.copy().setY(0).setZ(0);
		Frame xkickOffPosition = kickOffPos.copy().setY(0).setZ(0);

		Frame yFirstPosition = endPos.copy().setX(0).setZ(0);
		Frame ykickOffPosition = kickOffPos.copy().setX(0).setZ(0);

		Frame zFirstPosition = endPos.copy().setX(0).setY(0);
		Frame zkickOffPosition = kickOffPos.copy().setX(0).setY(0);

		double x_Distance = xFirstPosition.distanceTo(xkickOffPosition);
		double y_Distance = yFirstPosition.distanceTo(ykickOffPosition);
		double z_Distance = zFirstPosition.distanceTo(zkickOffPosition);

		if (x_Distance > y_Distance && x_Distance > z_Distance) {
			this.direction = MoveDirct.XDIRCT;
			this.distance = x_Distance;

			if (xkickOffPosition.getX() - xFirstPosition.getX() > 0) {
				this.direction_sign = MoveDirct.MINUS_DIRECTION;

				this.kickOffPosShifted = findMaximumPosShift(kickOffPos, this.endPosition);

			} else {
				this.direction_sign = MoveDirct.PLUS_DIRECTION;
				this.kickOffPosShifted = findMaximumPosShift(kickOffPos, this.endPosition);
			}

		} else if (y_Distance > x_Distance && y_Distance > z_Distance) {
			this.direction = MoveDirct.YDIRCT;
			this.distance = y_Distance;

			if (ykickOffPosition.getY() - yFirstPosition.getY() > 0) {
				this.direction_sign = MoveDirct.MINUS_DIRECTION;

				this.kickOffPosShifted = findMaximumPosShift(kickOffPos, this.endPosition);

			} else {
				this.direction_sign = MoveDirct.PLUS_DIRECTION;
				this.kickOffPosShifted = findMaximumPosShift(kickOffPos, this.endPosition);

			}

		} else if (z_Distance > x_Distance && z_Distance > y_Distance) {
			this.direction = MoveDirct.ZDIRCT;
			this.distance = z_Distance;

			this.kickOffPosShifted = this.kickOffPosition;
			logger.info("ROBERT: Z-direction path");
			if (zkickOffPosition.getZ() - zFirstPosition.getZ() > 0) {
				this.direction_sign = MoveDirct.MINUS_DIRECTION;

			} else {
				this.direction_sign = MoveDirct.PLUS_DIRECTION; 
			} 
		} 
		
		if (this.distance > vienPumpDistance) {
			sendMsg(UserCmd.VEIN_PUMP.toString() + ";false");
		} else {
			sendMsg(UserCmd.VEIN_PUMP.toString() + ";true");
		}
	}

	private Frame findMaximumPosShift(Frame _kickOffPos, Frame _lineSecondPos) {

		double framdis = _kickOffPos.distanceTo(World.Current.getRootFrame());
		logger.info("ROBERT: Kickoff distance : " + framdis);
		if (this.distance <= this.vienPumpDistance) {
			coefficient_t = 2;

		} else if (this.distance > this.vienPumpDistance) {
			// coefficient_t = 1.2;
			return _kickOffPos;
		}

		Frame shiftedFrame = _kickOffPos.copy();
		double offset = 0.0;
		double shiftDistance;

		double posVector_x = shiftedFrame.getX() - _lineSecondPos.getX();
		double posVector_y = shiftedFrame.getY() - _lineSecondPos.getY();
		double posVector_z = shiftedFrame.getZ() - _lineSecondPos.getZ();

		do {
			coefficient_t = coefficient_t - offset;

			if (coefficient_t <= 1) {
				return _kickOffPos;
			}

			double var1 = posVector_x * coefficient_t;
			double var2 = posVector_y * coefficient_t;
			double var3 = posVector_z * coefficient_t;

			shiftedFrame.setX(_lineSecondPos.getX() + var1);
			shiftedFrame.setY(_lineSecondPos.getY() + var2);
			shiftedFrame.setZ(_lineSecondPos.getZ() + var3);
			offset = 0.02;

			shiftDistance = shiftedFrame.distanceTo(World.Current.getRootFrame());
			logger.info("ROBERT: dis: " + shiftDistance + "Coeff: " + coefficient_t);

		} while (shiftDistance > 950);

		// logger.info("ROBERT: Coefficient is: " + coefficient_t);

		return shiftedFrame;
	}

	/**
	 * set the impedance control parameters based on the direction of the
	 * recording
	 */
	void setImpedanceForXYZ() { 

		// generic training
		if (this.distance > vienPumpDistance) {
			
			switch (this.direction) {
			case XDIRCT:
				setX_Resistance();
				break;
			case YDIRCT:
				setY_Resistance();
				break;
			case ZDIRCT:
				setZ_Resistance();
				break;
			}
			// Vein pump training
		} else if (this.distance <= vienPumpDistance) {

			switch (this.direction) {
			case XDIRCT:
				this.controlMode.parametrize(CartDOF.ROT).setDamping(1.0);
				this.controlMode.parametrize(CartDOF.TRANSL).setDamping(0.1);
				setX_ResistancePeristalic();
				break;
			case YDIRCT:
				this.controlMode.parametrize(CartDOF.ROT).setDamping(1.0);
				this.controlMode.parametrize(CartDOF.TRANSL).setDamping(0.1);

				setY_ResistancePeristalic();	
				break;
			case ZDIRCT:

				setZ_Resistance();
				break;
			}
		}
		
		logger.info("ROBERT: the resistance constants are: " + controlMode.getStiffness()[0] + ", " + controlMode.getStiffness()[1] + ", "
				+ controlMode.getStiffness()[2] + ", " + controlMode.getStiffness()[3] + ", " + controlMode.getStiffness()[4] + ", "
				+ controlMode.getStiffness()[5] + " ");
	}

	private void setY_ResistancePeristalic() {

		switch (this.resistance) {
		case 1:
			controlMode.parametrize(CartDOF.Y).setStiffness(500);
			controlMode.parametrize(CartDOF.X).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 77.5;
			break;
		case 2:
			controlMode.parametrize(CartDOF.Y).setStiffness(570.0);
			controlMode.parametrize(CartDOF.X).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 75;
			break;
		case 3:
			controlMode.parametrize(CartDOF.Y).setStiffness(640.0);
			controlMode.parametrize(CartDOF.X).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 72.5;
			break;
		case 4:
			controlMode.parametrize(CartDOF.Y).setStiffness(710.0);
			controlMode.parametrize(CartDOF.X).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 70;
			break;
		case 5:
			controlMode.parametrize(CartDOF.Y).setStiffness(780.0);
			controlMode.parametrize(CartDOF.X).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 67.5;
			break;
		case 6:
			controlMode.parametrize(CartDOF.Y).setStiffness(850.0);
			controlMode.parametrize(CartDOF.X).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 65;
			break;
		case 7:
			controlMode.parametrize(CartDOF.Y).setStiffness(920.0);
			controlMode.parametrize(CartDOF.X).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 62.5;
			break;
		case 8:
			controlMode.parametrize(CartDOF.Y).setStiffness(990.0);
			controlMode.parametrize(CartDOF.X).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 60;
			break;
		case 9:
			controlMode.parametrize(CartDOF.Y).setStiffness(1060.0);
			controlMode.parametrize(CartDOF.X).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 57.5;
			break;
		case 10:
			controlMode.parametrize(CartDOF.Y).setStiffness(1100.0);
			controlMode.parametrize(CartDOF.X).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 55;
			break;
		}

	}

	private void setX_ResistancePeristalic() {

		switch (this.resistance) {
		case 1:
			controlMode.parametrize(CartDOF.X).setStiffness(500);
			controlMode.parametrize(CartDOF.Y).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 77.5;
			break;
		case 2:
			controlMode.parametrize(CartDOF.X).setStiffness(570.0);
			controlMode.parametrize(CartDOF.Y).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 75;
			break;
		case 3:
			controlMode.parametrize(CartDOF.X).setStiffness(640.0);
			controlMode.parametrize(CartDOF.Y).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 72.5;
			break;
		case 4:
			controlMode.parametrize(CartDOF.X).setStiffness(710.0);
			controlMode.parametrize(CartDOF.Y).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 70;
			break;
		case 5:
			controlMode.parametrize(CartDOF.X).setStiffness(780.0);
			controlMode.parametrize(CartDOF.Y).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 67.5;
			break;
		case 6:
			controlMode.parametrize(CartDOF.X).setStiffness(850.0);
			controlMode.parametrize(CartDOF.Y).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 65;
			break;
		case 7:
			controlMode.parametrize(CartDOF.X).setStiffness(920.0);
			controlMode.parametrize(CartDOF.Y).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 62.5;
			break;
		case 8:
			controlMode.parametrize(CartDOF.X).setStiffness(990.0);
			controlMode.parametrize(CartDOF.Y).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 60;
			break;
		case 9:
			controlMode.parametrize(CartDOF.X).setStiffness(1060.0);
			controlMode.parametrize(CartDOF.Y).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 57.5;
			break;
		case 10:
			controlMode.parametrize(CartDOF.X).setStiffness(1100.0);
			controlMode.parametrize(CartDOF.Y).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(4500.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(250);
			donePercent = 55;
			break;
		}

	}

	/**
	 * set the resistance level of the Y direction
	 */
	void setY_Resistance() {

		switch (this.resistance) {
		case 1:
			controlMode.parametrize(CartDOF.Y).setStiffness(30.0);
			controlMode.parametrize(CartDOF.X).setStiffness(1800.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(30.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(30);
			break;
		case 2:
			controlMode.parametrize(CartDOF.Y).setStiffness(60.0);
			controlMode.parametrize(CartDOF.X).setStiffness(1800.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(30.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(30);
			break;
		case 3:
			controlMode.parametrize(CartDOF.Y).setStiffness(90.0);
			controlMode.parametrize(CartDOF.X).setStiffness(1800.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(30.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(30);
			break;
		case 4:
			controlMode.parametrize(CartDOF.Y).setStiffness(120.0);
			controlMode.parametrize(CartDOF.X).setStiffness(1800.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(30.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(30);
			break;
		case 5:
			controlMode.parametrize(CartDOF.Y).setStiffness(150.0);
			controlMode.parametrize(CartDOF.X).setStiffness(1800.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(30.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(30);
			break;
		case 6:
			controlMode.parametrize(CartDOF.Y).setStiffness(180.0);
			controlMode.parametrize(CartDOF.X).setStiffness(1800.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(30.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(30);
			break;
		case 7:
			controlMode.parametrize(CartDOF.Y).setStiffness(210.0);
			controlMode.parametrize(CartDOF.X).setStiffness(1800.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(30.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(30);
			break;
		case 8:
			controlMode.parametrize(CartDOF.Y).setStiffness(240.0);
			controlMode.parametrize(CartDOF.X).setStiffness(1800.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(30.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(30);
			break;
		case 9:
			controlMode.parametrize(CartDOF.Y).setStiffness(270.0);
			controlMode.parametrize(CartDOF.X).setStiffness(1800.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(30.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(30);
			break;
		case 10:
			controlMode.parametrize(CartDOF.Y).setStiffness(300.0);
			controlMode.parametrize(CartDOF.X).setStiffness(1800.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(30.0);
			controlMode.parametrize(CartDOF.A).setStiffness(250);
			controlMode.parametrize(CartDOF.B).setStiffness(250);
			controlMode.parametrize(CartDOF.C).setStiffness(30);
			break;
		}
	}

	/**
	 * set the resistance level of the x direction
	 */
	void setX_Resistance() {

		switch (this.resistance) {
		case 1:
			controlMode.parametrize(CartDOF.Y).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.X).setStiffness(5.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.A).setStiffness(1);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(100);
			break;
		case 2:
			controlMode.parametrize(CartDOF.Y).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.X).setStiffness(15.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.A).setStiffness(1);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(100);
			break;
		case 3:
			controlMode.parametrize(CartDOF.Y).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.X).setStiffness(25.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.A).setStiffness(1);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(100);
			break;
		case 4:
			controlMode.parametrize(CartDOF.Y).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.X).setStiffness(35.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.A).setStiffness(1);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(100);
			break;
		case 5:
			controlMode.parametrize(CartDOF.Y).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.X).setStiffness(45.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.A).setStiffness(1);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(100);
			break;
		case 6:
			controlMode.parametrize(CartDOF.Y).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.X).setStiffness(55.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.A).setStiffness(1);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(100);
			break;
		case 7:
			controlMode.parametrize(CartDOF.Y).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.X).setStiffness(65.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(5000.0);
			// controlMode.parametrize(CartDOF.ROT).setStiffness(50);
			controlMode.parametrize(CartDOF.A).setStiffness(1);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(100);
			break;
		case 8:
			controlMode.parametrize(CartDOF.Y).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.X).setStiffness(75.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(5000.0);
			// controlMode.parametrize(CartDOF.ROT).setStiffness(50);
			controlMode.parametrize(CartDOF.A).setStiffness(1);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(100);
			break;
		case 9:
			controlMode.parametrize(CartDOF.Y).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.X).setStiffness(85.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.A).setStiffness(1);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(100);
			break;
		case 10:
			controlMode.parametrize(CartDOF.Y).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.X).setStiffness(95.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.A).setStiffness(1);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(100);
			break;
		}
	}

	/**
	 * set the resistance level of the z direction
	 */
	void setZ_Resistance() {
		switch (this.resistance) {
		case 1:
			controlMode.parametrize(CartDOF.Y).setStiffness(300.0);
			controlMode.parametrize(CartDOF.X).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(100.0);
			controlMode.parametrize(CartDOF.A).setStiffness(100);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(10);
			break;
		case 2:
			controlMode.parametrize(CartDOF.Y).setStiffness(300.0);
			controlMode.parametrize(CartDOF.X).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(120.0);
			controlMode.parametrize(CartDOF.A).setStiffness(100);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(10);
			break;
		case 3:
			controlMode.parametrize(CartDOF.Y).setStiffness(300.0);
			controlMode.parametrize(CartDOF.X).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(140.0);
			controlMode.parametrize(CartDOF.A).setStiffness(100);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(10);
			break;
		case 4:
			controlMode.parametrize(CartDOF.Y).setStiffness(300.0);
			controlMode.parametrize(CartDOF.X).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(160.0);
			// controlMode.parametrize(CartDOF.ROT).setStiffness(40);
			controlMode.parametrize(CartDOF.A).setStiffness(100);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(10);
			break;
		case 5:
			controlMode.parametrize(CartDOF.Y).setStiffness(300.0);
			controlMode.parametrize(CartDOF.X).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(180.0);
			// controlMode.parametrize(CartDOF.ROT).setStiffness(50);
			controlMode.parametrize(CartDOF.A).setStiffness(100);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(10);
			break;
		case 6:
			controlMode.parametrize(CartDOF.Y).setStiffness(300.0);
			controlMode.parametrize(CartDOF.X).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(200.0);
			// controlMode.parametrize(CartDOF.ROT).setStiffness(50);
			controlMode.parametrize(CartDOF.A).setStiffness(100);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(10);
			break;
		case 7:
			controlMode.parametrize(CartDOF.Y).setStiffness(300.0);
			controlMode.parametrize(CartDOF.X).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(220.0);
			// controlMode.parametrize(CartDOF.ROT).setStiffness(50);
			controlMode.parametrize(CartDOF.A).setStiffness(100);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(10);
			break;
		case 8:
			controlMode.parametrize(CartDOF.Y).setStiffness(300.0);
			controlMode.parametrize(CartDOF.X).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(240.0);
			controlMode.parametrize(CartDOF.A).setStiffness(100);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(10);
			break;
		case 9:
			controlMode.parametrize(CartDOF.Y).setStiffness(300.0);
			controlMode.parametrize(CartDOF.X).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(260.0);
			controlMode.parametrize(CartDOF.A).setStiffness(100);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(10);
			break;
		case 10:
			controlMode.parametrize(CartDOF.Y).setStiffness(300.0);
			controlMode.parametrize(CartDOF.X).setStiffness(5000.0);
			controlMode.parametrize(CartDOF.Z).setStiffness(280.0);
			controlMode.parametrize(CartDOF.A).setStiffness(100);
			controlMode.parametrize(CartDOF.B).setStiffness(100);
			controlMode.parametrize(CartDOF.C).setStiffness(10);
			break;

		}
	}

	public void dispose() {

		hasFinishedCycle = true;
	}

	/**
	 * calculate and return the spline motion to switch between the first and
	 * next motion. This method will be called on the start of the next motion
	 * 
	 * @return {@link Spline}
	 */
	public Spline getSwitchToPath(boolean isNextMotion) {

		ArrayList<IMotion> path = goToNextMotion(this.switchToPathArray);
		return getSplineMotion(path);

	}

	private double forceToKilo() {

		ForceSensorData measurements = robot.getExternalForceTorque(recTCP, World.Current.getRootFrame());
		double forceSqrt = Math.sqrt(Math.pow(measurements.getForce().getX(), 2) + Math.pow(measurements.getForce().getY(), 2)
				+ Math.pow(measurements.getForce().getZ(), 2));

		if (measurements.isForceValid(9)) {
			return forceSqrt * 0.1019716213;
		} else {
			return -1;
		}

	}

	public String getKgForce() {

		if (avgForceCnt == 0) {
			return UserCmd.KG_FORCE.toString() + ";" + "0" + ";" + "0";
		} else {
			logger.info("ROBERT: counting: " + this.avgForceCnt + ", and avg force: " + (avgKgF / this.avgForceCnt));
			return UserCmd.KG_FORCE.toString() + ";" + maxKgF + ";" + (avgKgF / this.avgForceCnt);
		}
	}

}
