package guidedMode;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import main.Robert_2_3;
import shared.ActivityRecorder;
import shared.BounceConditionControl;
import shared.Degree;
import shared.LedColor;
import shared.LedColor.Color;
import shared.TCPConnection;
import shared.Training;
import sun.misc.GC.LatencyRequest;
import util.UserCmd;
import backgroundTask.StateHelper;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.BodyIOGroup;
import com.kuka.generated.ioAccess.LinkageIOGroup;
import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.EmergencyStop;
import com.kuka.roboticsAPI.deviceModel.Device;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.IFiredConditionInfo;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.ioModel.Input;
import com.kuka.roboticsAPI.ioModel.Output;
import com.kuka.roboticsAPI.motionModel.ErrorHandlingAction;
import com.kuka.roboticsAPI.motionModel.IErrorHandler;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.task.ITaskLogger;

public class Guided extends Training {

	private int cycles;

	private double velo;

	private double acce;

	private double jerk;

	private Spline spline;

	private BooleanIOCondition patientButton;

	private ArrayList<Frame> frameList;

	private IApplicationData appData;

	private LedColor ledColor;

	public static boolean isTorqueExceeded;

	public static boolean isPlayOrEndPressed;

	private ArrayList<Double> weights;

	private ArrayList<TorqueSensorData> registeredTorques;

	private ActivityRecorder activity;

	private double activeDistance = 0.0;

	private double fullDisatance = 0.0;

	private Degree degree;

	private TCPConnection tcpConnection;

	private ArrayList<IMotion> allMotions;

	private Frame stoppingPos;

	private ObjectFrame tcp;

	private BounceConditionControl bounceConditionControl;

	public boolean suspend;
	private static final int _LOCAL_PORT = 30007;

	public void asyncErrorHandler(IApplicationControl applicationControl) {
		// Error handling of asynchronous motion
		IErrorHandler errorHandler = new IErrorHandler() {
			@Override
			public ErrorHandlingAction handleError(Device device, IMotionContainer failedContainer, List<IMotionContainer> canceledContainers) {
				logger.warn("Robert: the following motion command has failed: " + failedContainer.getErrorMessage());

				if (ledColor != null) {
					ledColor.stopFlashing();
				}

				for (int i = 0; i < canceledContainers.size(); i++) {
					logger.info("I cannot execute the following commands: " + canceledContainers.get(i).toString());
				}

				// check if motion canceled due to pressing E-stop or an error.
				StateHelper.cancelMotion = true;
				// check if motion canceled due to pressing E-stop or an error.
				return ErrorHandlingAction.Ignore;
			}
		};
		applicationControl.registerMoveAsyncErrorHandler(errorHandler);

	}

	public Guided(LBR _robot, ArrayList<Frame> _frameList, ArrayList<TorqueSensorData> torqueArray, ArrayList<Double> _weights,
			IApplicationData _appData, LinkageIOGroup _linkageIOGroup, BodyIOGroup _bodyIOGroup, LedColor _ledColor, Tool _linkage,
			ITaskLogger _logger, IApplicationControl _appControl, TCPConnection _tcpConnection) {

		asyncErrorHandler(_appControl);

		this.frameList = _frameList;
		this.weights = _weights;
		this.registeredTorques = torqueArray;
		// remainingMotions = calculateGuidedPath(frameList, false);
		// this.spline = getSplineMotion(remainingMotions);
		this.logger = _logger;
		this.appData = _appData;
		this.linkageIOGroup = _linkageIOGroup;
		this.ledColor = _ledColor;
		this.robot = _robot;
		this.linkage = _linkage;
		this.bodyIOGroup = _bodyIOGroup;
		this.tcpConnection = _tcpConnection;
		rotationalSoft = getRotationalSoft();
		softBounce = getSoftBounce();

		// prepareBounceCondition(this.registeredTorques, this.appData);

		tcp = linkage.getFrame("/RecTCP");
	}

	@Override
	public int play(int _cycles, int _resistance) {
		this.cycles = _cycles;
		this.fullDisatance = getFullDistance();
		isMotionCancelled = false;
		this.suspend = false;
		activeDistance = 0.0;
		// bodyIOGroup.setPauseControl(false);
		// robot.getLoadData().setMass(weights.get(0));
		// ThreadUtil.milliSleep(100);

		startUdpCommunication(_LOCAL_PORT);

		Progress progress = new Progress();
		Thread thread = new Thread(progress);
		thread.start();

		for (int i = 1; i <= this.cycles; i++) {
			if (isMotionCancelled) {
				logger.info("Robert: the exercise is ended");
				this.cycles = i;
				return 1;
			}
			/**
			 * start controlling the activity by the patient
			 */
			activity = new ActivityRecorder(frameList, registeredTorques, weights, robot, linkage, logger, bodyIOGroup);
			bounceConditionControl = new BounceConditionControl(frameList, registeredTorques, weights, robot, linkage, logger, bodyIOGroup);

			Input playBtn = this.bodyIOGroup.getInput("PlayPauseButton");
			patientButton = new BooleanIOCondition(playBtn, true);

			Output pause = this.bodyIOGroup.getOutput("PauseControl");
			BooleanIOCondition pauseButton = new BooleanIOCondition(pause, true);

			bodyIOGroup.setEnd(false);
			Output end = this.bodyIOGroup.getOutput("End");
			BooleanIOCondition endButton = new BooleanIOCondition(end, true);

			allMotions = calculateGuidedPath(frameList);

			this.spline = getSplineMotion(remainingMotions);

			// StateHelper.isEstopPressed = false;

			do {

				robot.setESMState("3");

				Robert_2_3.activeAttachButton = false;

				if (robot.getSafetyState().getEmergencyStopEx() == EmergencyStop.INACTIVE) {
					linkageIOGroup.setLEDLightGreen(true);
					linkageIOGroup.setLEDLightRed(false);
				}

				velo = this.appData.getProcessData("velo").getValue();
				acce = this.appData.getProcessData("acce").getValue();
				jerk = this.appData.getProcessData("jerk").getValue();
				logger.info("ROBERT: the current velocity: " + velo + ", acceleratioin: " + acce + ", jerk: " + jerk);


				bodyIOGroup.setPauseControl(false);
				bodyIOGroup.setEnd(false);
				this.suspend = false;
				
				motionContainer = tcp.moveAsync(this.spline.breakWhen(patientButton).breakWhen(pauseButton).breakWhen(endButton)
						.setCartVelocity(velo).setJointAccelerationRel(acce).setJointJerkRel(jerk));

				// wait for container to be finished
				motionContainer.await();

				this.stoppingPos = robot.getCurrentCartesianPosition(linkage.getFrame("/RecTCP"));


				bodyIOGroup.setEnd(false);
				// cancel motion
				if (StateHelper.isEstopPressed) {
					this.cycles = i;
					// isMotionCancelled = true;
					// remainingMotions.clear();
					// activity.stop();
					// bounceConditionControl.stop();
					end();
					logger.info("ROBERT: all data was reset, due to pressing E-stop");
					return 1;
				}

				IFiredConditionInfo firedInfo = motionContainer.getFiredBreakConditionInfo();

				//ThreadUtil.milliSleep(100);
				if (firedInfo != null && robot.isReadyToMove()) {


					ICondition firedCond = firedInfo.getFiredCondition();

					/**
					 * cancel every thing if the END button was pressed
					 */
					if (firedCond.equals(endButton)) {
						linkageIOGroup.setLEDLightGreen(false);
						linkageIOGroup.setLEDLightRed(false);
						bodyIOGroup.setEnd(false);
						bodyIOGroup.setPauseControl(false);
						robot.setESMState("2");
						while (!motionContainer.isFinished()) {
							motionContainer.cancel();
							ThreadUtil.milliSleep(100);
						}
						// cancel every thing
						return 1;
					}

					this.suspend = true;
					Robert_2_3.activeAttachButton = true;
					robot.setESMState("5");
					bodyIOGroup.setPauseControl(true);
					
					/**
					 * recalculate the motion
					 */
					this.spline = resumeMotion(motionContainer, allMotions, this.frameList, this.stoppingPos, true);

					if (this.spline == null) {
						break;
					}

					this.position = robot.getCurrentJointPosition();
					// logger.info("Robert: bounce mode is active");
					softMc = robot.moveAsync(positionHold(softBounce, -1, TimeUnit.MILLISECONDS));

					if (firedCond.equals(patientButton) || firedCond.equals(pauseButton)) {
						
						activity.suspend();
						bounceConditionControl.suspend();
						
						// if (firedCond.equals(patientButton)) {
						try {
							tcpConnection.sendMessage(UserCmd.PAUSED.toString());
						} catch (IOException e) {
							this.logger.info("ROBERT: Failed to send PAUSED message to the control interfase" + e);
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
							this.cycles = i;
							// isMotionCancelled = true;
							// remainingMotions.clear();
							// activity.stop();
							// bounceConditionControl.stop();
							this.end();
							return 1;
						}

						// continue exercise by finishing bounce mode
						while (!softMc.isFinished()) {
							softMc.cancel();
							ThreadUtil.milliSleep(100);
						}
						// remove finger from play button
						while (bodyIOGroup.getPlayPauseButton()) {
							ThreadUtil.milliSleep(100);
						}
						
						recTCP.move(positionHold(new PositionControlMode(), 100, TimeUnit.MILLISECONDS));
						
						try {
							tcpConnection.sendMessage(UserCmd.RESUME.toString());

						} catch (IOException e) {
							this.logger.info("ROBERT: Failed to send PAUSED message to the control interfase" + e);
						}

						ledColor.stopFlashing();
						activity.resume();
						bounceConditionControl.resume();
						/*
						 * } else if (firedCond.equals(bounceCondition)) {
						 * logger.info(
						 * "Robert: bounce mode is active. One of the torque value has been exceeded:\n"
						 * + firedInfo.getFiredCondition().toString());
						 * 
						 * try {
						 * tcpConnection.sendMessage(UserCmd.PAUSED.toString());
						 * } catch (IOException e) { this.logger.info(
						 * "ROBERT: Failed to send PAUSED message to the control interfase"
						 * + e); }
						 * 
						 * linkageIOGroup.setLEDLightGreen(false);
						 * linkageIOGroup.setLEDLightRed(false);
						 * ledColor.startFlashing(Color.GREEN);
						 * motionContainer.cancel(); // waiting for interaction
						 * if (waitForInteraction(robot, softMc, soft, position,
						 * recTCP, linkageIOGroup, bodyIOGroup, ledColor,
						 * logger, false)) { activity.stop();
						 * bounceConditionControl.stop(); isMotionCancelled =
						 * true; remainingMotions.clear(); return 1; } //
						 * continue exercise by finishing bounce mode while
						 * (!softMc.isFinished()) { softMc.cancel();
						 * ThreadUtil.milliSleep(100); } // remove finger from
						 * play button while (bodyIOGroup.getPlayPauseButton())
						 * { ThreadUtil.milliSleep(100); }
						 * 
						 * try {
						 * tcpConnection.sendMessage(UserCmd.RESUME.toString());
						 * } catch (IOException e) { this.logger.info(
						 * "ROBERT: Failed to send RESUME message to the control interfase"
						 * + e); }
						 * 
						 * // continue exercise ledColor.stopFlashing();
						 * 
						 * activity.resume(); bounceConditionControl.resume();
						 */} else if (firedCond.equals(endButton)) {

						while (!motionContainer.isFinished()) {
							motionContainer.cancel();
						}
						// cancel every thing
						return 1;
					}
					// Set softMC to cancel anyway
					while (!softMc.isFinished()) {
						softMc.cancel();
						ThreadUtil.milliSleep(100);
					}

				} else {
					// in case of safety stop due to hard collision
					ThreadUtil.milliSleep(100);
					if (!robot.isReadyToMove()) {
						activity.resume();
						bounceConditionControl.resume();

						// recalculate motions
						this.spline = resumeMotion(motionContainer, allMotions, this.frameList, this.stoppingPos);
						// finishing route normally
					} else {
						/**
						 * clear the remaining motions
						 */
						remainingMotions.clear();
						if (softMc != null) {
							while (!softMc.isFinished()) {
								softMc.cancel();
								ThreadUtil.milliSleep(100);
							}
						}

						if (motionContainer != null) {
							while (!motionContainer.isFinished()) {
								motionContainer.cancel();
								ThreadUtil.milliSleep(100);
							}
						}

						// Robert_2_3.userCommand = UserCmd.NONE;
						// if (Robert_2_3.userCommand == UserCmd.SAFETY_STOP) {
						// Robert_2_3.userCommand = UserCmd.NONE;
						//
						// }

					}
				}


				ThreadUtil.milliSleep(100);

			} while (!remainingMotions.isEmpty());

			bodyIOGroup.setPauseControl(false);
			bodyIOGroup.setEnd(false);
			ThreadUtil.milliSleep(100);

			this.activeDistance = this.activeDistance + activity.getActiveDistance();

			activity.stop();
			bounceConditionControl.stop();
			logger.info("Nomber of repeation is: " + i);
			try {
				this.tcpConnection.sendMessage(UserCmd.CYCLE_DONE.toString());
			} catch (IOException e) {
				this.logger.info("ROBERT: guided failed to send the cycle done to the control interface" + e);
			}
		}
		isMotionCancelled = true;

		stopUdpComunication();
		robot.setESMState("2");
		return 0;
	}

	private class Progress implements Runnable {

		private Frame frame;
		private Frame lastFrame;
		private double totalDistance = 0;
		private double distToStartPos;
		private int lastValue;
		private ArrayList<Double> dist;
		private int index;

		private Progress() {
			distToStartPos = 0.0;
			double current = 0.0;
			dist = new ArrayList<Double>();

			lastValue = frameList.size() / 2;

			this.lastFrame = robot.getCurrentCartesianPosition(recTCP);

			for (int i = 0; i < lastValue; i++) {

				//if (i < lastValue - 1) {
					current = frameList.get(i).distanceTo(frameList.get(i + 1));
					distToStartPos += current;
					dist.add(distToStartPos);
				//}
			}
		}

		@Override
		public void run() {

			while (!isMotionCancelled) {

				if (motionContainer != null && allMotions != null) {

					frame = robot.getCurrentCartesianPosition(recTCP);

					double distance = frame.distanceTo(lastFrame);

					double disPercent = ((totalDistance) * 100) / distToStartPos;

					_udpHelper.sendInfoMessage(UserCmd.PROGRESS.toString() + ";" + disPercent);

					int indexOfLastExecutedFrame = allMotions.indexOf(motionContainer.getLastExecutedMotion());
					
					
					if (suspend) {
						distance = 0;
						try {
							if (indexOfLastExecutedFrame <= lastValue-1) {
								index = indexOfLastExecutedFrame;
								totalDistance = dist.get(index);
							} else if (indexOfLastExecutedFrame >  lastValue -1) {
								index = ((2 * lastValue) - 1 - indexOfLastExecutedFrame);
								totalDistance = dist.get(index);
							}

						} catch (Exception e) {
							logger.info("ROBERT: Index error while finding the current distance");
						}
						

						ThreadUtil.milliSleep(100);
						continue;

					}

					if (indexOfLastExecutedFrame >= lastValue) {
						totalDistance -= distance;
					} else if (indexOfLastExecutedFrame == 0 && totalDistance != 0) {
						totalDistance = 0;
					} else {
						totalDistance += distance;
					}

					lastFrame = frame;

				}
				ThreadUtil.milliSleep(50);
			}

		}

	}

	@Override
	public void end() {
		
		suspend = false;
		
		if (activity != null) {
			activity.stop();
		}
		if (bounceConditionControl != null) {
			bounceConditionControl.stop();
		}

		isMotionCancelled = true;

		if (remainingMotions != null) {
			remainingMotions.clear();
		}
		
		if (this.degree != null) {
			this.degree.isEndPressed = true;
		}


		stopUdpComunication();
		
		bodyIOGroup.setEnd(true);
		
	}

	@Override
	public void skipMotion() { 
		
		while(!motionContainer.isFinished()){
			motionContainer.cancel();
			ThreadUtil.milliSleep(100);
		}
		if (activity != null) {
			activity.stop();
		}
		if (bounceConditionControl != null) {
			bounceConditionControl.stop();
		}

		isMotionCancelled = true;
		remainingMotions.clear();

		//bodyIOGroup.setEnd(true);

		if (this.degree != null) {
			this.degree.isEndPressed = true;
		}

		stopUdpComunication();
	}

	@Override
	public void update(int _cycles, int _resistance) {
		this.cycles = _cycles;

	}

	/**
	 * pause the motion by setting the output signal of the linkage of the green
	 * light to off
	 */
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

	private double getFullDistance() {

		double dis = 0;

		for (int i = 0; i < this.frameList.size() - 1; i++) {

			dis = dis + this.frameList.get(i).distanceTo(this.frameList.get(i + 1));
		}
		return dis;

	}

	public double activePercentage() {
		/**
		 * calculate the average of the activeDistance
		 */
		double percent = 0;

		try {
			double avarageActivity = this.activeDistance / (this.cycles);
			percent = (avarageActivity / this.fullDisatance) * 100;

		} catch (Exception e) {
			logger.info("ROBERT: cannot calculate the persentage" + e);
		}

		if (Double.isNaN(percent) || Double.isInfinite(percent)) {
			return 0;
		}

		return percent;
	}

}
