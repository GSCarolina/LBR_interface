package guidedMode;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.TimeUnit;

import shared.ForceTorqueDataSetting;
import shared.LedColor;
import shared.LedColor.Color;
import shared.MeasuredTorquesMonitor;
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
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.IFiredConditionInfo;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
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

public class PathDataRecorder extends Training implements Runnable {

	private ArrayList<Frame> frameList;
	private IApplicationData appData;
	private LedColor ledColor;
	private double velo;
	private double acce;
	private ForceTorqueDataSetting forceTorqueDataSetting;
	private TCPConnection tcpConnection;
	private BooleanIOCondition patientButton;
	private ArrayList<Double> pathWeights;
	private ArrayList<TorqueSensorData> pathRegisteredTorques;
	private ArrayList<Frame> pathFrames;
	private ArrayList<IMotion> motions;
	private ArrayList<Frame> path;
	private Frame stoppingPos;
	
	
	public void asyncErrorHandler(IApplicationControl applicationControl) {
		// Error handling of asynchronous motion
		IErrorHandler errorHandler = new IErrorHandler() {
			@Override
			public ErrorHandlingAction handleError(Device device, IMotionContainer failedContainer,
					List<IMotionContainer> canceledContainers) {
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

	public PathDataRecorder(ArrayList<Frame> _frameList, IApplicationData _appData, LinkageIOGroup _linkageIOGroup,
			Tool _linkage, LedColor _ledColor, ITaskLogger _logger, LBR _robot, BodyIOGroup _bodyIOGroup,
			TCPConnection _tcpConnection, IApplicationControl _appControl) {

		asyncErrorHandler(_appControl);

		this.logger = _logger;
		this.frameList = _frameList;
		this.appData = _appData;
		this.linkageIOGroup = _linkageIOGroup;
		this.ledColor = _ledColor;
		this.linkage = _linkage;
		this.robot = _robot;
		this.bodyIOGroup = _bodyIOGroup;
		this.tcpConnection = _tcpConnection;
		/*
		 * create impedance for bounce
		 */
	
		rotationalSoft = getRotationalSoft();
		softBounce = getSoftBounce();

		path = new ArrayList<Frame>(this.frameList);

	}

	/**
	 * logging the first path forces
	 */
	private boolean playAndLogForces(ArrayList<Frame> _path) {

		ledColor.startFlashing(Color.GREEN);

		eStopPressed = false;
		Input playBtn = bodyIOGroup.getInput("PlayPauseButton");
		patientButton = new BooleanIOCondition(playBtn, true);

		Output pause = this.bodyIOGroup.getOutput("PauseControl");
		BooleanIOCondition pauseButton = new BooleanIOCondition(pause, true); 

		ThreadUtil.milliSleep(100);
		
		/**
		 * calculate the motion for the path that will be logged
		 */
		motions = calculateGuidedPath(_path);
		
		Spline motionSpline = getSplineMotion(motions);
		isMotionCancelled = false;

		do {
			this.bodyIOGroup.setPauseControl(false);

			robot.setESMState("3");
			velo = this.appData.getProcessData("velo").getValue();
			acce = this.appData.getProcessData("acce").getValue();
			// jerk = this.appData.getProcessData("jerk").getValue();

			if (forceTorqueDataSetting.isSuspended()) {
				forceTorqueDataSetting.resume();
			} 

			motionContainer = recTCP.moveAsync(motionSpline.setCartVelocity(velo).setJointAccelerationRel(acce)
					.breakWhen(pauseButton).breakWhen(patientButton));

			/**
			 * start torque monitoring here to make sure that the weak axes not
			 * been stressed, the thread will stop the motion controller.
			 */
			MeasuredTorquesMonitor mesure = new MeasuredTorquesMonitor(robot, logger);
			Thread t = new Thread(mesure);
			t.start();

			motionContainer.await();
			this.stoppingPos= robot.getCurrentCartesianPosition(linkage.getFrame("/RecTCP"));
			// bodyIOGroup.setPauseControl(false);
			IFiredConditionInfo firedInfo = motionContainer.getFiredBreakConditionInfo();

			if (firedInfo != null) {
				/**
				 * suspend the threads for measuring torques and weights if
				 * conditions were broken
				 */
				forceTorqueDataSetting.suspend();

				robot.setESMState("2");
				motionSpline = resumeMotion(motionContainer, motions, _path, this.stoppingPos);

				if (motionSpline == null) {
					forceTorqueDataSetting.stop();
					break;
				}

				ICondition firedCond = firedInfo.getFiredCondition();

				if (firedCond.equals(pauseButton) || firedCond.equals(patientButton)) {
					if (firedCond.equals(patientButton)) {
						this.bodyIOGroup.setPauseControl(true);
						try {
							tcpConnection.sendMessage(UserCmd.PAUSED.toString());
						} catch (IOException e) {
							this.logger.info("ROBERT: Failed to send PAUSED message to the control interfase");
						}
					}

					while (bodyIOGroup.getPlayPauseButton()) {
						ThreadUtil.milliSleep(100);
					}


					this.position = robot.getCurrentJointPosition();

					softMc = robot.moveAsync(positionHold(softBounce, -1, TimeUnit.MILLISECONDS));

					/*
					 * -----------------------------------------------
					 * ---Controlling the arm under Bounce mode ------
					 * -----------------------------------------------
					 */

					// waiting for interaction

					if (waitForInteraction(robot, softMc, softBounce ,rotationalSoft, position, recTCP, linkageIOGroup, bodyIOGroup, ledColor, logger,
							false)) {

						//ledColor.stopFlashingNow();
						forceTorqueDataSetting.stop();
						this.bodyIOGroup.setPauseControl(false);
						return false;
					}

					while (bodyIOGroup.getPlayPauseButton()) {
						ThreadUtil.milliSleep(100);
					}

					try {
						tcpConnection.sendMessage(UserCmd.RESUME.toString());
					} catch (IOException e) {
						this.logger.info("ROBERT: Failed to send PAUSED message to the control interfase");
					}

					// finish softMc anyway
					while (!softMc.isFinished()) {
						softMc.cancel();
					}

					if (isMotionCancelled) {
						forceTorqueDataSetting.stop();
						motions.clear();
						break;
					} else {
						forceTorqueDataSetting.resume();
					}

				}
			} else if (isTorqueExceeded) {

				ledColor.stopFlashing();

				ThreadUtil.milliSleep(100);

				ledColor.startFlashing(Color.YELLOW);

				forceTorqueDataSetting.stop();

				robot.getCurrentJointPosition();

				ThreadUtil.milliSleep(100);
				robot.setESMState("3");
				ThreadUtil.milliSleep(100);
				robot.setESMState("2");

				softMc = robot.moveAsync(positionHold(softBounce, -1, TimeUnit.MILLISECONDS));

				try {
					this.tcpConnection.sendMessage(UserCmd.LOGGING_CANCEL.toString());
				} catch (IOException e) {
					logger.info("ROBERT: Failed to send logging canceled to the control interface" + e);
				}

				if (waitForInteraction(robot, softMc, softBounce , rotationalSoft, position, recTCP, linkageIOGroup, bodyIOGroup, ledColor, logger, true)) {
					//ledColor.stopFlashingNow();
					isTorqueExceeded = false;
					this.bodyIOGroup.setPauseControl(false);
					return false;
				}

			} else {
				isMotionCancelled = false;

				motions.clear();

				forceTorqueDataSetting.stop();

				ledColor.stopFlashing();

				ThreadUtil.milliSleep(100);

				linkageIOGroup.setLEDLightGreen(false);
				linkageIOGroup.setLEDLightRed(false);

				// bodyIOGroup.setPauseControl(false);

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
				try {
					 recTCP.move(positionHold(new PositionControlMode(), 100, TimeUnit.MILLISECONDS));
				} catch (Exception e) {
					logger.info("ROBERT: ExecutioinController failed to run position hold of the helper function" + e);
				}

				break;
			}

		} while (!remainingMotions.isEmpty());
		/**
		 * stopping the threads for measuring torques and weights
		 */
		forceTorqueDataSetting.stop();
		return true;

	}


	private ArrayList<Frame> getFrames() {
		return forceTorqueDataSetting.getPathFrames();
	}

	public ArrayList<Frame> getPathFrames() {

		return pathFrames;
	}

	/**
	 * get the torque values
	 * 
	 * @return {@link HashMap} of [Integer, TorqueSensorData]
	 */
	private ArrayList<TorqueSensorData> getRegisteredTorques() {
		return forceTorqueDataSetting.getRegisteredTorques();
	}

	public ArrayList<TorqueSensorData> getPathRegisteredTorques() {

		return pathRegisteredTorques;
	}

	private ArrayList<Double> getWeights() {
		return forceTorqueDataSetting.getWeights();
	}

	public ArrayList<Double> getPathWeights() {

		return pathWeights;
	}

	@Override
	public void run() {

		forceTorqueDataSetting = new ForceTorqueDataSetting(path, this.robot, this.linkage, this.logger);
		ThreadUtil.milliSleep(100);
		
		if (forceTorqueDataSetting != null) {

			/**
			 * log the fist motion which returning to start position motion
			 */

			if (playAndLogForces(path)) {

				forceTorqueDataSetting.stop();

				/**
				 * copy the back trajectory data
				 */
				ArrayList<Frame> pathFrames = getFrames();

				ArrayList<Frame> backArray = new ArrayList<Frame>(pathFrames);
				this.pathFrames = new ArrayList<Frame>(pathFrames);
				Collections.reverse(this.pathFrames);
				this.pathFrames.addAll(backArray);

				ArrayList<TorqueSensorData> loggedTorques = getRegisteredTorques();

				ArrayList<TorqueSensorData> backTorques = new ArrayList<TorqueSensorData>(loggedTorques);
				this.pathRegisteredTorques = new ArrayList<TorqueSensorData>(loggedTorques);
				Collections.reverse(this.pathRegisteredTorques);
				this.pathRegisteredTorques.addAll(backTorques);

				ArrayList<Double> loggedWeights = getWeights();

				ArrayList<Double> backWeights = new ArrayList<Double>(loggedWeights);
				this.pathWeights = new ArrayList<Double>(loggedWeights);
				Collections.reverse(this.pathWeights);
				this.pathWeights.addAll(backWeights);

				/**
				 * logging done message
				 */
				try {
					this.tcpConnection.sendMessage(UserCmd.LOGGING_DONE.toString());
				} catch (IOException e) {
					e.printStackTrace();
				}
				
				try {
					recTCP.move(positionHold(new PositionControlMode(), 100, TimeUnit.MILLISECONDS));
				} catch (Exception e) {
					logger.info("ROBERT: PathDataRecorder failed to log the motion: " + e);
				}
			
			} else {
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
			}
		}

	}

	@Override
	public void pause() {
		this.bodyIOGroup.setPauseControl(true);
	}

	@Override
	public void resume() {
		this.bodyIOGroup.setPauseControl(false);
	}

	@Override
	public Spline getSwitchToPath(boolean isNextMotion) {

		return null;
	}

	@Override
	public void end() {
		
	}
}
