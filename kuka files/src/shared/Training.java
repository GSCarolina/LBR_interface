package shared;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.spl;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.handGuiding;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

import main.Robert_2_3;
import util.UdpHelper;
import util.UserCmd;
import backgroundTask.StateHelper;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.BodyIOGroup;
import com.kuka.generated.ioAccess.LinkageIOGroup;
import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.EnablingDeviceState;
import com.kuka.roboticsAPI.deviceModel.Device;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.IFiredConditionInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.ioModel.Input;
import com.kuka.roboticsAPI.motionModel.ErrorHandlingAction;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.IErrorHandler;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineMotionCP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.task.ITaskLogger;

/**
 * @author Firas Amin
 * 
 */

enum STATE {
	Bounce, Pause
}

public class Training implements TrainingInterface {

	@Inject
	protected IApplicationData appData;
	@Inject
	protected LBR robot;
	@Inject
	protected LinkageIOGroup linkageIOGroup;
	@Inject
	protected ITaskLogger logger;

	@Inject
	protected BodyIOGroup bodyIOGroup;

	@Inject
	@Named("Linkage")
	protected Tool linkage;
	protected static ObjectFrame recTCP;

	public static IMotionContainer motionContainer;
	public static IMotionContainer softMc;
	public static ArrayList<IMotion> remainingMotions;
	protected static boolean eStopPressed;
	public static boolean isMotionCancelled;
	protected static boolean isTorqueExceeded;

	protected ArrayList<Frame> framesArray;
	protected TrackRec recorder;

	private ArrayList<Frame> trackPoints;
	private Torques recTorques;
	protected ICondition maxTorqueCondition;
	private boolean isStarted;
	private Spline motionSpline;
	public ICondition stopCondition;
	private LBR _lbrMed;
	protected TCPConnection tcpConnection;
	protected ArrayList<Frame> oneWayPathArray;
	protected Degree degree;
	protected JointPosition position;
	protected LedColor ledColor;
	private int indexOfLastStoppingFrame = 0;
	private int indexOfLastExecutedFrame;
	protected CartesianImpedanceControlMode rotationalSoft;
	protected CartesianImpedanceControlMode softBounce;
	protected CartesianImpedanceControlMode hardImpedance;
	private IMotionContainer handy;
	protected static boolean isPlayOrEndPressed;


	protected UdpHelper _udpHelper;
	private ExecutorService _udpReceiverThread;

	protected TCPConnection getTcpConnection() {
		return tcpConnection;
	}

	public void setTcpConnection(TCPConnection tcpConnection) {
		this.tcpConnection = tcpConnection;
	}

	public BodyIOGroup getBodyIOGroup() {
		return bodyIOGroup;
	}

	public void setBodyIOGroup(BodyIOGroup bodyIOGroup) {
		this.bodyIOGroup = bodyIOGroup;
	}

	public LBR getlbrMed() {
		return _lbrMed;
	}

	public void setlbrMed(LBR _lbrMed) {
		this._lbrMed = _lbrMed;
	}

	private static final double[] JOINT_LIMITS_MAX = { Math.toRadians(160), Math.toRadians(60), Math.toRadians(30), Math.toRadians(-30),
			Math.toRadians(65), Math.toRadians(90), Math.toRadians(130) };
	private static final double[] JOINT_LIMITS_MIN = { Math.toRadians(-160), Math.toRadians(-10), Math.toRadians(-30), Math.toRadians(-105),
			Math.toRadians(-95), Math.toRadians(10), Math.toRadians(-130) };

	private static final HandGuidingMotion handguiding = handGuiding().setJointLimitsMax(JOINT_LIMITS_MAX).setJointLimitsMin(JOINT_LIMITS_MIN)
			.setJointLimitsEnabled(true, true, true, true, true, true, true).setJointVelocityLimit(50).setCartVelocityLimit(700)
			.setJointLimitViolationFreezesAll(false).setPermanentPullOnViolationAtStart(true);

	@Override
	public void moveMe() {

	}

	public ArrayList<Frame> pathRecorder() {
		// ThreadUtil.milliSleep(200);
		return trajectoryRecorder();
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
				return ErrorHandlingAction.Ignore;
			}
		};
		applicationControl.registerMoveAsyncErrorHandler(errorHandler);

	}

	@Override
	public ArrayList<Frame> trajectoryRecorder() {

		appData.getProcessData("trackPath").setValue(true);
		recorder = new TrackRec(robot, linkage, appData);
		recorder.start();

		guiding();
		logger.info(" after guided ");
		appData.getProcessData("trackPath").setValue(false);

		trackPoints = recorder.getList();

		return trackPoints;
	}

	protected boolean startUdpCommunication(final int _LOCAL_PORT) {
		if (null == _udpHelper) {
			_udpHelper = new UdpHelper(_LOCAL_PORT, "DefaultApplication", this.logger, null );
			if (!_udpHelper.initUDP()) {
				return false;
			}

			// create additional thread to receive UDP packages the code of the
			// thread is defined in UdpHelper.run()
			_udpReceiverThread = Executors.newSingleThreadExecutor();
			_udpReceiverThread.execute(_udpHelper);
		}

		return true;
	}

	protected boolean stopUdpComunication() {

		if (!_udpReceiverThread.isShutdown()) {

			_udpReceiverThread.shutdown();
		}
		return false;
	}

	@Override
	public void guiding() {
		recTCP = linkage.getFrame("/RecTCP");
		logger.info("ROBERT: the hand guiding maximum limits are: " + Math.toDegrees(JOINT_LIMITS_MAX[0]) + ", "
				+ Math.toDegrees(JOINT_LIMITS_MAX[1]) + ", " + Math.toDegrees(JOINT_LIMITS_MAX[2]) + ", " + Math.toDegrees(JOINT_LIMITS_MAX[3])
				+ ", " + Math.toDegrees(JOINT_LIMITS_MAX[4]) + ", " + Math.toDegrees(JOINT_LIMITS_MAX[5]) + ", "
				+ Math.toDegrees(JOINT_LIMITS_MAX[6]) + ", " + "and the minimum limits are: " + Math.toDegrees(JOINT_LIMITS_MIN[0]) + ", "
				+ Math.toDegrees(JOINT_LIMITS_MIN[1]) + ", " + Math.toDegrees(JOINT_LIMITS_MIN[2]) + ", " + Math.toDegrees(JOINT_LIMITS_MIN[3])
				+ ", " + Math.toDegrees(JOINT_LIMITS_MIN[4]) + ", " + Math.toDegrees(JOINT_LIMITS_MIN[5]) + ", "
				+ Math.toDegrees(JOINT_LIMITS_MIN[6]) + ", ");

		do {

			logger.info("ready to move");
			// if (robot.getSafetyState().getEnablingDeviceState() ==
			// EnablingDeviceState.HANDGUIDING) {

			handy = recTCP.moveAsync(handguiding);
			logger.info("finished moving ");
			controlGuidedSignal();
			handy.await();

		} while (robot.getSafetyState().getEnablingDeviceState() == EnablingDeviceState.HANDGUIDING);
	}

	public void controlGuidedSignal() {
		new Thread(new Runnable() {

			@Override
			public void run() {
				// logger.info("inside run ");
				ThreadUtil.milliSleep(200);
				while (robot.getSafetyState().getEnablingDeviceState() == EnablingDeviceState.HANDGUIDING) {
					ThreadUtil.milliSleep(100);
				}

				while (!handy.isFinished()) {
					handy.cancel();
					ThreadUtil.milliSleep(100);
				}
				// logger.info("thread died ");
				return;

			}

		}).start();
	}

	@Override
	public ArrayList<IMotion> calculateGuidedPath(ArrayList<Frame> frameList) {

		remainingMotions = new ArrayList<IMotion>();

		/**
		 * reverse the path if it is needed to return to start position,
		 * otherwise it is the next motion switch path, move to start position
		 */

		// frameList.remove(frameList.size() - 1);

		for (int i = 0; i < frameList.size(); i++) {

			remainingMotions.add(spl(frameList.get(i)));
		}

		return remainingMotions;

	}

	@Override
	public ArrayList<IMotion> calculateGuidedPathWithVelocities(ArrayList<Frame> frameList, ArrayList<Double> velarray) {

		remainingMotions = new ArrayList<IMotion>();

		for (int i = 0; i < frameList.size(); i++) {

			if (i < velarray.size()) {
				remainingMotions.add(spl(frameList.get(i)).setCartVelocity(velarray.get(i)));
			} else {
				remainingMotions.add(spl(frameList.get(i)));
			}

		}

		return remainingMotions;
	}

	/**
	 * Finding the minimum distance between a frame and other position frames.
	 * It returns the position of the frame from which the minimum distance is
	 * founded.
	 * 
	 * @return Integer
	 */

	public int distance(ArrayList<Frame> trajectory, Frame current) {

		double distance = 0;
		double lastDistance = current.distanceTo(trajectory.get(0));
		int pos = 0;
		// Frame firstPos = trajectory.get(0);

		for (int i = 1; i < trajectory.size(); i++) {
			Frame posFrame = trajectory.get(i);
			distance = current.distanceTo(posFrame);

			if (Math.abs(distance) < lastDistance) {
				lastDistance = distance;
				pos = i;
			}
		}
		return pos;
	}

	/**
	 * Calculate the path as an array of motions for the active training
	 * 
	 * @param frameList
	 * @return {@link ArrayList[IMotion]}
	 */
	public ArrayList<IMotion> calculateActivePath(ArrayList<Frame> frameList) {

		remainingMotions = new ArrayList<IMotion>();

		frameList = filterTrack(frameList, 5.0);

		for (Frame frame : frameList) {
			remainingMotions.add(spl(frame));
		}

		return remainingMotions;
	}

	/**
	 * filters the trajectory in order to remove unnecessary points due to
	 * shaking hand while recording
	 * 
	 * @param trajectory
	 * @param minDistance
	 * @return
	 */

	public ArrayList<Frame> filterTrack(ArrayList<Frame> trajectory, double minDistance) {

		Frame previousPos = trajectory.get(0);

		ArrayList<Frame> trackFiltered = new ArrayList<Frame>();
		trackFiltered.add(previousPos);
		for (int i = 1; i < trajectory.size(); i++) {
			Frame currPos = trajectory.get(i);
			double distance = previousPos.distanceTo(currPos);
			if (Math.abs(distance) < minDistance) {
				continue;
			} else if (Math.abs(distance) >= minDistance) {
				trackFiltered.add(trajectory.get(i));
				previousPos = currPos;
			}
		}
		return trackFiltered;
	}

	public ArrayList<Double> velocityCalculation(ArrayList<Frame> trajectory, double minDistance) {
		ArrayList<Double> velArray = new ArrayList<Double>();
		int cnt = 1;

		Frame previousPos = trajectory.get(0);

		for (int i = 1; i < trajectory.size(); i++) {

			Frame currPos = trajectory.get(i);

			double clcDistance = previousPos.distanceTo(currPos);

			if (Math.abs(clcDistance) < minDistance) {
				cnt++;
				continue;
			} else if (Math.abs(clcDistance) >= minDistance) {
				double time = 100 * cnt * 0.001;

				double vel = clcDistance / time;
				if (vel > 200) {
					velArray.add(200.00);
				} else {
					velArray.add(vel);
				}

				previousPos = currPos;
				cnt = 1;
			}
		}
		return velArray;
	}

	/**
	 * Calculating the returning path from the current frame.
	 * 
	 * @param framesArray
	 * @param currentPosition
	 * @return
	 */
	public Spline getReturningPath(ArrayList<Frame> framesArray, Frame currentPosition) throws Exception {
		ArrayList<IMotion> motion = calculateActivePath(framesArray);
		// int pos = distance(trackPoints, currentPosition);
		int pos = distance(framesArray, currentPosition);

		// return a copy of the list from the founded frame
		// check why it does return empty array
		List<IMotion> newMotions = motion.subList(pos, motion.size());

		// adding current position to the motions list at the first position
		// newMotions.add(0, spl(currentPosition));

		ArrayList<IMotion> motionArray = new ArrayList<IMotion>(newMotions);

		return new Spline(motionArray.toArray(new SplineMotionCP[motionArray.size()]));

	}

	public Spline getRemainingPathToEndPos(ArrayList<Frame> framesArray) {

		Collections.reverse(framesArray);

		Frame currentPos = robot.getCurrentCartesianPosition(recTCP, World.Current.getRootFrame());

		int pos = distance(framesArray, currentPos);

		// ArrayList<IMotion> motion = calculateActivePath(framesArray);

		ArrayList<IMotion> motion = new ArrayList<IMotion>();

		for (Frame frame : framesArray) {
			motion.add(spl(frame));
		}

		List<IMotion> newMotions = motion.subList(pos, motion.size());

		// adding current position to the motions list at the first position
		// newMotions.add(0, spl(currentPos));
		ArrayList<IMotion> motionArray = new ArrayList<IMotion>(newMotions);

		return new Spline(motionArray.toArray(new SplineMotionCP[motionArray.size()]));
	}

	/**
	 * move to kickoff position
	 */
	void returnTokickoffPosition() {

		// ArrayList<IMotion> motion = calculateActivePath(this.framesArray);

		ArrayList<IMotion> motion = new ArrayList<IMotion>();

		for (Frame frame : this.oneWayPathArray) {
			motion.add(spl(frame));
		}
		Spline spline = getSplineMotion(motion);
		double[] velocity = new double[] { 0.2, 0.2, 0.2, 0.3, 0.2, 0.2, 0.2 };
		recTCP.move(spline.setJointVelocityRel(velocity).setJointAccelerationRel(0.01));
	}

	/**
	 * move to the end position after guide motion
	 */
	void returnToEndPosition() {
		Collections.reverse(this.oneWayPathArray);

		ArrayList<IMotion> motion = new ArrayList<IMotion>();

		for (Frame frame : this.oneWayPathArray) {
			motion.add(spl(frame));
		}
		Spline spline = getSplineMotion(motion);
		double[] velocity = new double[] { 0.2, 0.2, 0.2, 0.3, 0.2, 0.2, 0.2 };
		recTCP.move(spline.setJointVelocityRel(velocity).setJointAccelerationRel(0.01));
	}

	/**
	 * Finding the closest frame and return the path from the founded frame to
	 * the kickoff position. Motion spline from the position that is been found,
	 * or adding the frame at the first position and returning the spline
	 * motion. The position is been projected in the y-axis
	 * 
	 * @param
	 * @return Spline motion from the frame that is been found
	 */
	public Spline returningPath(Frame currentPosition) {
		Frame frame = currentPosition.copy().setX(0).setZ(0).setAlphaRad(0).setBetaRad(0).setGammaRad(0);
		int newPos = 0;
		int lastPos = 0;
		int cnt = 0;
		boolean isFounded = false;

		// int pos = distance(trajectory)

		// the movement path is running from minus to positive
		if (trackPoints.get(0).getY() <= trackPoints.get(trackPoints.size()).getY()) {

			// the movement path is running form positice to minus
		} else {

		}
		// starting from the first position
		for (Frame position : trackPoints) {

			// in case of the motion path is running from minus to plus, return
			// the position that is just after the current frame position
			if (position.getY() >= frame.getY()) {

				// return a copy of the list from the founded frame
				List<IMotion> newMotions = remainingMotions.subList(cnt, remainingMotions.size());

				// adding current position to the motions list at the first
				// position
				newMotions.add(0, spl(frame));

				ArrayList<IMotion> motionArray = new ArrayList<IMotion>(newMotions);

				return new Spline(motionArray.toArray(new SplineMotionCP[motionArray.size()]));

				// in case of the motion path is running from plus to minus,
				// return the position that is just after the current frame
				// position
			}
			cnt++;
			// if
			// (position.copy().setX(0).setZ(0).setAlphaRad(0).setBetaRad(0).setGammaRad(0).distanceTo(frame)
			// <= 50) {

			//
			if (Math.abs(position.getY()) >= Math.abs(frame.getY())) {
				lastPos = newPos;
				isFounded = true;
				break;
			}

			break;

			// }
			// newPos++;
		}
		// logger.info("Robert: motions" + remainingMotions.toString());

		if (isFounded) {
			if (lastPos == -1 || (lastPos) >= remainingMotions.size()) {
				// logger.info("Robert: run out of segments");
				return null;
			}

			remainingMotions.set(lastPos, spl(trackPoints.get(lastPos)));
			ArrayList<IMotion> motionArray = new ArrayList<IMotion>(remainingMotions.subList(lastPos, remainingMotions.size()));
			// }
			// logger.info("Robert: motions size after: " + motionArray.size());
			return new Spline(motionArray.toArray(new SplineMotionCP[motionArray.size()]));

		} else {
			remainingMotions.add(0, spl(frame));
			ArrayList<IMotion> motionArray = new ArrayList<IMotion>(remainingMotions);
			return new Spline(motionArray.toArray(new SplineMotionCP[motionArray.size()]));
		}
	}

	protected ArrayList<IMotion> goToNextMotion(ArrayList<Frame> frameList) {

		remainingMotions = new ArrayList<IMotion>();

		for (Frame frame : frameList) {
			remainingMotions.add(spl(frame));
		}
		return remainingMotions;
	}

	protected Spline resumeMotion(IMotionContainer motionContainer) {

		// logger.info("Robert: motions size before: " + motionFrames.size());
		int indexOfLastFrame = remainingMotions.indexOf(motionContainer.getLastExecutedMotion());
		logger.info("Index of the last motion is: " + indexOfLastFrame);

		if (indexOfLastFrame == -1 || (indexOfLastFrame) >= remainingMotions.size()) {
			logger.info("Robert: run out of segments");
			return null;
		}

		remainingMotions.set(indexOfLastFrame, spl(trackPoints.get(indexOfLastFrame)));
		ArrayList<IMotion> motionArray = new ArrayList<IMotion>(remainingMotions.subList(indexOfLastFrame, remainingMotions.size()));
		// }
		// logger.info("Robert: motions size after: " + motionArray.size());
		return new Spline(motionArray.toArray(new SplineMotionCP[motionArray.size()]));
	}

	protected Spline getSplineMotion(ArrayList<IMotion> _allMotions) {

		return new Spline(_allMotions.toArray(new SplineMotionCP[_allMotions.size()]));
	}

	protected Spline resumeMotion(IMotionContainer _motionContainer, ArrayList<IMotion> _allMotions, ArrayList<Frame> frameList, Frame _stoppingPos) {

		// logger.info("Robert: motions size before: " + motionFrames.size());
		indexOfLastExecutedFrame = _allMotions.indexOf(_motionContainer.getLastExecutedMotion());
		logger.info("Index of the last motion is: " + indexOfLastExecutedFrame);

		if (indexOfLastExecutedFrame == -1 || indexOfLastExecutedFrame >= _allMotions.size()) {
			logger.info("Robert: run out of segments");
			return null;
		}

		if ((indexOfLastExecutedFrame) == _allMotions.size() - 1) {

			_allMotions.add(_allMotions.get(indexOfLastExecutedFrame));
		}

		// motions.set(indexOfLastFrame, spl(frameList.get(indexOfLastFrame)));
		remainingMotions = new ArrayList<IMotion>(_allMotions.subList(indexOfLastExecutedFrame, _allMotions.size()));
		return new Spline(remainingMotions.toArray(new SplineMotionCP[remainingMotions.size()]));
	}

	protected Spline resumeMotion(IMotionContainer _motionContainer, ArrayList<IMotion> _allMotions, ArrayList<Frame> frameList, Frame _stoppingPos,
			Boolean bounce) {

		// logger.info("Robert: motions size before: " + motionFrames.size());
		indexOfLastExecutedFrame = _allMotions.indexOf(_motionContainer.getLastExecutedMotion());
		logger.info("Index of the last motion is: " + indexOfLastExecutedFrame);

		if (indexOfLastExecutedFrame == -1 || indexOfLastExecutedFrame >= _allMotions.size()) {
			logger.info("Robert: run out of segments");
			return null;
		}

		if ((indexOfLastExecutedFrame) == _allMotions.size() - 1) {

			_allMotions.add(_allMotions.get(indexOfLastExecutedFrame));
		}

		if (indexOfLastStoppingFrame == indexOfLastExecutedFrame && indexOfLastExecutedFrame - 1 >= 0) {

			indexOfLastExecutedFrame--;
		}
		indexOfLastStoppingFrame = indexOfLastExecutedFrame;
		// motions.set(indexOfLastFrame, spl(frameList.get(indexOfLastFrame)));
		remainingMotions = new ArrayList<IMotion>(_allMotions.subList(indexOfLastExecutedFrame, _allMotions.size()));
		return new Spline(remainingMotions.toArray(new SplineMotionCP[remainingMotions.size()]));
	}

	@Override
	public void resetData() {
		recorder = null;
		trackPoints = null;
		remainingMotions = null;
		logger.info("Robert: \nRecorded point sets: " + trackPoints);
		isStarted = false;
	}

	protected void startTorquesRecording() {
		recTorques = new Torques(robot, logger, appData);
		recTorques.setRecTorques(true);
		recTorques.start();

	}

	protected void pauseTorquesRecording() {
		recTorques.setPause(true);

	}

	protected void resumeTorquesRecording() {
		recTorques.setPause(false);

	}

	protected void stopTorquesRecording() {
		recTorques.setRecTorques(false);
		recTorques.setPause(false);

	}

	/**
	 * call the method prepareBounceCondition(ArrayList<double[]> torqueArray)
	 * in Torques class for preparing the Bounce condition
	 * 
	 * @param registeredTorques
	 * @param _appData
	 * @return the condition for the Bounce
	 */
	protected ICondition prepareBounceCondition(ArrayList<TorqueSensorData> registeredTorques, IApplicationData _appData) {
		this.appData = _appData;
		recTorques = new Torques(robot, logger, _appData);
		return recTorques.prepareBounceCondition(registeredTorques);
	}

	//
	// protected double[] getMaxTorques() {
	// maxTorqueCondition = recTorques.prepareConditon();
	// //stopCondition = recTorques.getStopTorqueCondition();
	// return recTorques.calcMaxTorque();
	// }

	@Override
	public boolean torqueRecorder() {
		// lbr.setESMState("3");
		double velo = appData.getProcessData("velo").getValue();
		double acce = appData.getProcessData("acce").getValue();
		appData.getProcessData("jerk").getValue();

		new JointImpedanceControlMode(1000.0, 5000.0, 1000.0, 5000.0, 1000.0, 5000.0, 100.0);
		ICondition pauseCond = null;

		if (!isStarted) {
			isStarted = true;

			motionSpline = getSplineMotion(null);

			// tcp = linkage.getFrame("TCP");
			// recTCP = linkage.getFrame("RecTCP");

			startTorquesRecording();

		} else {
			resumeTorquesRecording();
		}

		Input playBtn = getBodyIOGroup().getInput("PlayPauseButton");
		pauseCond = new BooleanIOCondition(playBtn, false);

		motionContainer = recTCP.moveAsync(motionSpline.setCartVelocity(velo).setJointAccelerationRel(acce).breakWhen(pauseCond));

		// start torque monitoring here.
		// new MeasuredTorquesMonitor(robot, logger).start();

		motionContainer.await();

		IFiredConditionInfo firedInfo = motionContainer.getFiredBreakConditionInfo();

		if (firedInfo != null) {
			robot.setESMState("2");

			motionSpline = resumeMotion(motionContainer);

			if (motionSpline == null) {
				stopTorquesRecording();
				isStarted = false;
				return false;
			}

			ICondition firedCond = firedInfo.getFiredCondition();

			if (firedCond.equals(pauseCond)) {
				pauseTorquesRecording();
			}

		} else if (motionContainer.isFinished() && !motionContainer.hasError()) {

			// int indexOfLastFrame =
			// remainingMotions.indexOf(motionContainer.getLastExecutedMotion());

			if (remainingMotions.get(remainingMotions.size() - 1) == motionContainer.getLastExecutedMotion()) {

				logger.info("Robert: motion spline: " + motionSpline.getMotions());
				stopTorquesRecording();
				isStarted = false;
				motionSpline = null;
				return true;
			}

		}

		return false;

	}

	protected synchronized boolean waitForInteraction(LBR _robot, IMotionContainer _softMc, CartesianImpedanceControlMode _softBounce,
			CartesianImpedanceControlMode _rotationalSoft, JointPosition _position, ObjectFrame _recTCP, LinkageIOGroup _linkageIOGroup,
			BodyIOGroup _bodyIOGroup, LedColor _ledColor, ITaskLogger _logger, boolean controlRecordBtn) {

		this.robot = _robot;
		this.bodyIOGroup = _bodyIOGroup;
		this.linkageIOGroup = _linkageIOGroup;
		softMc = _softMc;
		ledColor = _ledColor;

		this.position = _position;
		// start monitoring of the bounce
		degree = new Degree(robot, softMc, _softBounce, position, _recTCP, bodyIOGroup, _logger);
		degree.start();

		if (isTorqueExceeded) {
			while (true) {
				if (hasPressedAttach()) {
					return true;
				} else if (controlRecordBtn && this.linkageIOGroup.getRecordButton() && Robert_2_3.userCommand == UserCmd.ADD
						|| Robert_2_3.system_Check_Needed == true) {
					while (!softMc.isFinished()) {
						softMc.cancel();
						ThreadUtil.milliSleep(100);
					}
					while (!motionContainer.isFinished()) {
						motionContainer.cancel();
						ThreadUtil.milliSleep(100);
					}
					isMotionCancelled = true;
					remainingMotions.clear();
					Robert_2_3.system_Check_Needed = false;
					return true;

				}
				ThreadUtil.milliSleep(100);
			}
		} else {

			isPlayOrEndPressed = false;

			while (!isPlayOrEndPressed) {
				if (hasPressedAttach() || Robert_2_3.system_Check_Needed) {
					bodyIOGroup.setPauseControl(false);
						
					Robert_2_3.system_Check_Needed = false;
						
					return true;
				}
				ThreadUtil.milliSleep(100);
			}
			return false;
		}
	}

	protected CartesianImpedanceControlMode getRotationalSoft() {
		rotationalSoft = new CartesianImpedanceControlMode();
		rotationalSoft.parametrize(CartDOF.Z).setStiffness(5000);
		rotationalSoft.parametrize(CartDOF.X).setStiffness(5000);
		rotationalSoft.parametrize(CartDOF.Y).setStiffness(5000);
		rotationalSoft.parametrize(CartDOF.ROT).setStiffness(70.0);
		rotationalSoft.parametrize(CartDOF.ALL).setDamping(1.0);
		return rotationalSoft;
	}

	protected CartesianImpedanceControlMode getSoftBounce() {
		softBounce = new CartesianImpedanceControlMode();
		softBounce.parametrize(CartDOF.Z).setStiffness(3000);
		softBounce.parametrize(CartDOF.X).setStiffness(800);
		softBounce.parametrize(CartDOF.Y).setStiffness(800);
		softBounce.parametrize(CartDOF.ROT).setStiffness(300.0);
		softBounce.parametrize(CartDOF.ALL).setDamping(1.0);
		return softBounce;
	}

	protected CartesianImpedanceControlMode getHardImpedance() {
		hardImpedance = new CartesianImpedanceControlMode();
		hardImpedance.parametrize(CartDOF.Z).setStiffness(5000);
		hardImpedance.parametrize(CartDOF.X).setStiffness(5000);
		hardImpedance.parametrize(CartDOF.Y).setStiffness(5000);
		hardImpedance.parametrize(CartDOF.ROT).setStiffness(300.0);
		hardImpedance.parametrize(CartDOF.ALL).setDamping(1.0);
		return hardImpedance;
	}

	private boolean hasPressedAttach() {
		int cnt = 1;
		while ((robot.getSafetyState().getEnablingDeviceState() == EnablingDeviceState.HANDGUIDING && !linkageIOGroup.getRecordButton())
				|| StateHelper.isEstopPressed) {
			ThreadUtil.milliSleep(10);

			if (cnt == 5) {
				while (!softMc.isFinished()) {
					softMc.cancel();
					ThreadUtil.milliSleep(100);
				}
				while (!motionContainer.isFinished()) {
					motionContainer.cancel();
					ThreadUtil.milliSleep(100);
				}
				isMotionCancelled = true;
				remainingMotions.clear();
				ledColor.stopFlashingNow();
				return true;
			}
			cnt++;
		}
		return false;
	}

	@Override
	public void end() {
	}

	@Override
	public void update(int cycle, int resistance) {
	}

	@Override
	public Spline getSwitchToPath(boolean isNextMotion) {
		return null;
	}

	@Override
	public void pause() {
	}

	@Override
	public void resume() {
	}

	@Override
	public int play(int cycles, int resistance) {
		return 0;
	}

	@Override
	public void skipMotion() {

	}

}
