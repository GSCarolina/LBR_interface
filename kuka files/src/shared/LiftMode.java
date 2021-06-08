package shared;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.IFiredConditionInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.task.ITaskLogger;

public class LiftMode {

	@Inject
	private LBR robot;

	@Inject
	@Named("Linkage")
	private Tool linkage;

	@Inject
	private ITaskLogger logger;

	private Frame attachPosFrame;
	public Frame startPosCalc;
	public Frame endPosCalc;

	final int maxload = 14; // Maximal payLoad in kilogram

	final double startPosHeight = 120.0; // Distance for measuring weight

	final double endPosHeight = 50.0; // Distance for end position

	private static double weight;
	private ObjectFrame tcp;
	double forceInZ;

	// private CartesianImpedanceControlMode soft;

	private Frame currentPosFrame;

	private IMotionContainer motionCmd;

	private boolean limitReached;
	private AxesMonitor axesMonitor;

	public boolean liftup() {

		// Define impedance mode when collision is detected
		CartesianImpedanceControlMode soft = new CartesianImpedanceControlMode();
		soft.parametrize(CartDOF.Z).setStiffness(5000.0);
		soft.parametrize(CartDOF.X).setStiffness(1000);
		soft.parametrize(CartDOF.Y).setStiffness(200);
		soft.parametrize(CartDOF.ROT).setStiffness(300.0);
		soft.parametrize(CartDOF.ALL).setDamping(1.0);

		// Define overload condition
		tcp = linkage.getFrame("/RecTCP");

		ForceCondition overload_14kg = ForceCondition.createSpatialForceCondition(tcp, (maxload * 9.82), 10);

		attachPosFrame = robot.getCurrentCartesianPosition(tcp, World.Current.getRootFrame());

		// start position is 10cm above the bed
		startPosCalc = attachPosFrame.copyWithRedundancy();
		startPosCalc.setZ(startPosCalc.getZ() + startPosHeight);

		// end position is 5cm above the bed
		endPosCalc = attachPosFrame.copyWithRedundancy();
		endPosCalc.setZ(endPosCalc.getZ() + endPosHeight);

		this.limitReached = false;


			/**
			 * Creating anonymous class because there is no concrete class (abstract class), then monitor axes while moving up
			 */
			axesMonitor = new AxesMonitor() {

				@Override
				public void hasReachedAxesLimit(boolean isLimitReached) {
					limitReached = isLimitReached;
					while (!motionCmd.isFinished()) {
						motionCmd.cancel();
						ThreadUtil.milliSleep(100);
						
					}
					logger.info("Robert: axis limit reached");
				}
			};

			axesMonitor.startMonitoringAxes(robot);
			JointImpedanceControlMode jointImp = new JointImpedanceControlMode(10.0, 5000.0, 5.0, 5000.0, 5.0, 5000.0, 10.0);
			jointImp.setDampingForAllJoints(0.1);
			// move first 5cm above attaching position
			motionCmd = tcp.move(ptp(startPosCalc).setJointVelocityRel(0.100).breakWhen(overload_14kg).setMode(jointImp));
			//motionCmd.await();
			//logger.info("Robert: lift mode is finished");

		// update start pose and fixate the robot at this pose
		currentPosFrame = robot.getCurrentCartesianPosition(tcp, World.Current.getRootFrame());
		startPosCalc = currentPosFrame.copyWithRedundancy();

		IFiredConditionInfo firedInfo = motionCmd.getFiredBreakConditionInfo();

		if (firedInfo != null) { 

			motionCmd.cancel();

			ICondition firedCond = firedInfo.getFiredCondition();
			if (firedCond.equals(overload_14kg)) {
				
				soft.parametrize(CartDOF.ROT).setStiffness(300.0);
				soft.parametrize(CartDOF.ALL).setDamping(0.9);

				try {
					motionCmd = tcp.move(positionHold(soft, 1, TimeUnit.MICROSECONDS));
				} catch (Exception e) {
					logger.info("Robert: error while trying to hold position of weight over 14kg");
					e.printStackTrace();
				}
				return false;
			}

		} else {		

			motionCmd = tcp.moveAsync(ptp(startPosCalc).setJointVelocityRel(0.250).setJointAccelerationRel(0.001));
			motionCmd.await();
			//ThreadUtil.milliSleep(100);
			
			if (this.limitReached) {
				logger.info("Robert: limit reached");
				axesMonitor.stopMointor();
				return false;

			}

			axesMonitor.stopMointor();

			linkage.getLoadData().setMass(0);
			
			double oneStepForce = calculateMeanForce();

			if (0 < oneStepForce) {

				// The mass of load where linkage is already added
				weight = Math.abs(oneStepForce / 9.82) - 0.3;

				// adding load to the robot
				if (weight >= 0) {
					// update the mass of the linkage to be weight plus the mass
					// of the linkage
					linkage.getLoadData().setMass(weight);
					logger.info("Robert: \n Attach position: " + attachPosFrame.getZ() + "\n Start position : "
							+ startPosCalc.getZ() + "\n End poisition: " + endPosCalc.getZ());
					logger.info("Robert: the weight is: " + (weight));
				} else {
					logger.info("Robert: Very small load attached to the Robert Linkage");
					return false;

				}				
				return true;

			} else
				return false;

		}
		return false;
	}

	/**
	 * This function calculate the mean force at a certain position during the
	 * lift Mode
	 * 
	 * @return Positive value if valid force has been calculated, otherwise zero
	 *         value
	 */
	
	private double calculateMeanForce() {

		ForceSensorData data = null;
		forceInZ = 0.0;
		int cnt = 0;

		for (int i = 1; i <= 3; i++) {
			data = robot.getExternalForceTorque(tcp, World.Current.getRootFrame());
			if (data.isForceValid(9)) {
				Vector force = data.getForce();
				forceInZ += force.getZ();
				cnt++;
			}

			ThreadUtil.milliSleep(50);
		}
		if (0 < cnt) {
			return Math.abs(forceInZ) / cnt;
		}
		return 0.0;
	}
	
	public Frame getStartFrame() {
		return startPosCalc;
	}

	public Frame getEndFrame() {
		return endPosCalc;
	}

	/*
	 * The weight of the leg plus the weight of the tool
	 */
	public double getWeight() {
		return weight;
	}

	public void resetData() {
		while (!motionCmd.isFinished()) {
			motionCmd.cancel();
			ThreadUtil.milliSleep(100);
		}
		startPosCalc = null;
		attachPosFrame = null;
		endPosCalc = null;
		forceInZ = 0.0;
		weight = 0.0;
		logger.info("Robert:\nstart position: " + startPosCalc + "\nAttach position: " + attachPosFrame + ",\nEnd position: "
				+ endPosCalc + "\nWeight: " + weight);
	}

}
