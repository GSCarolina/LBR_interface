package shared;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;

public abstract class AxesMonitor {

	private LBR robot;
	private boolean isMonitoring;

	/**
	 * asynchronous method that monitor the axes of the robot during impedance controller with position hold
	 */
	public void startMonitoringAxes(LBR lbr) {
		this.robot = lbr;
		new Thread(	new Runnable() {
			public void run() {
				startMointor();
			}
		}).start();
	}

	private void startMointor(){
		this.isMonitoring = true;
		while (this.isMonitoring) {
			double[] CurrentDegrees = getJointsDegrees();

			// A4 and A6 are mostly exposed to hitting the joint limits
			if (
					170 - CurrentDegrees[0] < 5 
					|| 120 - CurrentDegrees[1] < 5
					|| 170 - CurrentDegrees[2] < 5
					|| 120 - CurrentDegrees[3] < 5
					|| 170 - CurrentDegrees[4] < 5
					|| 120 - CurrentDegrees[5] < 5
					|| 175 - CurrentDegrees[6] < 5 ) {

				hasReachedAxesLimit(true);
				robot.setESMState("5");
				ThreadUtil.milliSleep(100);
				robot.setESMState("2");

				return;
			}
			ThreadUtil.milliSleep(50);
		}
	}
	/**
	 * 
	 * @return stop monitoring the axes of the robot and return false
	 */
	public boolean stopMointor(){
		this.isMonitoring = false;
		return isMonitoring;
	}
	
	private double[] getJointsDegrees() {
		double init_A1 = Math.abs(Math.toDegrees(robot
				.getCurrentJointPosition().get(JointEnum.J1)));
		double init_A2 = Math.abs(Math.toDegrees(robot
				.getCurrentJointPosition().get(JointEnum.J2)));
		double init_A3 = Math.abs(Math.toDegrees(robot
				.getCurrentJointPosition().get(JointEnum.J3)));
		double init_A4 = Math.abs(Math.toDegrees(robot
				.getCurrentJointPosition().get(JointEnum.J4)));
		double init_A5 = Math.abs(Math.toDegrees(robot
				.getCurrentJointPosition().get(JointEnum.J5)));
		double init_A6 = Math.abs(Math.toDegrees(robot
				.getCurrentJointPosition().get(JointEnum.J6)));
		double init_A7 = Math.abs(Math.toDegrees(robot
				.getCurrentJointPosition().get(JointEnum.J7)));

		return new double[] { init_A1, init_A2, init_A3, init_A4, init_A5,
				init_A6, init_A7 };
	}

	public abstract void hasReachedAxesLimit(boolean isLimitReached);
}
