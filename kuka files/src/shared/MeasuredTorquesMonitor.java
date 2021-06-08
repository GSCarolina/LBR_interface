package shared;

import guidedMode.PathDataRecorder;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.task.ITaskLogger;

public class MeasuredTorquesMonitor implements Runnable {

	private LBR robot;
	private double j6;
	private double j7;
	private ITaskLogger logger;
	public MeasuredTorquesMonitor(LBR _robot, ITaskLogger _logger) {
		robot = _robot;
		logger = _logger;
	}

	public void run() {

		while (!PathDataRecorder.motionContainer.isFinished()) {
		j6 =	robot.getMeasuredTorque().getSingleTorqueValue(JointEnum.J6);
		j7 = robot.getMeasuredTorque().getSingleTorqueValue(JointEnum.J7);
			if (j6 > 20.0 || j7 > 20.0) {
				PathDataRecorder.isTorqueExceeded = true;
				while (!PathDataRecorder.motionContainer.isFinished()) {
					PathDataRecorder.motionContainer.cancel();
					ThreadUtil.milliSleep(100);
				} 
				 logger.info("Robert: high torque value measured at either: joint 6" + j6 + "joint 7" + j7);
				break;
			}
			ThreadUtil.milliSleep(100);
		}
 
	}

}
