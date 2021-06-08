/**
 * 
 */
package shared;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;

import backgroundTask.StateHelper;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.BodyIOGroup;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.ITaskLogger;

/**
 * @author Firas Amin
 * 
 */
public class Degree extends Thread {
	private LBR robot;

	private double init_A4;
	private double init_A6;
	private JointPosition position;
	private ObjectFrame recTcp;
	public boolean isEndPressed = false;
	private CartesianImpedanceControlMode softBounce;
	private ITaskLogger logger;

	private BodyIOGroup bodyIOGroup;

	public Degree(LBR _robot, IMotionContainer _softMc, CartesianImpedanceControlMode _softBounce,  JointPosition _position, ObjectFrame _recTCP,BodyIOGroup _bodyIOGroup , ITaskLogger _logger) {
		this.robot = _robot;
		this.position = _position;
		this.recTcp = _recTCP;
		this.softBounce = _softBounce;
		this.logger = _logger;
		this.bodyIOGroup = _bodyIOGroup;
		Training.softMc = _softMc;

	}

	private double[] getJointsDegrees() {

		init_A4 = Math.toDegrees(robot.getCurrentJointPosition().get(JointEnum.J4));
		init_A6 = Math.toDegrees(robot.getCurrentJointPosition().get(JointEnum.J6));

		return new double[]{init_A4, init_A6};
	}

	public void run() {

		if (Training.isTorqueExceeded) {

			while (Training.isTorqueExceeded && !StateHelper.isEstopPressed) {
				torqueDegreeMointoring();
			}

		} else {
			//logger.info("pause control: " + this.bodyIOGroup.getPauseControl() + ", End : " + this.isEndPressed + ", play/pause : " + bodyIOGroup.getPlayPauseButton()); 
			while (this.bodyIOGroup.getPauseControl() && !this.isEndPressed && !bodyIOGroup.getPlayPauseButton()/* && !this.softMc.isFinished()*/ ) {				

				torqueDegreeMointoring();
			}
				
				Training.isPlayOrEndPressed = true;
			}

			//this.logger.info("Value of boolean in Degree is set to true");
		}

	
	
	
	/**
	 * The function is used to monitor the axes degree
	 * Put the function in a while loop to continually monitor the axes
	 * @param position, 
	 */
	private void torqueDegreeMointoring() {

		double[] CurrentDegrees = getJointsDegrees();

		// A4 and A6 are mostly exposed to hitting the joint limits
		if (120 - Math.abs(CurrentDegrees[0]) < 5 || 120 - Math.abs(CurrentDegrees[1]) < 5) {
			while (!Training.softMc.isFinished()) {
				Training.softMc.cancel();
				// ThreadUtil.milliSleep(50);
			}
			robot.setESMState("6");
			ThreadUtil.milliSleep(100);
			robot.setESMState("3");
			//PathDataRecorder.isPlayOrEnd = false;
			//PathDataRecorder.setPlayOrEnd(false);
			Training.isPlayOrEndPressed = false; 
			try {
				recTcp.move(ptp(position).setJointVelocityRel(0.250));
				robot.setESMState("5");
				//TODO soft bounce does not work
				Training.softMc = robot.moveAsync(positionHold(softBounce, -1, TimeUnit.MILLISECONDS));
			} catch (Exception e) {
				 logger.info("Robert: can not reposition the robot to the original place");
			}
			ThreadUtil.milliSleep(1000);
			
			
		}

		ThreadUtil.milliSleep(30);
	}

	void terminate(){
		
	}
}
