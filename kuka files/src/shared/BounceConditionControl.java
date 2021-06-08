package shared;

import java.util.ArrayList;
import java.util.HashMap;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.BodyIOGroup;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.task.ITaskLogger;

public class BounceConditionControl {

	private LBR robot;
	private Tool linkage;
	private ITaskLogger logger;
	private boolean isFinished;
	private HashMap<Integer, TorqueSensorData> register;
	private ArrayList<Frame> trajectory;
	protected ArrayList<Double> weights;
	private boolean suspended;
	private ArrayList<TorqueSensorData> registeredTorques;

	public ArrayList<Double[]> preparedBounceThresholds;
	private BodyIOGroup bodyIOGroup; 

	public BounceConditionControl(ArrayList<Frame> _trajectory, ArrayList<TorqueSensorData> _registeredTorques,
			ArrayList<Double> _weights, LBR _lbr, Tool _linkage, ITaskLogger _logger, BodyIOGroup _bodyIOGroup) {
		this.robot = _lbr;
		this.linkage = _linkage;
		this.logger = _logger;
		this.trajectory = _trajectory;
		this.registeredTorques = _registeredTorques;
		this.weights = _weights;
		this.bodyIOGroup = _bodyIOGroup;

		// setCurrentWeight();
		// ThreadUtil.milliSleep(100);
		
		this.preparedBounceThresholds = prepareThresholdCondition(this.registeredTorques);

		bounceControl();
	}

	private void bounceControl() {
		
		new Thread(new Runnable() {

			
			public void run() {
				for (int i = 0; i < trajectory.size(); i++) {
					//logger.info("ROBERT: setting the weight of the leg: " + weights.get(i));
					while (trajectory.get(i).distanceTo(robot.getCurrentCartesianPosition(linkage.getFrame("/RecTCP"))) >= 3) {
						TorqueSensorData currentTorques = robot.getExternalTorque();						
							if (controlBounceCondition(currentTorques, preparedBounceThresholds.get(i))) {
								bodyIOGroup.setPauseControl(true); 
								//logger.info("Inside");

								ThreadUtil.milliSleep(500);
							
						}

								synchronized (this) {
									while (suspended) {
										ThreadUtil.milliSleep(100);

									}
								}
							

						
						if (isFinished) {
							return;
						}
						ThreadUtil.milliSleep(10);
					}
				}
			}
			private boolean controlBounceCondition(TorqueSensorData currentTorques, Double[] threshold){
				if ((Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J1)) > threshold[0]) 
						|| (Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J2)) > threshold[1]) 
						|| (Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J3)) > threshold[2]) 
						|| (Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J4)) > threshold[3]) 
						|| (Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J5)) > threshold[4])
						|| (Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J6)) > threshold[5]) 
						|| (Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J7)) > threshold[6])) {
					logger.info("ROBERT: torque thresholds exceeded- current:" + currentTorques + "\n measured: " + threshold[0] + ";" +threshold[1] + ";" + threshold[2] + ";" + threshold[3] + ";" + threshold[4] + ";" + threshold[5] + ";" + threshold[6]);
					return true; 
				}
				return false;					
			}
		}).start();
		
	}


	/**
	 * get the torque values
	 * 
	 * @return
	 */
	public HashMap<Integer, TorqueSensorData> getRegisteredTorques() {
		return register;
	}

	/**
	 * stop calculating the weights and the torques
	 */
	public synchronized void stop() {
		isFinished = true;
		suspended = false;
		// notifyAll();
	}

	public synchronized void suspend() {

		suspended = true;
	}

	public synchronized void resume() {

		suspended = false;
		// notifyAll();
	}

	/**
	 * check if the threads are running
	 * 
	 * @return Boolean
	 */
	public synchronized boolean isSuspended() {

		return suspended;

	}

	
private ArrayList<Double[]> prepareThresholdCondition(ArrayList<TorqueSensorData> _torques) {
		
		ArrayList<Double[]> adjustedTorques = new ArrayList<Double[]>();
		

		for (TorqueSensorData torque : _torques) {
			Double[] torqueValue = new Double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			torqueValue[0] = 1.4 * Math.abs(torque.getSingleTorqueValue(JointEnum.J1)) + 70;			
			if (torqueValue[0] > 300.0) {
				torqueValue[0] = 300.0;
			}			
			torqueValue[1] = 1.4 * Math.abs(torque.getSingleTorqueValue(JointEnum.J2)) + 50;
			if (torqueValue[1] > 300.0) {
				torqueValue[1] = 300.0;
			}
			torqueValue[2] = 1.3 * Math.abs(torque.getSingleTorqueValue(JointEnum.J3)) + 40;
			if (torqueValue[2] > 170.0) {
				torqueValue[2] = 170.0;
			}
			torqueValue[3] = 1.3 * Math.abs(torque.getSingleTorqueValue(JointEnum.J4)) + 30;
			if (torqueValue[3] > 170.0) {
				torqueValue[3] = 170.0;
			}
			torqueValue[4] = 1.3 * Math.abs(torque.getSingleTorqueValue(JointEnum.J5)) + 20;
			if (torqueValue[4] > 100.0) {
				torqueValue[4] = 100.0;
			}
			torqueValue[5] = 1.3 * Math.abs(torque.getSingleTorqueValue(JointEnum.J6)) + 20;
			if (torqueValue[5] > 37.0) {
				torqueValue[5] = 37.0;
			}
			torqueValue[6] = 1.3 * Math.abs(torque.getSingleTorqueValue(JointEnum.J7)) + 20;
			if (torqueValue[6] > 37.0) {
				torqueValue[6] = 37.0;
			}
			adjustedTorques.add(torqueValue);
		}
		
		return adjustedTorques;
	}
}
