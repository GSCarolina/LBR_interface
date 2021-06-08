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

public class ActivityRecorder {

	private LBR robot;
	private Tool linkage;
	private ITaskLogger logger;
	private boolean isFinished;
	private HashMap<Integer, TorqueSensorData> register;
	private ArrayList<Frame> trajectory;
	protected ArrayList<Double> weights;
	private boolean suspended;
	private ArrayList<TorqueSensorData> registeredTorques;
	private double activeDistance = 0.0;
	private double weightBuffer = 5;
	public ArrayList<Double[]> preparedBounceThresholds;


	public ActivityRecorder(ArrayList<Frame> _trajectory, ArrayList<TorqueSensorData> _registeredTorques,
			ArrayList<Double> _weights, LBR _lbr, Tool _linkage, ITaskLogger _logger, BodyIOGroup _bodyIOGroup) {
		this.robot = _lbr;
		this.linkage = _linkage;
		this.logger = _logger;
		this.trajectory = _trajectory;
		this.registeredTorques = _registeredTorques;
		this.weights = _weights;


		// setCurrentWeight();
		// ThreadUtil.milliSleep(100);
		
		this.preparedBounceThresholds = prepareThresholdCondition(this.registeredTorques);
		control();
		//bounceControl();
	}


	/**
	 * register torques and a new trajectory frames for each torque value
	 */
	private void control() {
		// monitor axes while moving up
		try {
			new Thread(new Runnable() {

				private int lastFrameIndex = 0;

				public void run() {

					for (int i = 0; i < trajectory.size(); i++) {

						linkage.getLoadData().setMass(weights.get(i));
						//logger.info("ROBERT: setting the weight of the leg: " + weights.get(i));
						while (trajectory.get(i).distanceTo(robot.getCurrentCartesianPosition(linkage.getFrame("/RecTCP"))) >= 8) {
							
							if (isFinished) {
								return;
							}

							ThreadUtil.milliSleep(10);
						}
							/**
							 * set the weight that has been recorded during 1st
							 * route at this position
							 */

							TorqueSensorData currentTorques = robot.getExternalTorque();
							//ForceSensorData forces = robot.getExternalForceTorque(linkage.getFrame("/RecTCP"));
							
							/**
							 * compare current torques with the recorded at 1st
							 * route
							 */
							if (checkActivity(currentTorques, registeredTorques.get(i))) {
								// activePos.add(trajectory.get(i));
								
								//logger.info("Activity, current: " + currentTorques + " registered : " + registeredTorques.get(i));
								/**
								 * calculate the distance only between (i) and
								 * (i+1) of the trajectory
								 */
								if (i == lastFrameIndex + 1) {
									
									activeDistance = activeDistance
											+ trajectory.get(i).distanceTo(trajectory.get(lastFrameIndex));
									
									logger.info("Activity");
									//logger.info("Active distance: " + trajectory.get(i).distanceTo(trajectory.get(lastFrameIndex)) / 10);
									 
								}
								lastFrameIndex = i;

							}
							

							if (isFinished) {
								return;
							}

							synchronized (this) {
								while (suspended) {
									ThreadUtil.milliSleep(100);

								}
							}



						// logger.info("found a frame");
					}
					
					// logger.info("The number of frames founded : " + cnt);
				}

				/**
				 * compare the torques of the current measurements with the
				 * recorded at the 1st route
				 * 
				 * @param currentTorques
				 * @param torqueSensorData
				 * @return Boolean
				 */
				private boolean checkActivity(TorqueSensorData currentTorques, TorqueSensorData torqueSensorData) {
						if ((Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J1)) > Math.abs(torqueSensorData
							.getSingleTorqueValue(JointEnum.J1)) + weightBuffer) 
							|| (Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J2)) > Math.abs(torqueSensorData
									.getSingleTorqueValue(JointEnum.J2))+ weightBuffer) 
							|| (Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J3)) > Math.abs(torqueSensorData
									.getSingleTorqueValue(JointEnum.J3))+ weightBuffer) 
							|| (Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J4)) > Math.abs(torqueSensorData
									.getSingleTorqueValue(JointEnum.J4))+ weightBuffer) 
							|| (Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J5)) > Math.abs(torqueSensorData
									.getSingleTorqueValue(JointEnum.J5))+ weightBuffer)
							|| (Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J6)) > Math.abs(torqueSensorData
									.getSingleTorqueValue(JointEnum.J6))+ weightBuffer) 
							|| (Math.abs(currentTorques.getSingleTorqueValue(JointEnum.J7)) > Math.abs(torqueSensorData
									.getSingleTorqueValue(JointEnum.J7))+ weightBuffer)) {
						return true; 
					}
					return false;
				}
				
				
			}).start();

		} catch (Exception e) {
			logger.info("Robert: thread is intruppted");
			e.printStackTrace();
		}
	}

	/**
	 * get a new trajectory path during the 1st Route which has the same length
	 * as the weights array and torques array
	 * 
	 * @return {@link ArrayList} of frames
	 */
	public ArrayList<Frame> getPathFrames() {
		return trajectory;
	}

	/**
	 * get the torque values
	 * 
	 * @return
	 */
	public HashMap<Integer, TorqueSensorData> getRegisteredTorques() {
		return register;
	}

	public ArrayList<Double> getWeights() {
		return weights;
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

	/**
	 * get the activity during the current route
	 * 
	 * @return {@link Double} : the activity in millimeter of the current route
	 */
	public double getActiveDistance() {
		logger.info("ROBERT: Total Active Distance : " + activeDistance);
		return activeDistance;
	}
	
private ArrayList<Double[]> prepareThresholdCondition(ArrayList<TorqueSensorData> _torques) {
		
		ArrayList<Double[]> adjustedTorques = new ArrayList<Double[]>();
		

		for (TorqueSensorData torque : _torques) {
			Double[] torqueValue = new Double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			torqueValue[0] = 1.2 * Math.abs(torque.getSingleTorqueValue(JointEnum.J1)) + 50;			
			if (torqueValue[0] > 300.0) {
				torqueValue[0] = 300.0;
			}			
			torqueValue[1] = 1.2 * Math.abs(torque.getSingleTorqueValue(JointEnum.J2)) + 50;
			if (torqueValue[1] > 300.0) {
				torqueValue[1] = 300.0;
			}
			torqueValue[2] = 1.2 * Math.abs(torque.getSingleTorqueValue(JointEnum.J3)) + 40;
			if (torqueValue[2] > 170.0) {
				torqueValue[2] = 170.0;
			}
			torqueValue[3] = 1.2 * Math.abs(torque.getSingleTorqueValue(JointEnum.J4)) + 40;
			if (torqueValue[3] > 170.0) {
				torqueValue[3] = 170.0;
			}
			torqueValue[4] = 1.1 * Math.abs(torque.getSingleTorqueValue(JointEnum.J5)) + 25;
			if (torqueValue[4] > 100.0) {
				torqueValue[4] = 100.0;
			}
			torqueValue[5] = 1.1 * Math.abs(torque.getSingleTorqueValue(JointEnum.J6)) + 15;
			if (torqueValue[5] > 37.0) {
				torqueValue[5] = 37.0;
			}
			torqueValue[6] = 1.1 * Math.abs(torque.getSingleTorqueValue(JointEnum.J7)) + 15;
			if (torqueValue[6] > 37.0) {
				torqueValue[6] = 37.0;
			}
			adjustedTorques.add(torqueValue);
		}
		
		return adjustedTorques;
	}
}
