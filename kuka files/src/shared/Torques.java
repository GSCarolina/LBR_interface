package shared;

import java.util.ArrayList;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.task.ITaskLogger;

public class Torques extends Thread {

	private LBR robot;
	private ITaskLogger logger;
	private ArrayList<double[]> torquesDataArray;
	private double actTJ7;
	private double actTJ6;
	private double actTJ5;
	private double actTJ4;
	private double actTJ3;
	private double actTJ2;
	private double actTJ1;
	private double[] maxTorquesValues;
	public boolean pause;
	private boolean recTorques;
	private JointTorqueCondition jt1;
	private JointTorqueCondition jt2;
	private JointTorqueCondition jt3;
	private JointTorqueCondition jt4;
	private JointTorqueCondition jt5;
	private JointTorqueCondition jt6;
	private JointTorqueCondition jt7;

	public void setPause(boolean pause) {
		this.pause = pause;
	}

	public void setRecTorques(boolean recTorques) {
		this.recTorques = recTorques; 
	}

	public Torques(LBR lbr, ITaskLogger _logger, IApplicationData appData) {
		// trackFrames = motionFrames;
		torquesDataArray = new ArrayList<double[]>();
		logger = _logger;
		robot = lbr;

		maxTorquesValues = new double[7];
	}

	@Override
	public void run() {

		recordTorques();
	}

	private void recordTorques() {

		do {
			torquesDataArray.add(robot.getExternalTorque().getTorqueValues());
			ThreadUtil.milliSleep(50);

			// wait if playing is paused
			while (this.pause) {
				ThreadUtil.milliSleep(50);
			}
		} while (this.recTorques);

	}

	public double[] calcMaxTorque(ArrayList<double[]> torqueArray) {

		double maxValue1 = 0.0;
		double maxValue2 = 0.0;
		double maxValue3 = 0.0;
		double maxValue4 = 0.0;
		double maxValue5 = 0.0;
		double maxValue6 = 0.0;
		double maxValue7 = 0.0;

		for (double[] torque : torqueArray) {
			double currentValue1 = Math.abs(torque[0]);
			double currentValue2 = Math.abs(torque[1]);
			double currentValue3 = Math.abs(torque[2]);
			double currentValue4 = Math.abs(torque[3]);
			double currentValue5 = Math.abs(torque[4]);
			double currentValue6 = Math.abs(torque[5]);
			double currentValue7 = Math.abs(torque[6]);

			maxValue1 = Math.max(currentValue1, maxValue1);
			maxValue2 = Math.max(currentValue2, maxValue2);
			maxValue3 = Math.max(currentValue3, maxValue3);
			maxValue4 = Math.max(currentValue4, maxValue4);
			maxValue5 = Math.max(currentValue5, maxValue5);
			maxValue6 = Math.max(currentValue6, maxValue6);
			maxValue7 = Math.max(currentValue7, maxValue7);

			maxTorquesValues[0] = maxValue1;
			maxTorquesValues[1] = maxValue2;
			maxTorquesValues[2] = maxValue3;
			maxTorquesValues[3] = maxValue4;
			maxTorquesValues[4] = maxValue5;
			maxTorquesValues[5] = maxValue6;
			maxTorquesValues[6] = maxValue7;

//			logger.info("torques:\n" + "[J1 : " + currentValue1 + "], [J2 : " + currentValue2 + "], [J3 : " + currentValue3 + "], [J4 : "
//					+ currentValue4 + "], [J5 : " + currentValue5 + "], [J6 : " + currentValue6 + "], [J7 : " + currentValue7 + "]\n");
		}

		return maxTorquesValues;
	}

	/**
	 * prepare the bounce condition 
	 * @param torqueArray
	 * @return the condition for bounce mode
	 */
	public ICondition prepareBounceCondition(ArrayList<TorqueSensorData> torqueArray){
		double[] max = maxtTorqueDataValues(torqueArray);
		logger.info("ROBERT: torques recordings: " + torqueArray.toString());
		return prepareConditon(max);
	}
	
	public double[] maxtTorqueDataValues (ArrayList<TorqueSensorData> torqueArray){
		double maxValue1 = 0.0;
		double maxValue2 = 0.0;
		double maxValue3 = 0.0;
		double maxValue4 = 0.0;
		double maxValue5 = 0.0;
		double maxValue6 = 0.0;
		double maxValue7 = 0.0;
		
		for (TorqueSensorData torque : torqueArray) {
			double currentValue1 = Math.abs(torque.getSingleTorqueValue(JointEnum.J1));
			double currentValue2 = Math.abs(torque.getSingleTorqueValue(JointEnum.J2));
			double currentValue3 = Math.abs(torque.getSingleTorqueValue(JointEnum.J3));
			double currentValue4 = Math.abs(torque.getSingleTorqueValue(JointEnum.J4));
			double currentValue5 = Math.abs(torque.getSingleTorqueValue(JointEnum.J5));
			double currentValue6 = Math.abs(torque.getSingleTorqueValue(JointEnum.J6));
			double currentValue7 = Math.abs(torque.getSingleTorqueValue(JointEnum.J7));

			maxValue1 = Math.max(currentValue1, maxValue1);
			maxValue2 = Math.max(currentValue2, maxValue2);
			maxValue3 = Math.max(currentValue3, maxValue3);
			maxValue4 = Math.max(currentValue4, maxValue4);
			maxValue5 = Math.max(currentValue5, maxValue5);
			maxValue6 = Math.max(currentValue6, maxValue6);
			maxValue7 = Math.max(currentValue7, maxValue7);

			maxTorquesValues[0] = maxValue1;
			maxTorquesValues[1] = maxValue2;
			maxTorquesValues[2] = maxValue3;
			maxTorquesValues[3] = maxValue4;
			maxTorquesValues[4] = maxValue5;
			maxTorquesValues[5] = maxValue6;
			maxTorquesValues[6] = maxValue7;
		}
		return maxTorquesValues;		
	}
	public ICondition prepareConditon(double[] maxValue) { 

		//double[] maxValue = calcMaxTorque();
		// logger.info(" Max torque value : " + maxValue);

		
		//double sensCLS = roboticApiAppData.getProcessData("sensCLS").getValue(); //  margin 32N
		double sensCLS = 32;
		
		actTJ1 = maxValue[0];
		actTJ2 = maxValue[1];
		actTJ3 = maxValue[2];
		actTJ4 = maxValue[3];
		actTJ5 = maxValue[4];
		actTJ6 = maxValue[5];
		actTJ7 = maxValue[6];

		logger.info("ROBERT: the max measured torque values: A1: " + actTJ1 + ", A2: " + actTJ2 + ", A3: " + actTJ3 + ", A4: " + actTJ4 + ", A5: " + actTJ5 + ", A6: " + actTJ6 + ", A7: " + actTJ7);
		// torque ranges
		
		logger.info("ROBERT: the upper safety limits are: A1:300, A2:300, A3:170, A4 170, A5:100, A6:37, A7:37");
		if (actTJ1 + sensCLS > 320) {
			int limit = torquelimit(JointEnum.J1) - 20;
			logger.info("ROBERT: the upper limit exceeded at A1 the threshold set to 300");
			jt1 = new JointTorqueCondition(JointEnum.J1, -limit, limit);
		} else {
			jt1 = new JointTorqueCondition(JointEnum.J1, -sensCLS - actTJ1, sensCLS + actTJ1);
		}

		if (actTJ2 + sensCLS > 320) {
			int limit = torquelimit(JointEnum.J2) - 20;
			logger.info("ROBERT: the upper limit exceeded at A2 the threshold set to 300");
			jt2 = new JointTorqueCondition(JointEnum.J2, -limit, limit);
		} else {
			jt2 = new JointTorqueCondition(JointEnum.J2, -sensCLS - actTJ2, sensCLS + actTJ2);
		}

		if (actTJ3 + sensCLS > 176) {
			int limit = torquelimit(JointEnum.J3) - 10;
			logger.info("ROBERT: the upper limit exceeded at A3 the threshold set to 170");
			jt3 = new JointTorqueCondition(JointEnum.J3, -limit, limit);
		} else {
			jt3 = new JointTorqueCondition(JointEnum.J3, -sensCLS - actTJ3, sensCLS + actTJ3);
		}

		if (actTJ4 + sensCLS > 176) {
			int limit = torquelimit(JointEnum.J4) - 10;
			logger.info("ROBERT: the upper limit exceeded at A4 the threshold set to 170");
			jt4 = new JointTorqueCondition(JointEnum.J4, -limit, limit);
		} else {
			jt4 = new JointTorqueCondition(JointEnum.J4, -sensCLS - actTJ4, sensCLS + actTJ4);
		}

		if (actTJ5 + sensCLS > 110) {
			int limit = torquelimit(JointEnum.J5) - 5;
			jt5 = new JointTorqueCondition(JointEnum.J5, -limit, limit);
			logger.info("ROBERT: the upper limit exceeded at A5 the threshold set to 100");
		} else {
			jt5 = new JointTorqueCondition(JointEnum.J5, -sensCLS - actTJ5, sensCLS + actTJ5);
		}

		if (actTJ6 + sensCLS > 40) { 
			int limit = torquelimit(JointEnum.J6) - 2;
			logger.info("ROBERT: the upper limit exceeded at A6 the threshold set to 37");
			jt6 = new JointTorqueCondition(JointEnum.J6, -limit, limit);
		} else {
			jt6 = new JointTorqueCondition(JointEnum.J6, -sensCLS - actTJ6, sensCLS + actTJ6);
		}

		if (actTJ7 + sensCLS > 40) {
			int limit = torquelimit(JointEnum.J7) - 2;
			logger.info("ROBERT: the upper limit exceeded at A7the threshold set to 37");
			jt7 = new JointTorqueCondition(JointEnum.J7, -limit, limit);
		} else {
			jt7 = new JointTorqueCondition(JointEnum.J7, -sensCLS - actTJ7, sensCLS + actTJ7);
		}

		
//		logger.info("Robert: thresholds are:\n" + "joint 1: " + (actTJ1 + sensCLS) + " Nm\n" + "joint 2: " + (actTJ2 + sensCLS) + " Nm\n"
//				+ "joint 3: " + (actTJ3 + sensCLS) + " Nm\n" + "joint 4: " + (actTJ4 + sensCLS) + " Nm\n" + "joint 3: " + (actTJ5 + sensCLS)
//				+ " Nm\n" + "joint 5: " + (actTJ5 + sensCLS) + " Nm\n" + "joint 6: " + (actTJ6 + sensCLS) + " Nm\n" + "joint 7: "
//				+ (actTJ7 + sensCLS) + " Nm\n");

		return jt1.or(jt2, jt3, jt4, jt5, jt6, jt7);
	}

	public ICondition getStopTorqueCondition(){
		return jt1 = new JointTorqueCondition(JointEnum.J1, -30 - actTJ1, 20 + actTJ1); 
		
	}
	private int torquelimit(JointEnum joint) {

		int[] rangeValue = new int[]{320, 320, 176, 176, 110, 40, 40};
//		logger.info("Robert: the torque ranges : " + "joint 1: ±"+rangeValue[0] + ", joint 2: ±"+rangeValue[1] + ", joint 3: ±"+rangeValue[2]+ ", joint 4: ±"+rangeValue[3]+ ", joint 5: ±"+rangeValue[4]+ ", joint 6: ±"+rangeValue[5]+ ", joint 7: ±"+rangeValue[6]);
		switch (joint) {
			case J1 :
				return rangeValue[0];
			case J2 :
				return rangeValue[1];
			case J3 :
				return rangeValue[2];
			case J4 :
				return rangeValue[3];
			case J5 :
				return rangeValue[4];
			case J6 :
				return rangeValue[5];
			case J7 :
				return rangeValue[6];
		}
		return 0;

	}

}
