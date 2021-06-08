package shared;

import java.util.List;

import com.kuka.common.ThreadUtil;
import com.kuka.med.controllerModel.MedController;
import com.kuka.med.mastering.Mastering;
import com.kuka.med.robotState.FailedConditionForNormalUse;
import com.kuka.med.robotState.RobotStateAggregator;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.task.ITaskLogger;

/** This class is responsible for checking the robot system after power on */

public class SystemCheck {
	public enum Status {
		NOT_POSITION_REFERENCED, NOT_MASTERED, NEED_BrakeTest, NEED_GMS_REFERENCED, STATUS_OK
	}

	private boolean robotReadyForNormalUse;
	private boolean masteringReq;
	private boolean referencingReq;
	private boolean allAxesMastered;
	private boolean allAxesReferenced;
	// private Referencing referencing;
//	private ReferencingHelper referenceHelper;
	private RobotStateAggregator robotState;
	private MedController controller;
	private LBR robot;
	private ITaskLogger logger;

	// private LedColor ledColor;

	public SystemCheck(MedController _controller, LBR _robot, ITaskLogger _logger) {
		controller = _controller;
		robot = _robot;
		logger = _logger;
		robotState = new RobotStateAggregator(robot);
		robotState.isReadyForNormalUse();

		robotReadyForNormalUse = false;

	}

	public boolean systemStateHandle(LedColor ledColor, IApplicationData iApplicationData, Tool linkage) {
		logger.info("Robert: status" + robotState.isReadyForNormalUse());

	//	referenceHelper = new ReferencingHelper(robot, controller, logger);

		// Check of robot needs mastering
		if (isMasteringReq()) {

			// If robot needs mastering, perform mastering and subsequent
			// referencing
			allAxesMastered = performMastering(controller, robot, logger);

			if (allAxesMastered) {
				logger.info("Robert: all axes are successfully mastered, referencing is required");
			//	allAxesReferenced = referenceHelper.doPositionAndGMSReferencing();
				if (allAxesReferenced) {
					logger.info("Robert: all axes are successfully referenced");
				}
			}
		} else if (isReferencingReq()) {
			// If robot needs referencing, perform referencing
			//allAxesReferenced = referenceHelper.doPositionAndGMSReferencing();
			if (allAxesReferenced) {
				logger.info("Robert: all axes are successfully referenced");
				robotReadyForNormalUse = true;
			}
		}

	//	BrakeTest brakeTest = new BrakeTest(controller, robot);


		checkFailConditions();
		return robotReadyForNormalUse;
	}

	public boolean checkFailConditions() {

		ThreadUtil.milliSleep(100);
		allAxesMastered = true;
		allAxesReferenced = true;
		boolean returnedValue = false;

		List<FailedConditionForNormalUse> failList = robotState.getFailedConditionsForNormalUse();

		for (int i = 0; i < failList.size(); i++) {
			FailedConditionForNormalUse failedCondition = failList.get(i);
			int failNo = i + 1;
			logger.info("Robert: robot state returned with fail no. " + failNo + ": " + failedCondition.toString());
			switch (failedCondition) {
				case NOT_MASTERED :
					allAxesMastered = false;
					returnedValue = true;
					break;
				case NOT_GMS_REFERENCED :
					allAxesReferenced = false;
					returnedValue = true;
					break;
				case NOT_POSITION_REFERENCED :
					allAxesReferenced = false;
					returnedValue = true;
					break;
				case WRONG_BOOT_STATE :
					returnedValue = true;
					break;
				case CONNECTION_LOST :
					returnedValue = true;
					break;
				case WRONG_DEVICE_STATE :
					returnedValue = true;
					break;
				case NOT_READY_TO_MOVE :
					// the robot is stopped for safety reason or T-mode was
					// selected
					// and enabling switch not selected
					returnedValue = false;
					break;
				case WRONG_OPERATION_MODE :
					returnedValue = true;
					break;
			}
		}

		// logger.info("Robert: finished iterating fail list");

		if (!allAxesMastered) {
			logger.info("Robert: mastering and subsequent referencing is required");
			masteringReq = true;

		} else if (allAxesMastered && !allAxesReferenced) {
			logger.info("Robert: referencing is required");
			referencingReq = true;
		}
		return returnedValue;
	}

	public boolean performMastering(MedController controller, LBR robot, ITaskLogger logger) {
		allAxesMastered = true;
		boolean axisMasteredSuccessful = false;

		// logger.info("Robot is performing mastering");
		Mastering masterRobot = new Mastering(robot);

		for (int axis = 0; axis < robot.getJointCount(); axis++) {
			int axisNo = axis + 1;

			if (!masterRobot.isAxisMastered(axis)) {

				axisMasteredSuccessful = masterRobot.masterAxis(axis);
				if (!axisMasteredSuccessful) {
					// error handling ...
					logger.info("Robert: axis A" + axisNo + " not mastered");
				} else {
					logger.info("Robert: axis A" + axisNo + " successfully mastered");
				}
			} else {
				axisMasteredSuccessful = true;
			}
			allAxesMastered &= axisMasteredSuccessful;
		}

		return allAxesMastered;
	}

	public boolean isMasteringReq() {
		return masteringReq;
	}

	public boolean isReferencingReq() {
		return referencingReq;
	}

}
