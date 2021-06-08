package util;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

import com.kuka.med.controllerModel.MedController;
import com.kuka.roboticsAPI.controllerModel.sunrise.ISunriseRequestService;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.SSR;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.SSRFactory;
import com.kuka.roboticsAPI.controllerModel.sunrise.connectionLib.Message;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.OperationMode;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.PTPHome;
import com.kuka.task.ITaskLogger;

/**
 * This helper can be used for LBR iiwa/Med Position and GMS Referencing. The
 * safety needs exactly 10 measurements to perform a successful GMS Referencing.
 * The time between two measurements must be less than 15 seconds.
 */
public class ReferencingHelper {

	private LBR _lbrMed;
	private ITaskLogger _logger;
	private ISunriseRequestService _requestService;
	private static final double SIDE_OFFSET = Math.toRadians(5);
	private static double VELOCITY = 0.2;
	// safety command for GMS referencing
	private static final int GMS_REFERENCING_COMMAND = 2;
	private static final int COMMAND_SUCCESSFUL = 1;
	private int _positionCounter = 0;

	public ReferencingHelper(LBR lbrMed, MedController medController, ITaskLogger logger) {
		_lbrMed = lbrMed;
		_logger = logger;

		_requestService = medController.getRequestService();
	}

	public boolean doPositionAndGMSReferencing() {
		// We can move faster, if operation mode is T1
		if (OperationMode.T1 == _lbrMed.getOperationMode()) {
			VELOCITY = 0.4;
		}

		_logger.info("Performing position and GMS referencing with 5 positions");
		_logger.info("Moving to home position first"); 
		PTPHome positions = ptpHome().setJointVelocityRel(VELOCITY);
		_lbrMed.move(positions); 

		// In this example 5 positions are defined, though each one will be
		// reached from negative and from positive axis
		// direction resulting 10 measurements. The safety needs exactly 10
		// measurements to perform the referencing.
		boolean result = true;
		result &= performMotion(new JointPosition(Math.toRadians(10.0), Math.toRadians(-10.0), Math.toRadians(11.0), Math.toRadians(-11.0),
				Math.toRadians(11.0), Math.toRadians(-15.0), Math.toRadians(15.0)));

		result &= performMotion(new JointPosition(Math.toRadians(-10.0), Math.toRadians(10.0), Math.toRadians(-11.0), Math.toRadians(11.0),
				Math.toRadians(-11.0), Math.toRadians(15.0), Math.toRadians(-15.0)));

		result &= performMotion(new JointPosition(Math.toRadians(10.0), Math.toRadians(-10.0), Math.toRadians(11.0), Math.toRadians(-11.0),
				Math.toRadians(11.0), Math.toRadians(-15.0), Math.toRadians(15.0)));

		result &= performMotion(new JointPosition(Math.toRadians(-10.0), Math.toRadians(10.0), Math.toRadians(-11.0), Math.toRadians(11.0),
				Math.toRadians(-11.0), Math.toRadians(15.0), Math.toRadians(-15.0)));

		result &= performMotion(new JointPosition(Math.toRadians(10.0), Math.toRadians(-10.0), Math.toRadians(11.0), Math.toRadians(-11.0),
				Math.toRadians(11.0), Math.toRadians(-15.0), Math.toRadians(15.0)));

		// Move to home position at the end
		_logger.info("Moving to home position");
		_lbrMed.move(ptpHome().setJointVelocityRel(VELOCITY));

		return result;
	}

	private boolean performMotion(JointPosition position) {
		_logger.info("Moving to position #" + (++_positionCounter));

		PTP mainMotion = new PTP(position).setJointVelocityRel(VELOCITY);
		_lbrMed.move(mainMotion);

		// Moving to current position from negative direction
		JointPosition position1 = new JointPosition(_lbrMed.getJointCount());
		for (int i = 0; i < _lbrMed.getJointCount(); ++i) {
			position1.set(i, position.get(i) - SIDE_OFFSET);
		}
		PTP motion1 = new PTP(position1).setJointVelocityRel(VELOCITY);
		_lbrMed.move(motion1);
		_lbrMed.move(mainMotion);

		// Wait a little to reduce robot vibration after stop.
		// ThreadUtil.milliSleep(2500);

		// Send the command to safety to trigger the measurement
		if (!sendSafetyCommand()) {
			return false;
		}

		// Moving to current position from positive direction
		JointPosition position2 = new JointPosition(_lbrMed.getJointCount());
		for (int i = 0; i < _lbrMed.getJointCount(); ++i) {
			position2.set(i, position.get(i) + SIDE_OFFSET);
		}
		PTP motion2 = new PTP(position2).setJointVelocityRel(VELOCITY);
		_lbrMed.move(motion2);
		_lbrMed.move(mainMotion);

		// Wait a little to reduce robot vibration after stop
		// ThreadUtil.milliSleep(2500);

		// Send the command to safety to trigger the measurement
		return sendSafetyCommand();
	}

	private boolean sendSafetyCommand() {
		SSR ssr = SSRFactory.createSafetyCommandSSR(GMS_REFERENCING_COMMAND);
		Message response = _requestService.sendSynchronousSSR(ssr);
		int result = response.getParamInt(0);
		if (COMMAND_SUCCESSFUL != result) {
			_logger.warn("Command did not execute successfully, response = " + result);
			return false;
		}
		return true;
	}
}
