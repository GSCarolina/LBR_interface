package shared;

import java.io.IOException;
import java.util.List;

import main.Robert_2_3;

import shared.LedColor.Color;
import util.UserCmd;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.LinkageIOGroup;
import com.kuka.med.controllerModel.MedController;
import com.kuka.med.mastering.Mastering;
import com.kuka.med.robotState.FailedConditionForNormalUse;
import com.kuka.med.robotState.RobotStateAggregator;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseExecutionService;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.task.ITaskLogger;

public class MotionMonitor implements Runnable {
	private boolean running;
	private SunriseExecutionService execService;
	private TCPConnection tcpConnection;

	private LBR lbr;

	private Mastering mastering;
	private ITaskLogger logger;
	private LedColor ledColor;
	private LinkageIOGroup linkageIOGroup;
	private boolean isSystemCheck;
	private ExcutionController excutionController;

	public MotionMonitor(ITaskLogger _logger, SunriseExecutionService _execService, TCPConnection _tcpConnection, LBR _lbrMed,
			MedController _medController, LinkageIOGroup _linkageIOGroup, LedColor _ledColor) {
		running = true;
		this.execService = _execService;
		this.tcpConnection = _tcpConnection;
		lbr = _lbrMed;
		mastering = new Mastering(_lbrMed);
		logger = _logger;
		this.ledColor = _ledColor;
		this.linkageIOGroup = _linkageIOGroup;
		isSystemCheck = true;
	}

	@Override
	public void run() { 

		while (running) {

			controlAxesValidity();
			checkMasteringAndRefState();
		
			try {
				Thread.sleep(2000); 
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	private void controlAxesValidity() {


		for (int i = 0; i < lbr.getCurrentJointPosition().get().length; i++) {

			if (mastering.isAxisMastered(i)) {
				if (Math.abs(lbr.getCurrentJointPosition().get(i)) >= Math.abs(lbr.getJointLimits().getMaxJointPosition().get(i))) {
					mastering.invalidateMastering(i);
					try {
						tcpConnection.sendMessage(UserCmd.HARD_LIMIT.toString());
					} catch (IOException e) {
						e.printStackTrace();
					}
					logger.info("ROBERT: axis no: " + i + "has been unmastered");
				}
			}

		}
	}

	private void checkMasteringAndRefState() {
		// check if robot is ready for running the main application steps
		RobotStateAggregator stateAggregator = new RobotStateAggregator(this.lbr);
		// go through failedConditions and react accordingly
		List<FailedConditionForNormalUse> failedConditions = stateAggregator.getFailedConditionsForNormalUse();

		if (failedConditions.contains(FailedConditionForNormalUse.NOT_MASTERED)
				|| failedConditions.contains(FailedConditionForNormalUse.NOT_POSITION_REFERENCED)
				|| failedConditions.contains(FailedConditionForNormalUse.NOT_GMS_REFERENCED)) {
			

			ThreadUtil.milliSleep(200);
			
			if (isSystemCheck) { 
				Robert_2_3.system_Check_Needed = true;

				isSystemCheck = false;
				if (this.excutionController != null) {
					this.excutionController.end();
				}
//				while (Training.motionContainer != null && !Training.motionContainer.isFinished()) {
//					Training.motionContainer.cancel();
//					ThreadUtil.milliSleep(100);
//				}
								
				this.logger.warn("ROBERT: system check is needed!, APP_ERROR");
				try {
					tcpConnection.sendMessage(UserCmd.SYS_CHECK.toString());
				} catch (IOException e) {
					this.logger.info("ROBERT: Failed to send system check to the control interface" + e);
				}
				
				if (ledColor.isFlashing()) {
					ledColor.stopFlashingNow();
					ThreadUtil.milliSleep(100);
				}

				linkageIOGroup.setLEDLightGreen(false);
				linkageIOGroup.setLEDLightRed(false);
				ThreadUtil.milliSleep(100);
				ledColor.startFlashing(Color.YELLOW);
			}
		} else {
			isSystemCheck = true;
		}

	}
	public void terminate() {
		running = false;
	}

	public void setMotionControler(ExcutionController _excutionController) {
		
		this.excutionController = _excutionController;
	}
}
