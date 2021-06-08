package backgroundTask;

import guidedMode.Guided;

import java.io.IOException;
import javax.inject.Inject;
import main.Robert_2_3;
import shared.LedColor;
import shared.LedColor.Color;
import shared.TCPConnection;
import shared.Training;
import util.UserCmd;
import activeMode.Active;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.BodyIOGroup;
import com.kuka.generated.ioAccess.LinkageIOGroup;
import com.kuka.med.controllerModel.MedController;
import com.kuka.med.robotStateEvent.IRobotStateListener;
import com.kuka.med.robotStateEvent.RobotStateEvent;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPIBackgroundTask;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.applicationModel.tasks.UseRoboticsAPIContext;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.EmergencyStop;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.statuscontroller.api.IStatusController;
import com.kuka.statuscontroller.api.IStatusListener;
import com.kuka.statuscontroller.api.StatusEvent;
import com.kuka.statuscontroller.statusgroups.DefaultStatusGroups;

/**
 * Implementation of a cyclic background task.
 * <p>
 * It provides the {@link RoboticsAPICyclicBackgroundTask#runCyclic} method
 * which will be called cyclically with the specified period.<br>
 * Cycle period and initial delay can be set by calling
 * {@link RoboticsAPICyclicBackgroundTask#initializeCyclic} method in the
 * {@link RoboticsAPIBackgroundTask#initialize()} method of the inheriting
 * class.<br>
 * The cyclic background task can be terminated via
 * {@link RoboticsAPICyclicBackgroundTask#getCyclicFuture()#cancel()} method or
 * stopping of the task.
 * 
 * @see UseRoboticsAPIContext
 * 
 */

public class StateHelper extends RoboticsAPIBackgroundTask implements IRobotStateListener, IStatusListener {
	private MedController controller;
	private LinkageIOGroup linkageLights;
	public static boolean cancelMotion;
	public static boolean motionPaused;
	public static boolean running;
	public static boolean ESM_5_STOP;

	private LBR lbr;
	@Inject
	private IStatusController statusController;

	@Inject
	private BodyIOGroup bodyIOGroup;

	public static boolean eStopWasPressed;
	public static boolean isEstopPressed;
	public static boolean redLight;
	private LedColor ledColor;
	private static TCPConnection tcpConnection;
	private boolean app_running;
	private boolean onBattery;
	private boolean power;
	private boolean critical_battery;
	private boolean eStopSent;
	private boolean cancelAllMotions;

	@Override
	public void initialize() {

		eStopWasPressed = false;
		cancelMotion = false;
		eStopSent = false;
		controller = (MedController) getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(controller, "LBR_Med_14_R820_1");

		linkageLights = new LinkageIOGroup(controller);

		ledColor = new LedColor(getApplicationData(), controller);

		controller.addRobotStateListener(this, lbr);

		// this.execService = (SunriseExecutionService)
		// controller.getExecutionService();

		statusController.addStatusListener(this, DefaultStatusGroups.APPLICATION_READY, DefaultStatusGroups.APPLICATION_RUNNING,
				DefaultStatusGroups.SAFETY_STOP, DefaultStatusGroups.ERROR_GENERAL);

	}

	@Override
	public void run() {

		while (true) {

			try {
				if (cancelMotion) {
					cancelMotion = false;
					if (Training.motionContainer != null) {
						Training.motionContainer.cancel();
						Training.remainingMotions.clear();
						Training.isMotionCancelled = true;

					}
					if (Training.softMc != null) {
						Training.softMc.cancel();
					}
				}

				if (tcpConnection == null) {
					tcpConnection = Robert_2_3.getTcpConnection();

				}

				powerStatus();

			} catch (Exception e) {
				e.printStackTrace();
			}

			ThreadUtil.milliSleep(100);

		}

	}

	/**
	 * check the status of the power signals, battery, power, and critical
	 * battery level.
	 * 
	 * @throws IOException
	 */
	private void powerStatus() throws IOException {

		if (bodyIOGroup.getBatteryAlarm() && !critical_battery) {

			if (Robert_2_3.getTcpConnection() != null) {
				tcpConnection.sendMessage(UserCmd.CRITICAL_BATTERY.toString());
				critical_battery = true;
				power = false; // false
				onBattery = true; // true
			}

		} else if (bodyIOGroup.getOnBattery() && !onBattery) {

			if (Robert_2_3.getTcpConnection() != null) {
				tcpConnection.sendMessage(UserCmd.ONBATTERY.toString());
				onBattery = true;
				power = false;
				critical_battery = false;
			}

		} else if (!bodyIOGroup.getOnBattery() && !power) {

			if (Robert_2_3.getTcpConnection() != null) {
				tcpConnection.sendMessage(UserCmd.POWER.toString());
				power = true;
				onBattery = false;
				critical_battery = false;
			}
		}
	}

	@Override
	public void dispose() {
		// remove listener registration of this application
		controller.removeRobotStateListener(this);
		super.dispose();
	}

	@Override
	public void onRobotStateChanged(RobotStateEvent event) {

		app_running = true;

		if (lbr.getSafetyState().getEmergencyStopEx() == EmergencyStop.ACTIVE) { 
			
			redLight = true;
			ledColor.stopFlashingNow();
			linkageLights.setLEDLightRed(true);
			linkageLights.setLEDLightGreen(false);

			eStopWasPressed = true;
			try {
				if (eStopSent == false) {
					tcpConnection.sendMessage(UserCmd.ESTOP.toString());
					eStopSent = true;
				}
			} catch (IOException e) { 
				e.printStackTrace();
			}

		} else if (lbr.getSafetyState().getEmergencyStopEx() == EmergencyStop.INACTIVE) {
			
			cancelAllMotions = true;
			if (eStopWasPressed) {
				eStopWasPressed = false;
				isEstopPressed = true;
				if (!lbr.isMastered()) {
					ledColor.startFlashing(Color.YELLOW);
				} else {
					linkageLights.setLEDLightRed(false);
					linkageLights.setLEDLightGreen(false);

					try {
						tcpConnection.sendMessage(UserCmd.ESTOP_RELEASED.toString());
						eStopSent = false;
					} catch (IOException e) {
						e.printStackTrace();
					}

				}
				

				Robert_2_3.userCommand = UserCmd.ESTOP;

			}
		}

	}

	@Override
	public void onStatusCleared(StatusEvent statusEvent) {
		// do nothing
	}

	@Override
	public void onStatusSet(StatusEvent statusEvent) {

		if (statusEvent.getStatusGroup().getId().equals("application_running")) {
			redLight = false;
			app_running = true;
			ESM_5_STOP = true;
			Robert_2_3.activeAttachButton = true;
			Robert_2_3.userCommand = UserCmd.NONE;
			if (cancelAllMotions) {
				cancelAllMotions = false;
				while (!Training.motionContainer.isFinished()) {
					Training.motionContainer.cancel();
					ThreadUtil.milliSleep(100);
				}
			}
			
		} 

	}
}
