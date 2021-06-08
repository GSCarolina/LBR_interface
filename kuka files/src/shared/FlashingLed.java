/**
 * 
 */
package shared;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.LinkageIOGroup;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationData;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.task.ITaskLogger;

/**

 * This cyclic background task is responsible for flashing yellow with either
 * the LED or the play/pause button. The flashing is specified to occur with a
 * frequency of 0.8 Hz and a duty cycle of 50 %. A frequency of 0.8 Hz means 1
 * blink per 1.25 second, and with a 50 % duty cycle, this means that the state
 * of the yellow light in the LED or button (either true or false) should be
 * changed every 625 milliseconds.
 * 
 * @author Firas Amin
 * */

public class FlashingLed extends Thread {
	private LinkageIOGroup linkageIOGroup;
	private RoboticsAPIApplicationData roboticApiAppData;
	private int flashingValue;
	@Inject
	private ITaskLogger logger;
	private boolean isOn = true;
	public FlashingLed(Controller controller, IApplicationData appData) {
		roboticApiAppData = (RoboticsAPIApplicationData) appData;
		// new BodyIOGroup(controller);
		linkageIOGroup = new LinkageIOGroup(controller);
	}

	@Override
	public void run() {

		while (roboticApiAppData.getProcessData("flashing").getValue()) {
			flashingValue = roboticApiAppData.getProcessData("flashingValue").getValue();

			if (flashingValue != 0) {
				isOn = true;
			}

			switch (flashingValue) {
				case 0 :
					if (isOn) {
						// reset
						linkageIOGroup.setLEDLightRed(false);
						linkageIOGroup.setLEDLightGreen(false);
						isOn = false;
					}
					break;
				case 11 :
					// linkage LED flashes green
					linkageIOGroup.setLEDLightRed(false);
					linkageIOGroup.setLEDLightGreen(!linkageIOGroup.getLEDLightGreen());
					try {
						ThreadUtil.milliSleep(1000);
					} catch (Exception e) {
						logger.info("Sleep interapted for linkage yellow flashing");
					}
					break;
				case 21 :
					// linkage LED flashes yellow, turn on both LEDs to give
					// yellow light
					linkageIOGroup.setLEDLightRed(!linkageIOGroup.getLEDLightRed());
					linkageIOGroup.setLEDLightGreen(!linkageIOGroup.getLEDLightGreen());
					try {
						ThreadUtil.milliSleep(650);
					} catch (Exception e) {
						logger.info("Sleep interapted for linkage yellow flashing");
					}
					break;
				case 31 :
					// linkage LED flashes red
					linkageIOGroup.setLEDLightGreen(false);
					linkageIOGroup.setLEDLightRed(!linkageIOGroup.getLEDLightRed());
					try {
						ThreadUtil.milliSleep(250);
					} catch (Exception e) {
						logger.info("Sleep interapted for linkage red flashing");
					}
					break;
				case 41 :
					// linkage LED lights red constant
					linkageIOGroup.setLEDLightRed(true);
					linkageIOGroup.setLEDLightGreen(false);
					break;
			}
		}
	}
}
