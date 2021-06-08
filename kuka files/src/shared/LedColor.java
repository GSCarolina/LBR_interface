/**
 * 
 */
package shared;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.controllerModel.Controller;

/**
 * @author Firas Amin
 * 
 */
public class LedColor {

	public enum Color {
		GREEN, YELLOW, RED
	}

	private IApplicationData appData;
	private Controller controller;
	private boolean flashing;
	public LedColor(IApplicationData _iApplicationData, Controller _Controller) {
		appData = _iApplicationData;
		controller = _Controller;

	}

	/**
	 * This method used to start a thread for flashing light
	 * 
	 * @param
	 */
	public void startFlashing(Color color) {
		FlashingLed flashingLed = new FlashingLed(controller, appData);
		appData.getProcessData("flashing").setValue(true);

		switch (color) {

			case GREEN :
				appData.getProcessData("flashingValue").setValue(11);
				break;

			case YELLOW :
				appData.getProcessData("flashingValue").setValue(21);
				break;
			case RED :
				appData.getProcessData("flashingValue").setValue(31);
				break;
		}

		setFlashing(true);
		flashingLed.start();
	}

	public void stopFlashing() {
		appData.getProcessData("flashingValue").setValue(0);
		ThreadUtil.milliSleep(1000);
		appData.getProcessData("flashing").setValue(false);

		setFlashing(false);
	}

	public void stopFlashingNow() {
		appData.getProcessData("flashingValue").setValue(0);
		appData.getProcessData("flashing").setValue(false);
		setFlashing(false);
	}

	private void setFlashing(boolean flashing) {
		this.flashing = flashing;
	}

	public boolean isFlashing() {
		return this.flashing;
	}

}
