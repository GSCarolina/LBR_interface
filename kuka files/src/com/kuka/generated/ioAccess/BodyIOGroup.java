package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>Body</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * ./.
 */
@Singleton
public class BodyIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'Body'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'Body'
	 */
	@Inject
	public BodyIOGroup(Controller controller)
	{
		super(controller, "Body");

		addInput("PlayPauseButton", IOTypes.BOOLEAN, 1);
		addInput("BatteryAlarm", IOTypes.BOOLEAN, 1);
		addInput("OnBattery", IOTypes.BOOLEAN, 1);
		addDigitalOutput("End", IOTypes.BOOLEAN, 1);
		addDigitalOutput("PauseControl", IOTypes.BOOLEAN, 1);
	}

	/**
	 * Gets the value of the <b>digital input '<i>PlayPauseButton</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'PlayPauseButton'
	 */
	public boolean getPlayPauseButton()
	{
		return getBooleanIOValue("PlayPauseButton", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>BatteryAlarm</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'BatteryAlarm'
	 */
	public boolean getBatteryAlarm()
	{
		return getBooleanIOValue("BatteryAlarm", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>OnBattery</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'OnBattery'
	 */
	public boolean getOnBattery()
	{
		return getBooleanIOValue("OnBattery", false);
	}

	/**
	 * Gets the value of the <b>digital output '<i>End</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'End'
	 */
	public boolean getEnd()
	{
		return getBooleanIOValue("End", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>End</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'End'
	 */
	public void setEnd(java.lang.Boolean value)
	{
		setDigitalOutput("End", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>PauseControl</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'PauseControl'
	 */
	public boolean getPauseControl()
	{
		return getBooleanIOValue("PauseControl", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>PauseControl</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'PauseControl'
	 */
	public void setPauseControl(java.lang.Boolean value)
	{
		setDigitalOutput("PauseControl", value);
	}

}
