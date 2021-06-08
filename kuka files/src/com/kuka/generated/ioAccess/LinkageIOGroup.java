package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>Linkage</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * ./.
 */
@Singleton
public class LinkageIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'Linkage'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'Linkage'
	 */
	@Inject
	public LinkageIOGroup(Controller controller)
	{
		super(controller, "Linkage");

		addDigitalOutput("LEDLightGreen", IOTypes.BOOLEAN, 1);
		addDigitalOutput("LEDLightRed", IOTypes.BOOLEAN, 1);
		addInput("RecordButton", IOTypes.BOOLEAN, 1);
	}

	/**
	 * Gets the value of the <b>digital output '<i>LEDLightGreen</i>'</b>.<br>
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
	 * @return current value of the digital output 'LEDLightGreen'
	 */
	public boolean getLEDLightGreen()
	{
		return getBooleanIOValue("LEDLightGreen", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>LEDLightGreen</i>'</b>.<br>
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
	 *            the value, which has to be written to the digital output 'LEDLightGreen'
	 */
	public void setLEDLightGreen(java.lang.Boolean value)
	{
		setDigitalOutput("LEDLightGreen", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>LEDLightRed</i>'</b>.<br>
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
	 * @return current value of the digital output 'LEDLightRed'
	 */
	public boolean getLEDLightRed()
	{
		return getBooleanIOValue("LEDLightRed", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>LEDLightRed</i>'</b>.<br>
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
	 *            the value, which has to be written to the digital output 'LEDLightRed'
	 */
	public void setLEDLightRed(java.lang.Boolean value)
	{
		setDigitalOutput("LEDLightRed", value);
	}

	/**
	 * Gets the value of the <b>digital input '<i>RecordButton</i>'</b>.<br>
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
	 * @return current value of the digital input 'RecordButton'
	 */
	public boolean getRecordButton()
	{
		return getBooleanIOValue("RecordButton", false);
	}

}
