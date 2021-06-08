package shared;

import java.util.EnumMap;

import javax.inject.Inject;

import com.kuka.task.ITaskLogger;

public class NaviController {
	public static EnumMap<ButtonTypeEnum, TrainingTypeEnum> trainingNavi;
	public static EnumMap<ButtonTypeEnum, Integer> buttonClicked;
	private ButtonTypeEnum navi;
	
	@Inject
	private ITaskLogger logger;
	
	public NaviController() {
		// creating dictionary
		trainingNavi = new EnumMap<ButtonTypeEnum, TrainingTypeEnum>(ButtonTypeEnum.class);
		buttonClicked = new EnumMap<ButtonTypeEnum, Integer>(ButtonTypeEnum.class);

	}

	/**
	 * It checks if the button was clicked
	 * 
	 * @param button
	 * @return boolean
	 */
	public Integer getBtClickingState(ButtonTypeEnum button) {
		//logger.info("Robert: button clicked: " + button + ", state: " + buttonClicked.get(button));
		return buttonClicked.get(button);
	}

	/**
	 * Set the value of the button clicked true or false
	 * 
	 * @param bt
	 *            of type enumeration
	 * @param clickingNumber
	 *            of type boolean
	 */
	public void setBTClickingState(ButtonTypeEnum bt, Integer clickingNumber) {
		logger.info("Robert: setting button: " + bt + " Clicked: " + clickingNumber );
		buttonClicked.put(bt, clickingNumber);
	}

	/**
	 * Setup the training type if it has the type NONE and reset button to
	 * clicked false. Make sure the type is NONE to call the method again.
	 * 
	 * @param
	 */
	public void setTrainingType(TrainingTypeEnum type) {
		if (type == TrainingTypeEnum.NONE) {
			trainingNavi.put(ButtonTypeEnum.RECORD, type);
			trainingNavi.put(ButtonTypeEnum.PLAY, type);
			trainingNavi.put(ButtonTypeEnum.END, type);
			navi.setLevel(type);
			resetButtons();
		}
	}
	
	public EnumMap<ButtonTypeEnum, TrainingTypeEnum> getTrainingType(){
		
		return trainingNavi;
		
	}

	/**
	 * reseting the training type of the buttons to NONE and clicked to false.
	 */
	public static void resetNavi() {
		trainingNavi.put(ButtonTypeEnum.RECORD, TrainingTypeEnum.NONE);
		trainingNavi.put(ButtonTypeEnum.PLAY, TrainingTypeEnum.NONE);
		trainingNavi.put(ButtonTypeEnum.END, TrainingTypeEnum.NONE);
		trainingNavi.put(ButtonTypeEnum.NONE, TrainingTypeEnum.NONE);
				
		resetButtons();
	}

	/**
	 * Reset the buttons to by setting the value of each to false
	 */
	public static void resetButtons() {
		buttonClicked.put(ButtonTypeEnum.RECORD, 0);
		buttonClicked.put(ButtonTypeEnum.PLAY, 0);
		buttonClicked.put(ButtonTypeEnum.END, 0);
		
	}

}
