package util;

/**
 * Enum of all possible states of the robot application state machine. 
 *
 */
public enum AppStateMachine
{

    /**
     * Initial state.
     */
    START_STATE(0),
    /**
     * Master all non-mastered axes.
     */
    MASTERING(1),
    /**
     * The application produced an error and needs to quit.
     */
    APP_ERROR(2),
    /**
     * Executing a brake-test is optional at this time, but recommended.
     */
    BRAKE_TEST_OPTIONAL(3),
    /**
     * Executing a brake test is necessary at this time before using the robot for its intended use.
     */
    BRAKE_TEST_HANDLING(4),
    /**
     * Check if the robot is ready for its intended use.
     */
    CHECK_ROBOT_STATE(5),
    /**
     * Executing GMS and position referencing.
     */
    REFERENCING(6),
    /**
     * Main application running state.
     */
    RUNNING(7),
    /**
     * Executing hand guiding motions.
     */
    HANDGUIDING(8),
    /**
     * Application is finishing successfully.
     */
    EXITING(9);

    /** Constructor. */
    private AppStateMachine(int value)
    {
        _value = value;
    }

    private final int _value;

    /**
     * Gets the state defined through the given ID. Fails if no state could be found.
     * 
     * @param id
     *            The id.
     * @return The matching enum type.
     */
    public static AppStateMachine fromInt(int id)
    {
        for (AppStateMachine instance : values())
        {
            if (instance._value == id)
            {
                return instance;
            }
        }
        throw new IllegalArgumentException("No state for given id found: " + id);
    }

    /**
     * Converts the state in the integer value. Fails if no state could be found.
     * 
     * @param state
     *            The enum state.
     * @return The matching integer value.
     */
    public static int toInt(AppStateMachine state)
    {
        for (AppStateMachine instance : values())
        {
            if (instance == state)
            {
                return instance._value;
            }
        }
        throw new IllegalArgumentException("No state for given state found: " + state);
    }
}
