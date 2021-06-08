package util;

/**
 * Enum of all possible user commands that are allowed.
 * 
 */
public enum UserCmd
{
    NONE(-1, "NONE"),
    EXIT_APP(0, "EXIT_APP"),
    RESTART_APP(1, "RESTART_APP"),
    TEST_BRAKES(2, "TEST_BRAKES"),
    POSTPONE_BRAKE_TEST(3, "POSTPONE_BRAKE_TEST"),
    UNMASTER(4, "UNMASTER"),
    REFERENCE(5, "REFERENCE"),
    IGNORE_MOTION(6, "IGNORE_MOTION"),
    START_MOTION(7, "START_MOTION"),
    START_HANDGUIDING(8, "START_HANDGUIDING"),
    OK(9, "OK"),
    LOG(10,"LOG"),
    ADD(11,"ADD"),
    START(12,"START"),
    UPDATE(13,"UPDATE"),
    PAUSE(14,"PAUSE"),
    END(15,"END"),
    GUIDED(16,"GUIDED"),
    ACTIVE(17,"ACTIVE"),
    HYBRID(18,"HYBRID"),
    READY(19, "READY"),
    REC_STARTED(20, "REC_STARTED"),
    REC_DONE(21, "REC_DONE"), 
    CYCLE_DONE(22, "CYCLE_DONE"), 
    SET_DONE(23 , "SET_DONE"),
    ROB_TRANSPORT(24, "ROB_TRANSPORT"),
    KUKA_TRANSPORT(25, "KUKA_TRANSPORT"), 
    LOGGING_DONE(26, "LOGGING_DONE"), 
    RESUME(27, "RESUME"),
    PLAYING(28, "PLAYING"),
    PAUSED(29, "PAUSED"),
    ATTACH_PRESSED(30, "ATTACH_PRESSED"),
    ACTIVITY_PERCENT(31, "ACTIVITY_PERCENT"),
    SYS_CHECK(32, "SYS_CHECK"),
    APP_READY(33,"APP_READY"),
    APP_PAUSED(34,"APP_PAUSED"),
    SAFETY_STOP(35,"SAFETY_STOP"),
    ERROR_GENERAL(36,"ERROR_GENERAL"),
    ONBATTERY(37,"ONBATTERY"),
    CRITICAL_BATTERY(38,"CRITICAL_BATTERY"),
    BOOST(39,"BOOST"),
    ESTOP(40,"ESTOP"),
    MOTION_PAUSED(41, "MOTION_PAUSED"), 
    ESTOP_RELEASED(42, "ESTOP_RELEASED"), 
    EXERCISE_DONE(43,"EXERCISE_DONE"),
    POWER(44, "POWER"), 
    NEXT_MOTION(45,"NEXT_MOTION"),
    MOTION_STARTED(46,"MOTION_STARTED"),
    KG_FORCE(47,"KG_FORCE"),
    MOTION_TRANSITION(48,"MOTION_TRANSITION"),
    LOGGING_CANCEL(49,"LOGGING_CANCEL"),
    RERECORD(50,"RERECORD"), 
    FINAL_TRANSITION(51,"FINAL_TRANSITION"), 
    END_POSITION(52, "END_POSITION"), 
    REBOOT(53, "REBOOT"),
    SHUTDOWN(54,"SHUTDOWN"),
    HARD_LIMIT(55,"HARD_LIMIT"),
    UNMASTERED(56, "UNMASTERED"),
    CYCLE_START(57, "CYCLE_START"),
    INVALID_RECORDING(58, "INVALID_RECORDING"),
    VALID_RECORDING(59, "VALID_RECORDING"), PLAY(60,"PLAY"),
    REMOVE(61,"REMOVE"), 
    SPEED(62,"SPEED"),
    TRANSITION_START(62,"TRANSITION_START"),
    TRANSITION_DONE(63,"TRANSITION_DONE"),
    PROGRESS(64,"PROGRESS"),
    SKIP(65,"SKIP"), 
    SKIP_DONE(66,"SKIP_DONE"), 
    REPEAT(67,"REPEAT"), 
    VEIN_PUMP(68,"VEIN_PUMP");

    /** Private constructor. */
    private UserCmd(int value, String label)
    {
        _value = value;
        _label = label;
    }

    private final int _value;
    private final String _label;

    /**
     * Gets the state defined through the given ID. Fails if no state could be found.
     * 
     * @param id
     *            The id.
     * @return The matching enum type.
     */
    public static UserCmd fromInt(int id)
    {
        for (UserCmd instance : values())
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
    public static int toInt(UserCmd state)
    {
        for (UserCmd instance : values())
        {
            if (instance == state)
            {
                return instance._value;
            }
        }
        throw new IllegalArgumentException("No corresponding state found for: " + state);
    }

    /**
     * Gets the state defined through the given String. Fails if no state could be found.
     * 
     * @param userCmdString
     *            User command as string.
     * @return The matching enum type.
     */
    public static UserCmd fromString(String userCmdString)
    {
        for (UserCmd instance : values())
        {
            if (instance._label.equals(userCmdString))
            {
                return instance;
            }
        }
        throw new IllegalArgumentException("No command with given name found: " + userCmdString);
    }

    @Override
    public String toString()
    {
        return _label;
    }
}
