package util;

import javax.inject.Inject;

import com.kuka.med.controllerModel.MedController;
import com.kuka.med.cyclicBrakeTest.BrakeTestDiskSpaceState;
import com.kuka.med.cyclicBrakeTest.BrakeTestMonitor;
import com.kuka.med.cyclicBrakeTest.BrakeTestMonitorEvent;
import com.kuka.med.cyclicBrakeTest.BrakeTestOutcome;
import com.kuka.med.cyclicBrakeTest.CyclicMonitoringState;
import com.kuka.med.cyclicBrakeTest.IBrakeTestMonitorListener;
import com.kuka.med.cyclicBrakeTest.OverallBrakeTestResult;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.task.ITaskLogger;

/**
 * Helper class for the cyclic brake test of the LBR Med.
 * <p>
 * Brake tests need to be performed periodically by an LBR Med after each booting and after a certain cyclic time. This
 * class uses the BrakeTestMonitor to enforces those brake tests by listening for BrakeTestMonitor events in order to
 * notify if a brake test is due or overdue. It also offers the methods to execute a brake test.
 * <p>
 * The start pose of a brake test is hard coded in this class, but can be customized.
 * <p>
 * Listening to BrakeTestMonitor state changes is realized by implementing IBrakeTestMonitorListener.
 * <p>
 * WARNING: The implementation here is just an example and not suited for any medical purpose or application.
 */
public class BrakeTestHelper implements IBrakeTestMonitorListener
{
	@Inject
    private ITaskLogger _logger;

    /**
     * Start pose of the robot for the brake test.
     */
    private static final JointPosition BRAKE_TEST_START_POSE = new JointPosition(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /**
     * The response time is set to 1200s, so 20min before a brake test is overdue and all robot motions are stopped an
     * event will be raised by the BrakeTestMonitor notifying all registered BrakeTestMonitorListeners of the situation.
     */
    private static final int RESPONSE_TIME = 1200; // in [s]

    /**
     * Cyclic brake tests need to be performed each booting and after a certain cyclic time.
     * The BrakeTestMonitor enforces those brake-tests and informs if a brake test is due.
     */
    private BrakeTestMonitor _brakeTestMonitor;

    /**
     * Data fields to store information of the last BrakeTestMonitorEvent, which include
     * <ul>
     * <li>the current state of the Cyclic Brake Test Monitoring State Machine,</li>
     * <li>a flag if postponement of the brake test is currently possible,</li>
     * <li>a flag if the brake test is executable,</li>
     * <li>the status of the disk if enough space is available for logging of the brake test results.</li>
     * </ul>
     */
    private CyclicMonitoringState _currentState = CyclicMonitoringState.SYSTEM_STARTED;
    private boolean _postponementPossible = false;
    private boolean _brakeTestExecutable = true;
    private BrakeTestDiskSpaceState _diskStatus = BrakeTestDiskSpaceState.UNKNOWN;


    /**
     * Helper class for the cyclic brake test of the LBR Med.
     * <p>
     * Brake tests need to be performed periodically by an LBR Med after each booting and after a certain cyclic time.
     * This class uses the BrakeTestMonitor to enforce those brake tests by listening for BrakeTestMonitor events in
     * order to notify if a brake test is due as well as offering methods to execute such a brake test.
     * 
     * @param medController
     *            The MedController needed by the BrakeTestMonitor.
     * @param logger
     *            The logger to print out text messages.
     */
    public BrakeTestHelper(MedController medController, ITaskLogger logger)
    {
        _brakeTestMonitor = new BrakeTestMonitor(medController);

        // register this class to listen to brake test monitoring events to e.g. be informed if a brake test is due
        _brakeTestMonitor.addMonitoringListener(this);

        _logger = logger;

        // set response time
        int actualResponseTime = _brakeTestMonitor.setResponseTime(RESPONSE_TIME);
        if (actualResponseTime != RESPONSE_TIME)
        {
            _logger.info("Response time was set too high. Value has to be below: "
                    + RESPONSE_TIME);
        }

        // initialize values once at the beginning, but they will be updated by events afterwards
        _currentState = _brakeTestMonitor.getCurrentState();
        _postponementPossible = _brakeTestMonitor.isPostponementPossible();
        _brakeTestExecutable = _brakeTestMonitor.isBrakeTestExecutable();
        _diskStatus = _brakeTestMonitor.checkFreeDiskSpace();

        _logger.info("Current info from BrakeTestMonitor: State = " + _currentState.toString() +
                ", Postponement Possible = " + _postponementPossible + ", BrakeTestExecutable = " +
                _brakeTestExecutable + ", Disk Status = " + _diskStatus.toString());
    }

    /**
     * This method is called each time the brake test monitoring state machine changes its state (e.g. when a brake-test
     * is due or overdue) and sends out an UDP info message to the external client containing the events information.
     * 
     * IMPORTANT: Only do short tasks in this method (e.g. setting a member variable) or start another thread for a
     * more complex task, because the framework expects to quickly return from this method and will otherwise throw
     * "CommandInvalidException: Sunrise did not respond to the SPR".
     * 
     * @param event
     *            Event containing the information of the current state of the BrakeTestMonitor.
     */
    @Override
    public void onMonitoringStateChanged(BrakeTestMonitorEvent event)
    {
        // if state changed, print it and inform external client (if connected) with a UDP message
        if (!_currentState.equals(event.getCurrentCyclicMonitorState()))
        {
            _logger.info("CyclicBrakeTestMonitorState changed to " + event.getCurrentCyclicMonitorState().toString());

        }

        _currentState = event.getCurrentCyclicMonitorState();
        _postponementPossible = event.isPostponementAcceptable();
        _brakeTestExecutable = event.isBrakeTestExecutable();
        _diskStatus = event.getStateOfFreeDiskSpace();
    }


    /**
     * Execute a LBR Med specific brake test if possible.
     * 
     * @param lbrMed
     *            The LBR Med this brake test is executed by.
     * @return True if brake test was possible and successful and false, otherwise.
     */
    public boolean handleBrakeTest(LBR lbrMed)
    {
        if (!isBrakeTestPossibleNow())
        {
            return false;
        }

        _brakeTestMonitor.setInitialPositionForBrakeTest(BRAKE_TEST_START_POSE, lbrMed);

        BrakeTestOutcome outcome = _brakeTestMonitor.executeBrakeTest(lbrMed);

        if (outcome.getOverallResult() == OverallBrakeTestResult.SUCCESSFUL)
        {
            return true;
        }

        if (outcome.getOverallResult() == OverallBrakeTestResult.WARNING)
        {
            _logger.warn("The overall result of the brake test was WARNING! The brake test was successful and the " +
                    "brakes are able to hold the robot as the measured torques are in the tolerance range. But the " +
                    "test almost reached the torque limits at least for one brake!");
            return true;
        }

        _logger.error("The brake test failed! The overall result of the brake test was: " +
                outcome.getOverallResult().toString());
        return false;
    }

    /**
     * Commands the brake test monitor to postpone the brake test after checking some preconditions.
     */
    public boolean postponeBrakeTest()
    {
        String logText = "Brake test postponement successful!";
        boolean success = true;

        // In this implementation postponing a brake test in states OK or RUNNING is blocked. In these states postponing
        // might reduce the time till the brake test is overdue instead of extending it. Postponing is allowed in state
        // FEEDBACK_REQUIRED. Also if another thread already postponed this check avoids postponing twice.
        if (_currentState.equals(CyclicMonitoringState.OK) || _currentState.equals(CyclicMonitoringState.RUNNING))
        {
            logText = "Postponement not necessary in Brake Test Monitor state " + _currentState;
            success = true;
        }
        else if (!_postponementPossible)
        {
            logText = "A postponement is not possible! Please perform a brake test.";
            success = false;
        }
        else if (!_brakeTestMonitor.postponeBrakeTest())
        {
            logText = "Unexpected Error - Postponement not possible!";
            success = false;
        }


        _logger.info(logText);
        return success;
    }

    /**
     * Checks if a brake test postponement is possible and recommendable. It is for example possible, but not
     * recommended if the brake test monitor is in state OK or RUNNING, because postponing then could lead to an
     * unintentional reduction of the time till the brake test is overdue.
     * 
     * @return True, if postponement is possible and Cyclic Brake Test Monitor is in FEEDBACK_REQUIRED state.
     */
    public boolean isPostponementPossible()
    {
        return (_postponementPossible
        && (!_currentState.equals(CyclicMonitoringState.OK)
        && (!_currentState.equals(CyclicMonitoringState.RUNNING))));
    }

    /**
     * Get the current state of the CyclicMonitoring State Machine.
     * 
     * @return The current state of the CyclicMonitoring State Machine.
     */
    public CyclicMonitoringState getCurrentState()
    {
        return _currentState;
    }

    /**
     * Get the time left till a brake test is overdue, i.e. the time all robot motions will be stopped and the
     * CyclicMonitor State Machine goes into state ERROR.
     * <p>
     * This 'overdue' time is different to the time the brake test is 'due'. A test is 'due' some time before the test
     * is 'overdue'. This time is set by the constant RESPONSE_TIME.
     * 
     * @return Time in seconds till a brake test is overdue.
     */
    public int getTimeTillBrakeTestOverdue()
    {
        return _brakeTestMonitor.getTimeTillBraketestOverdue();
    }

    /**
     * Checks if the brake test monitor is blocking robot motions, i.e. the brake test monitor is in ERROR or
     * FATAL_ERROR state. In case of ERROR a brake test can solve this problem, but in case of FATAL_ERROR the robot
     * application can not recover on its own.
     * 
     * @return True if brake test is overdue. False, otherwise.
     */
    public boolean isBrakeTestMonitorBlockingMotions()
    {
        switch (_currentState)
        {
        case ERROR:
            _logger.error("Brake-test is overdue! Restart app or use BrakeTestMonitor.executeBrakeTest()!");
            return true;
        case FATAL_ERROR:
            _logger.error("Brake-tests failed due to serious problems with at least one brake! Robot movement stopped! "
                    + "Please contact customer service!");
            return true;
        default:
            return false;
        }
    }

    /**
     * Checks if the brake test is possible right now by checking if the last brake test monitor event signaled true for
     * isBrakeTestExecutable. It also provides logger messages about the current disk space needed for brake tests.
     * 
     * @return True, if brake test can be executed right now and false, otherwise.
     */
    private boolean isBrakeTestPossibleNow()
    {
        switch (_diskStatus)
        {
        case ENOUGH_SPACE:
            break;
        case LOW_SPACE:
            _logger.warn("Brake-test possible, but disk space LOW! Please contact customer service soon!");
            break;
        case NOT_ENOUGH_SPACE:
            _logger.warn("Brake-test NOT possible! Disk space NOT enough. Contact customer service!");
            break;
        default:
            _logger.warn("Unknown disk space status for brake-test! Try checking again!");
            return false;
        }

        if (!_brakeTestExecutable)
        {
            _logger.warn("Brake-test NOT possible! Current brake-test monitoring state: " + _currentState);
            return false;
        }

        return true;
    }

    /**
     * Checks if the brake test monitor allows jogging of the robot at the moment.
     * 
     * @return True, if the brake test monitor is in a state where jogging is possible.
     */
    public boolean isJoggingPossibleNow()
    {
        return (_currentState == CyclicMonitoringState.SYSTEM_STARTED
                || _currentState == CyclicMonitoringState.FEEDBACK_REQUIRED
                || _currentState == CyclicMonitoringState.OK);
    }

    public void dispose()
    {
        _brakeTestMonitor.removeMonitoringListener(this);
    }

}
