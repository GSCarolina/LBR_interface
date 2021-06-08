package thesisUtilities;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import util.*;
import activeMode.Active;

import com.kuka.common.ThreadUtil;
import com.kuka.med.cyclicBrakeTest.BrakeTestMonitorEvent;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.task.ITaskLogger;

/**
 * This helper creates and handles UDP connection to an external client (e.g. ExampleMedGUI).
 * <p>
 * WARNING: The UDP communication is very basic and not secure. Do not use it in a public network and replace it with a
 * more suitable implementation complying to your product requirements and risk analysis.
 * <p>
 * This class will communicate with the last remote address it received a correctly structured ping message from (e.g.
 * 'DefaultApplication;Ping;1234').
 * <p>
 * This class can be run in its own thread as a Runnable if it needs to continuously check for incoming UDP packages.
 * <p>
 * WARNING: UDP does not guarantee the delivery of messages and the order in which they are received. Note that this
 * example class does not handle these cases sufficiently!
 * <p>
 * WARNING: The implementation here is just an example and not suited for any medical purpose or application.
 * 
 */
public class UdpState extends Thread
{

    private ITaskLogger _logger;

    /**
     * Strings used in UDP messages
     */
    private static final String PING = "Ping";
    private static final String PONG = "Pong";
    private static final String INFO = "Info";
    private static final String HEARTBEAT = "Heartbeat";
    private static final String GET_COMMAND = "Get_Command";
    private static final String COMMAND = "Command";
    private static final String POSTPONE_BRAKETEST = "Postpone_Braketest";
    private static final String BRAKE_TEST_MONITOR_EVENT = "BrakeTestMonitorEvent";

    /**
     * Members for socket communication with the external GUI (e.g. ExampleMedGUI)
     */
    private InetSocketAddress _externalClientAddress = null;
    private DatagramSocket _socket;
    private int _localPort;
    private static final int SOCKET_TIMEOUT = 20;
    private static final int UDP_PACKET_SIZE = 508;
    private static final String UTF_8 = "UTF-8";

    /**
     * Time to wait if an answer message to a previously sent message is expected.
     */
    private static final int TIME_TO_WAIT_FOR_ANSWER = 500;

    /**
     * Unique string identifier for packages (to distinguish between different receivers).
     */
    private String _packetIdentifier;

    /**
     * Packet counter for messages that expect an answer. It allows the receiver to ignore duplicate messages requesting
     * the same answer.
     */
    private int _packetCounter = 0;

    /**
     * Last command received from an external client. Should be reset to null if command was handled locally.
     */
    private UserCmd _commandAnswer = null;

    /**
     * Time last Ping message was received. This can be used to check if connection is still alive.
     */
    private long _timeOfLastHeartbeat = 0;

    /**
     * Flag to cancel waiting for an external command.
     */
    private boolean _cancelGetExternalCommand = false;

    /**
     * If an UDP message with "Postpone_Braketest;true" is received the command is forwarded to this BrakeTestHelper (if
     * instantiated).
     */
    private BrakeTestHelper _brakeTestHelper;
    
    // =================== State Machine =================== 
    public int status = 0;
    // =================== Recording trajectory process =================== 
    public List<JointPosition> recordedPathJS; 
    public int JS_index = 0, JS_size = 1;
    public boolean JS_checked = false;
    // =================== Follow trajectory from CoppeliaSim =================== 
    public double[] qA17, qA17d, tA17, fEE, pEE; // old variables
    public double[] sim_qA;
    public boolean sim_qA_error, sim_finished, sim_status;
    public int isMoving = 0, pathCnt = 0, pathPointer = 0;
    // vritual constraint
    public final int TYPE_NONE = 0, TYPE_WALL = 1, TYPE_CUBE = 2;
    public int type = TYPE_NONE;
    public double[] vcPoint = {0.0, 0.0, 0.0};
    
    //
    public String testMsg = new String();
    /**
     * Constructor.
     * 
     * @param localPort
     *            Local port to open and communicate over (Sunrise Cabinet typically allows 30000-30010)
     * @param packetIdentifier
     *            Unique string identifier for packages (to distinguish between different receivers)
     * @param logger
     *            The logger for text outputs.
     */
    public UdpState(int localPort, String packetIdentifier, ITaskLogger logger)
    {
        _localPort = localPort;
        _packetIdentifier = packetIdentifier;
        _logger = logger;
        qA17 = new double[7];
        tA17 = new double[7];
        pEE =  new double[6];	
        fEE =  new double[6];	// Linear force and momentum toguether
        //mEE  = new double[3];
        for (int i = 0; i<7; i++)
        {
        	qA17[i] = 0.0;
        	tA17[i] = 0.0;
        	if(i<6)
        	{
        		fEE[i] = 0.0;
        		//mEE[i] = 0.0;
        	}
        }
        // trajectory
        sim_qA = new double[7];
        sim_qA_error = false;
        sim_finished = false;
        sim_status = false;
        // initial position
        sim_qA[0] = Math.toRadians(0);
        sim_qA[1] = Math.toRadians(0);
        sim_qA[2] = Math.toRadians(0);
        sim_qA[4] = Math.toRadians(0);
        sim_qA[6] = Math.toRadians(0);
        
        sim_qA[3] = Math.toRadians(-90);
        sim_qA[5] = Math.toRadians(55);
        

		// trajectories start up
		recordedPathJS = new ArrayList<JointPosition>();
    }

    /**
     * Initialize socket for UDP communication to an external client.
     * 
     * @return True, if socket opened successfully. False, otherwise.
     */
    public boolean initUDP()
    {
        try
        {
            _socket = new DatagramSocket(null);
            _socket.setReuseAddress(true);
            _socket.bind(new InetSocketAddress(_localPort));
            _socket.setSoTimeout(SOCKET_TIMEOUT);
        }
        catch (Exception e)
        {
            _logger.error("Problems initializing UDP socket: ", e);
            return false;
        }

        return true;
    }

    /**
     * Wait for reception of an UDP message. If none arrives return empty array. If an invalid one arrives return empty
     * array. If a valid one arrives return the message.
     * 
     * @return Returns empty array if none arrived (SocketTimeout). Returns empty array if an invalid one arrived.
     *         Returns the content of the message if a valid one was received in time.
     * @throws IOException
     *             Forwards IOException of the socket.
     */
    public String[] receive() throws IOException
    {
        byte[] receivedData = new byte[UDP_PACKET_SIZE];
        DatagramPacket receivedPacket = new DatagramPacket(receivedData, receivedData.length);

        try
        {
            _socket.receive(receivedPacket);
            return handleMessage(receivedPacket);
        }
        catch (SocketTimeoutException e)
        {
            return new String[0];
        }
        
    }

    /**
     * Checks format of the incoming UDP messages and if the message is valid it is parsed and returned as a String
     * array. If the message is a ping message a pong is sent back and the remote address is stored as the destination
     * for all outgoing UDP packages of this instance.
     * 
     * Incoming messages have to be an UTF-8 String structured as e.g.
     * <ul>
     * <li>"DefaultApplication;Heartbeat" as a signal that the connection is active without expecting an answer (where
     * 'DefaultApplication' is an example for the packet identifier and 1234 for a packet counter value).</li>
     * <li>"DefaultApplication;Command;1234;EXIT APP" as an answer to a previous 'Get_Command' message from the
     * DefaultApplication (where 'DefaultApplication' is an example for the packet identifier, 1234 for a packet counter
     * value and EXIT APP for a command to exit the application.</li>
     * <li>"DefaultApplication;Postpone_Braketest;1" to postpone the braketest (where 'DefaultApplication' is an example
     * for the packet identifier and 1 for for true/approval).</li>
     * </ul>
     * 
     * @param receivedPacket
     *            DatagramPacket as received from the socket.
     * @return Returns the DatagramPacket's data field parsed into a String array or empty array if the data did not
     *         comply to the expected message structure.
     */
    private String[] handleMessage(DatagramPacket receivedPacket)
    {

        String[] message = new String(receivedPacket.getData()).split(";");

        for (int i = 0; i < message.length; i++)
        {
            // delete white space
            message[i] = message[i].trim();
        }

        if (message.length < 2)
        {
            _logger.error("Received UDP message with < 2 fields: " + new String(receivedPacket.getData()).trim());
            return new String[0];
        }
        
        _externalClientAddress = new InetSocketAddress(receivedPacket.getAddress(), receivedPacket.getPort());
        // state machine messages
        if (message[1].equals("STATUS")) {
			
        	sim_status = true;
        	sendStatusVariablesBack();
        	
        	return message;
		}
        else if (message[1].equals("PATH")) {
        	sim_status = false;
        	String dummyMsg = new String(message[2]);
        	//_logger.info("UDP received: " + message[0] + ";" + message[1] + ";" + message[3]);//
        	int index = Integer.parseInt(dummyMsg);
        	if(index >= 0 && index < JS_size)
        	{
        		JS_size = recordedPathJS.size();
        		sendPathBack(JS_size,index);
        	}
        	else
        	{
        		_logger.error("UDP Coppelia: Received wrong path index");
        	}
        	
        	
        	return message;       	
        }
        else if (message[1].equals("READY")) {
        	sim_status = false;
        	sendDummyBack();
        	JS_checked = true;
        	
        	return message;
		}
        else if (message[1].equals("RUNNING")) {
        	sim_status = false;
        	//_logger.info("Received running");
        	//sendJointsBack();
        	sendStatusVariablesBack();
        	// Update robot position
        	if(message.length > 8 && message.length < 18)
        	{
        		final double M_TO_MM = 1000;
        		pathPointer = Integer.parseInt(message[2]);
            	for(int k=0;k<7;k++)
            	{
            		sim_qA[k] = Double.parseDouble(message[k+3]);
            	}        	
            	//type = Integer.parseInt(message[10]);
            	type = (int)(Double.parseDouble(message[10]));
            	vcPoint[0] = Double.parseDouble(message[11])*M_TO_MM;
            	vcPoint[1] = Double.parseDouble(message[12])*M_TO_MM;
            	vcPoint[2] = Double.parseDouble(message[13])*M_TO_MM;
            	testMsg = "VC Point " + vcPoint[0] + " , " +vcPoint[1] + " , " + vcPoint[2];

        	}
        	else
        	{
        		sim_qA_error = true;
        		_logger.error("UDP Coppelia: Received wrong position to follow");
        	}
        	
        	//JS_checked = false;
        	
        	return message;
		}
        else if (message[1].equals("ENDPATH")) {
        	sim_status = false;
        	sendDummyBack();
        	sim_finished = true;
        	
        	return message;
		}
        // Follow simulator 
		else if (message[1].equals("FOLLOWER")) {
			sim_status = false;
			sendJointsBack();
		    return message;
		}

        _logger.error("Received incorrect formated UDP message: " + new String(receivedPacket.getData()).trim());
        
        return new String[0];
    }
    
    
//
    private void sendStatusVariablesBack(){
    	// Carol was here messing around
    	//String strMoving = new String(isMoving + ";");
    	String qMsg = new String("Helloooo");
    	String fullMsgString = new String(status + ";" + JS_size + ";" + isMoving + ";" + pathCnt + ";");
    	//_logger.info("Sending: " + fullMsgString);
        try
        {
            byte[] msg = fullMsgString.getBytes(UTF_8);
            _socket.send(new DatagramPacket(msg, msg.length, _externalClientAddress));
        }
        catch (Exception e)
        {
            _logger.error("Failed sending UDP STATUS variables to external device: ", e);
        }
    }
    
    private void sendPathBack(int pathL, int listIndex){
    	// Send 1 trajectory point in JS
		JointPosition _jointPos = recordedPathJS.get(listIndex);
		qA17[0] = _jointPos.get(JointEnum.J1);
		qA17[1] = _jointPos.get(JointEnum.J2);
		qA17[2] = _jointPos.get(JointEnum.J3);
		qA17[3] = _jointPos.get(JointEnum.J4);
		qA17[4] = _jointPos.get(JointEnum.J5);
		qA17[5] = _jointPos.get(JointEnum.J6);
		qA17[6] = _jointPos.get(JointEnum.J7);
    	
		String qMsg = new String(status + ";" + qA17[0]+";"+qA17[1]+";"+qA17[2]+";"+qA17[3]+";"+qA17[4]+";"+qA17[5]+";"+qA17[6]+";");
    	String fullMsgString = new String("PATH;" + pathL + ";" + listIndex + ";" + qMsg);
    	
    	//_logger.info("Sending: " + fullMsgString); //
        try
        {
            byte[] msg = fullMsgString.getBytes(UTF_8);
            _socket.send(new DatagramPacket(msg, msg.length, _externalClientAddress));
        }
        catch (Exception e)
        {
            _logger.error("Failed sending UDP STATUS variables to external device: ", e);
        }
    }
 
    private void sendJointsBack(){
    	// Send current joint position to Coppelia    	
		String qMsg = new String(qA17[0]+";"+qA17[1]+";"+qA17[2]+";"+qA17[3]+";"+qA17[4]+";"+qA17[5]+";"+qA17[6]+";");
    	String fullMsgString = new String(qMsg);
    	
    	//_logger.info("Sending: " + fullMsgString); //
        try
        {
            byte[] msg = fullMsgString.getBytes(UTF_8);
            _socket.send(new DatagramPacket(msg, msg.length, _externalClientAddress));
        }
        catch (Exception e)
        {
            _logger.error("Failed sending UDP STATUS variables to external device: ", e);
        }
    }
    
    private void sendDummyBack(){
    	// Carol was here messing around
    	String qMsg = new String("Helloooo");
    	String fullMsgString = new String(status + ";" + qMsg);
    	//_logger.info("Sending: " + fullMsgString);
        try
        {
            byte[] msg = fullMsgString.getBytes(UTF_8);
            _socket.send(new DatagramPacket(msg, msg.length, _externalClientAddress));
        }
        catch (Exception e)
        {
            _logger.error("Failed sending UDP STATUS variables to external device: ", e);
        }
    }
    
    /**
     * Send an answer message to a ping message (pong) back to the external client.
     * 
     */
    private void sendPongBack(int packetCounter)
    {
        String fullMsgString = new String(_packetIdentifier + ";" + PONG + ";" + packetCounter);

        try
        {
            byte[] msg = fullMsgString.getBytes(UTF_8);
            _socket.send(new DatagramPacket(msg, msg.length, _externalClientAddress));
        }
        catch (Exception e)
        {
            _logger.error("Failed sending UDP (" + PONG + ") message: ", e);
        }
    }

    /**
     * Send an info message to the external client. The message is structured as e.g.
     * "DefaultApplication;Info;MessageText".
     * 
     * If an info message is longer than the maximum UDP packet size it is split into several info messages.
     * 
     * @param messageText
     *            The message text to be put in the UDP message.
     * @return False, if no externalClient was registered to send the message to or if an exception was thrown when
     *         trying to send. True, otherwise.
     */
    public boolean sendInfoMessage(String messageText)
    {
        if (null == _externalClientAddress)
        {
            return false;
        }

        try
        {
            String msgHeader = new String(_packetIdentifier + ";" + INFO + ";");
            byte[] messageHeaderBytes = msgHeader.getBytes(UTF_8);
            byte[] messageTextBytes = messageText.getBytes(UTF_8);

            // info messages longer than max. UDP packet size need to be split
            int bytesAvailableForText = UDP_PACKET_SIZE - messageHeaderBytes.length;
            for (int i = 0; i <= messageTextBytes.length; i += bytesAvailableForText)
            {
                int textSegmentStart = i;
                int textSegmentEnd = i + bytesAvailableForText;
                if (textSegmentEnd > messageTextBytes.length)
                {
                    textSegmentEnd = messageTextBytes.length;
                }
                byte[] textSegment = Arrays.copyOfRange(messageTextBytes, textSegmentStart, textSegmentEnd);
                byte[] msg = Arrays.copyOf(messageHeaderBytes, messageHeaderBytes.length + textSegment.length);
                System.arraycopy(textSegment, 0, msg, messageHeaderBytes.length, textSegment.length);

                // send the message (synchronized as this method might be called by different threads)
                _logger.info("sending: " + new String(msg));
                synchronized (this)
                {
                    _socket.send(new DatagramPacket(msg, msg.length, _externalClientAddress));
                }
            }
        }
        catch (Exception e)
        {
            _logger.error("Failed sending UDP (" + INFO + ") message: ", e);
            return false;
        }

        return true;
    }

    /**
     * Send a list of valid commands to an external client frequently and wait for an answer containing which command
     * was chosen.
     * <p>
     * If no external client address is known yet, it will wait till a message arrives from such a client. A message
     * holding the possible commands is structured as e.g.
     * <p>
     * "DefaultApplication;Get_Command;1234;StartMotion;Exit;Start the motion?",
     * <p>
     * , where 1234 is the packet counter, StartMotion/Exit commands the external client can choose from and the last
     * field is a text message that can explain the list of possible commands.
     * 
     * Warning: If the full message gets longer than maximum UDP packet size, it will be cropped. Splitting and merging
     * of long messages is not yet implemented here.
     * 
     * @param validCommands
     *            The list of currently possible commands the external client can choose from.
     * @param textMessage
     *            The text message explaining the list of valid commands.
     * @return Returns the received command from the external client. Returns UserCmd.NONE in case of socket issues or
     *         when if waiting for an answer was canceled.
     */
    public UserCmd getExternalCommand(List<UserCmd> validCommands, String textMessage)
    {
        _cancelGetExternalCommand = false;

        if (null == _externalClientAddress)
        {
            _logger.info("No external client is known yet. Waiting to receive a first ping message.");
            while (null == _externalClientAddress)
            {
                // wait till a valid packet arrives, which will register _externalClientAddress
                ThreadUtil.milliSleep(50);

                if (_cancelGetExternalCommand)
                {
                    return UserCmd.NONE;
                }
            }
            _logger.info("External client registered at: " + _externalClientAddress.getHostName() + ":"
                    + _externalClientAddress.getPort());
        }

        // do not send a ';' as it is the field separator of the UDP message
        textMessage.replaceAll(";", ",");

        // increase packet counter (the counter allows the receiver to ignore duplicate messages)
        ++_packetCounter;

        // create Get_Command message content
        String fullMsgString = new String(_packetIdentifier + ";" + GET_COMMAND + ";" + _packetCounter + ";");
        for (UserCmd cmd : validCommands)
        {
            fullMsgString = fullMsgString + cmd.toString() + ";";
        }
        fullMsgString = fullMsgString + textMessage;

        while (!_cancelGetExternalCommand)
        {
            // send a Get_Command message periodically
            try
            {
                byte[] msg = fullMsgString.getBytes(UTF_8);
                _socket.send(new DatagramPacket(msg, msg.length, _externalClientAddress));
            }
            catch (Exception e)
            {
                _logger.error("Failed to send UDP message (" + GET_COMMAND + ") due to exception: ", e);
                return UserCmd.NONE;
            }

            // check if a new command arrived until wait time is up or command is received
            long timeStarted = System.currentTimeMillis();
            while (System.currentTimeMillis() - timeStarted < TIME_TO_WAIT_FOR_ANSWER)
            {
                UserCmd answer = takeCommandAnswer();
                if (null != answer && validCommands.contains(answer))
                {
                    return answer;
                }
                if (null != answer)
                {
                    // if an answer message arrived with an answer not included in validCommands, send GET_COMMAND
                    // message again with increased packetCounter (so external client does not treat it as a duplicate)
                    ++_packetCounter;
                }
            }
        }

        _logger.info("Cancelling wait to receive external command.");
        return UserCmd.NONE;
    }

    /**
     * Call this method to interrupt the infinite wait for an external command.
     */
    public synchronized void cancelGetExternalCommand()
    {
        _cancelGetExternalCommand = true;
    }

    /**
     * Send a notification that the brake test monitor state changed to the external client, if an external client is
     * known. Otherwise the notification is discarded. The message is structured as e.g.
     * <p>
     * 'DefaultApplication;BrakeTestMonitorEvent;FEEDBACK_REQUIRED;ENOUGH_SPACE;true'
     * <p>
     * , where FEEDBACK_REQUIRED is the current monitor state, ENOUGH_SPACE says that enough disk space is available for
     * brake test (logging) and true says that a brake test postponement is currently acceptable.
     * 
     * @param event
     *            The brake test monitor event triggered by the state change.
     */
    public void sendBrakeTestMonitorStateChanged(BrakeTestMonitorEvent event)
    {
        if (null == _externalClientAddress)
        {
            // no external client address registered yet, discarding message
            return;
        }

        String fullMsgString = new String(_packetIdentifier + ";" + BRAKE_TEST_MONITOR_EVENT + ";"
                + event.getCurrentCyclicMonitorState() + ";"
                + event.getStateOfFreeDiskSpace() + ";"
                + event.isPostponementAcceptable());

        try
        {
            byte[] msg = fullMsgString.getBytes(UTF_8);
            synchronized (this)
            {
                _socket.send(new DatagramPacket(msg, msg.length, _externalClientAddress));
            }
        }
        catch (Exception e)
        {
            _logger.error("Failed sending UDP (" + BRAKE_TEST_MONITOR_EVENT + ") message: ", e);
        }
    }

    /**
     * Use this method to pass an instance of a BrakeTestHelper, to allow this UdpHelper to postpone BrakeTests if a UDP
     * message with "Postpone_Braketest;true" is received.
     * 
     * @param brakeTestHelper
     *            Instance of BrakeTestHalper to command with brake test postponement.
     */
    public void setBrakeTestHelper(BrakeTestHelper brakeTestHelper)
    {
        _brakeTestHelper = brakeTestHelper;
    }

    /**
     * Returns the last command that was received in a valid 'Command' message via UDP. Internally it will delete this
     * command by resetting it to null so each command can only be taken once.
     * 
     * @return last received command
     */
    private synchronized UserCmd takeCommandAnswer()
    {
        UserCmd currentCommand = _commandAnswer;

        _commandAnswer = null;

        return currentCommand;
    }

    /**
     * Returns the time when the last heart beat messages was received via UDP.
     * 
     * @return last received command
     */
    public long getTimeOfLastHeartbeat()
    {
        return _timeOfLastHeartbeat;
    }

    /**
     * This method allows this UdpHelper to run in a separate thread for receiving UDP messages. This is necessary as
     * the receive() command of a socket is a blocking call and would halt the main thread while waiting for a message.
     * 
     * This is only needed to handle asynchronously incoming messages such as the postponement of a brake test, which
     * command could come in any time.
     */
    @Override
    public void run()
    {
        String[] receivedMessage;

        while (true)
        {
            receivedMessage = new String[0];

            try
            {
                receivedMessage = receive();
                
            }
            catch (IOException e)
            {
                // an IOException 'socket closed' is normal, when dispose() was called 
                _logger.warn("Quitting receive thread while waiting to receive UDP package because of "
                        + e.getMessage());
                break;
            }

        }
    }

    /**
     * Call this if the instance of this class is not needed anymore.
     */
    public void dispose()
    {
        if (_socket != null)
        {
            _socket.close();
        }
    }

}
