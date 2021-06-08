package util;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import java.util.Arrays;
import java.util.List;

import javax.inject.Inject;

import activeMode.Active;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.BodyIOGroup;
import com.kuka.med.cyclicBrakeTest.BrakeTestMonitorEvent;
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
public class UdpHelper extends Thread
{	
    private ITaskLogger _logger;

    private static final String INFO = "Info";
    private static final String GET_COMMAND = "Get_Command";
    private static final String COMMAND = "Command";
    /**
     * Members for socket communication with the external GUI (e.g. ExampleMedGUI)
     */
    private InetSocketAddress _externalClientAddress = null;
    private DatagramSocket _socket;
    private int _localPort;
    private static final int SOCKET_TIMEOUT = 20;
    private static final int UDP_PACKET_SIZE = 508;
    private static final String UTF_8 = "UTF-8";
    private static final String PING = "Ping";
    private static final String PONG = "Pong"; 
    private static final String CIU_IP = "172.31.1.200";
    
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
     * the same answer
     */
    private int _packetCounter = 0;

    /**
     * Last command received from an external client. Should be reset to null if command was handled locally.
     */
    private UserCmd _commandAnswer = null;


    /**
     * Flag to cancel waiting for an external command.
     */
    private boolean _cancelGetExternalCommand = false;

	private BodyIOGroup bodyIOGroup;

 
    /**
     * Constructor.
     * 
     * @param localPort
     *            Local port to open and communicate over (Sunrise Cabinet typically allows 30000-30010)
     * @param packetIdentifier
     *            Unique string identifier for packages (to distinguish between different receivers)
     * @param logger
     *            The logger for text outputs.
     * @param _bodyIOGroup 
     */
    public UdpHelper(int localPort, String packetIdentifier, ITaskLogger logger, BodyIOGroup _bodyIOGroup)
    {
        _localPort = localPort;
        _packetIdentifier = packetIdentifier;
        _logger = logger;
        this.bodyIOGroup = _bodyIOGroup;
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
        //
        // _logger.info("SAS: received message : " + message);
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
        
        if (message[1].equals("STATUS")) {
			
        	sendStatusVariablesBack();
        	
        	return message;
		}
        // New for software 3.0
        if (message[1].equals("WEIGHT")) {
			
        	sendWeightBack();
        	
        	return message;
		}
        _logger.error("Received incorrect formated UDP message: " + new String(receivedPacket.getData()).trim());
        
        return new String[0];
    }
     
    private void sendStatusVariablesBack(){
    	String fullMsgString = new String(Math.abs(Active.isVelocity) + ";" + Active.hasReachedEnd + ";" + this.bodyIOGroup.getPlayPauseButton());

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
    
    private void sendWeightBack(){
    	//_bodyIOGroup.getPlayPauseButton();
    	String fullMsgString = new String(Math.abs(Active.isVelocity) + ";" + Active.hasReachedEnd + ";" + this.bodyIOGroup.getPlayPauseButton());

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
        String fullMsgString = new String(PONG + ";" + packetCounter);

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
    	 _externalClientAddress = new InetSocketAddress(CIU_IP, _localPort);
        

        try
        {
            byte[] messageTextBytes = messageText.getBytes(UTF_8);

            // info messages longer than max. UDP packet size need to be split
            int bytesAvailableForText = UDP_PACKET_SIZE;
            for (int i = 0; i <= messageTextBytes.length; i += bytesAvailableForText)
            {
                int textSegmentStart = i;
                int textSegmentEnd = i + bytesAvailableForText;
                if (textSegmentEnd > messageTextBytes.length)
                {
                    textSegmentEnd = messageTextBytes.length;
                }
                byte[] textSegment = Arrays.copyOfRange(messageTextBytes, textSegmentStart, textSegmentEnd);

                synchronized (this)
                {
                    _socket.send(new DatagramPacket(textSegment, textSegment.length, _externalClientAddress));
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
     * This method allows this UdpHelper to run in a separate thread for receiving UDP messages. This is necessary as
     * the receive() command of a socket is a blocking call and would halt the main thread while waiting for a message.
     * 
     * This is only needed to handle asynchronously incoming messages such as the postponement of a brake test, which
     * command could come in any time.
     */
    @Override
    public void run()
    {

        while (true)
        {

            try
            {
                receive();
                
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
