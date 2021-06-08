package shared;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import main.Robert_2_3;
import com.kuka.common.ThreadUtil;
import com.kuka.task.ITaskLogger;

public class TCPConnection implements Runnable {

	private OutputStream os;
	private ITaskLogger logger;
	private InputStream is;
	private Socket socket;
	private ServerSocket serverSocket;
	private boolean stopRunning;

	private int time;

	public boolean initTCP() { 
		stopRunning = false;
		
		if (serverSocket == null) {
			try {
				
				serverSocket = new ServerSocket(30007, 10); 
				//serverSocket.setSoTimeout(SOCKET_TIMEOUT);
				socket = serverSocket.accept();
				is = socket.getInputStream();
				os = socket.getOutputStream();				
			} catch (Exception e) {
				this.logger.error("Problems initializing TCP socket: ", e);
				return false;
			}
		}
		
		return true;

	}

	public TCPConnection(ITaskLogger _logger) {
		this.logger = _logger;
	}

	@Override
	public void run() {
		time = 0;
		// Receiving
		while (!stopRunning) {

			byte[] lenBytes = new byte[4];

			try {
				is.read(lenBytes, 0, 4);
			} catch (IOException e) {
				try {

					serverSocket.close();
					ThreadUtil.milliSleep(100);
					time = time + 100;
					this.initTCP();
					if (time >= 5000) {
						break;
					}

				} catch (IOException e1) {

					this.logger.info("ROBERT: Failed to close and call initTCP socket: " + e);					
				}
				
				this.logger.info("ROBERT: Failed to read the msg from the socket: " + e);
			}

			int len = (((lenBytes[3] & 0xff) << 24) | ((lenBytes[2] & 0xff) << 16) | ((lenBytes[1] & 0xff) << 8) | (lenBytes[0] & 0xff));
			byte[] receivedBytes = new byte[len];

			try {
				is.read(receivedBytes, 0, len);
			} catch (IOException e) {
				this.logger.error("failed to read the message length from the socket: ", e);
			}

			String received = new String(receivedBytes, 0, len);

			if (!received.equals("")) {

					logger.info("ROBERT: the RMU received from the CIU: " + received); 

					/**
					 * set the received message to be handled in the application
					 */
					Robert_2_3.cmdMsg = received;

					received = "";		
			}

		}

		ThreadUtil.milliSleep(100);
	}


	/**
	 * This method sends a message to the control interface
	 * 
	 * @param message
	 * @throws IOException
	 */
	public void sendMessage(String message) throws IOException {
		byte[] toSendBytes = message.getBytes();
		int toSendLen = toSendBytes.length;
		byte[] toSendLenBytes = new byte[4];
		toSendLenBytes[0] = (byte) (toSendLen & 0xff);
		toSendLenBytes[1] = (byte) ((toSendLen >> 8) & 0xff);
		toSendLenBytes[2] = (byte) ((toSendLen >> 16) & 0xff);
		toSendLenBytes[3] = (byte) ((toSendLen >> 24) & 0xff);
		byte[] byteMessage = new byte[toSendLenBytes.length + toSendBytes.length];
		System.arraycopy(toSendLenBytes, 0, byteMessage, 0, toSendLenBytes.length);
		System.arraycopy(toSendBytes, 0, byteMessage, toSendLenBytes.length, toSendBytes.length);		
		os.write(byteMessage);
		logger.info("ROBERT: the RMU sent to the CIU : " + message);
	}

	public void setExecutionController(ExcutionController _excutionController) {
	}

	/**
	 * closing the socked and the server socket if something went wrong.
	 * 
	 * @throws Exception
	 */
	public void terminate() {
		if (serverSocket != null) {
			try {

				is.close();
				os.close();
				if (socket != null) {
					socket.close();
				}
				serverSocket.close();

				logger.info("Closed the socket");
			} catch (IOException e) {
				logger.info("failed to close the socket: " + e);
			}
		}

		stopRunning = true;
	}
}
