import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Scanner;
import com.fazecast.jSerialComm.*;

public class SerialReader {

	int valBytes;
	int numMpus;
	int messageLength;
	int dimensionLength;
	int samplingLength;
	int characterOffset;
	
	int NUM_DIMENSIONS = 6;
	
	SerialPort ports[];
	SerialPort curPort;
	
	
	public SerialReader() {
		valBytes = 4;
		numMpus = 3;
		messageLength = 36;
		dimensionLength = 6;
		samplingLength = 2 * dimensionLength;
		characterOffset = 97;
		
		ports = SerialPort.getCommPorts();
	}
	
	public String[] getPorts() {

		String[] portNames = new String[ports.length];
		
		for (int i = 0; i < ports.length; i++) {
			portNames[i] = ports[i].getSystemPortName();
		}
		
		return  portNames;
	}
	
	
	public boolean connectPort(String aPortName) {
		
		for (SerialPort port : ports) {
			if (port.getSystemPortName().equalsIgnoreCase(aPortName)) {
				if (port.openPort()) {
					curPort = port;
					port.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, 0, 0);
					port.setBaudRate(115200);
					return true;
				}
				
			}
		}
		
		return false;
	}
	
	public boolean disconnectPort(String aPortName) {
		
		for (SerialPort port : ports) {
			if (port.getSystemPortName().equalsIgnoreCase(aPortName)) {
				if (port.closePort()) {
					return true;
				}
				
			}
		}
		
		return false;
	}
	
	public void toNextStart() {
		if (curPort == null) return;
		
		byte[] initByteArray = new byte[12];
		curPort.readBytes(initByteArray, initByteArray.length);
		int bytesToSkip = 0;
		
		for (int i = 0; i < dimensionLength; i++) {
			if (((char) initByteArray[i]) <= 'f' && ((char) initByteArray[i]) >= 'a') {
				if (initByteArray[i + dimensionLength] == (char)(initByteArray[i] + 1)) {
					int intValLetter = (((int) ((char) initByteArray[i])) - characterOffset) + 1;
					bytesToSkip = (numMpus * messageLength) - ((dimensionLength * intValLetter) 
							+ (samplingLength - (i + dimensionLength)) + (Character.getNumericValue(((char)initByteArray[i + 1])) * messageLength));
					break;
				}
				else if (initByteArray[i] == 'f' && initByteArray[i + 6] == 'a') {
					
				}
			}
		}
		
		byte[] readBuffer = new byte[messageLength * numMpus]; 
		curPort.readBytes(readBuffer, bytesToSkip);
	}
	
	public IMUReadingSixDOF[] readMessage() {
		
		byte[] byteArray = new byte[valBytes];
		byte[] readBuffer = new byte[messageLength * numMpus];
		curPort.readBytes(readBuffer, messageLength * numMpus);
		IMUReadingSixDOF[] readings = new IMUReadingSixDOF[numMpus];
		
		//Initialize reading objects
		for (int i = 0; i < numMpus; i++) {
			readings[i] = new IMUReadingSixDOF(i);
		}
		
		for (int i = 0; i <= (messageLength * numMpus - dimensionLength); i += dimensionLength) {
			
			for (int k = 0; k < valBytes; k++) {
				byteArray[k] = readBuffer[i + 2 + k];
			}

			float f = ByteBuffer.wrap(byteArray).order(ByteOrder.LITTLE_ENDIAN).getFloat();
			int imu = Character.getNumericValue((char)readBuffer[i + 1]);
			int dimension = ((int) ((char) readBuffer[i])) - characterOffset;
			
			try {
				readings[imu].setValue(dimension, f);
			}
			catch (Exception e) {
				return null;
			}

		}
		
		return readings;
	}
	
	
	
	
	
	
}
