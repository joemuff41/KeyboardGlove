import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Scanner;

import com.fazecast.jSerialComm.*;
public class Main {
	
	

	public static void main(String[] args) {
		
		GloveKeyboard keyboard = new GloveKeyboard();
		keyboard.establishReadings();
		keyboard.calibrate();
		keyboard.learnSetPoints();
		keyboard.start();


		
	}
	

}


