
public class GloveKeyboard {
	
	
	static final int DELAY = 10;
	static final int NUM_IMUS = 3;
	static final char[][] fingerLetters = new char[][] {
		{'q', 'w', 'e', 'r'}, 
		{'a', 's', 'd', 'f'}, 
		{'z', 'x', 'c', 'v'}
	};
	static final int START_IMU = 1;
	
	SerialReader reader;
	
	
	float[] accTiltOffsets;
	float[] accOffsets;
	float[][] gyroOffsets;
	float[][] theta;
	float[] lastTheta;
	float[][] deltaTheta;
	float[] accTilt;
	float[] accSumSquareChange;
	float[] accSumSquare;
	float[] totalAcc;
	float[] steadyChange;
	int[] gyroState;
	int[] accState;
	int[] countRepeats;
	float[] gyroPeak;
	float[] gyroValley;
	float[] accPeak;
	int[] sequence;				//Set to 1 if going from rising to steady, set to 2 if falling and it is 1 (supposed to catch a gyroPeak)
	int countSinceLastGyroLetter;
	int countSinceLastAccLetter;
	char lastAccLetter;
	char[][] calibrationLetters;
	float[][] setPointsGyro;
	float[][] setPointsAcc;
	int t;
	LetterPeak[] gyroLettersFound;
	LetterPeak[] accLettersFound;
	
	IMUReadingSixDOF[] reading;
	private boolean gyroChange[];
	private boolean accChange[];
	
	
	public GloveKeyboard() {
		reader = new SerialReader();
		reader.connectPort("COM3");
		
		reader.toNextStart();
		reader.toNextStart();
		reader.toNextStart();
		reader.toNextStart();
		reader.toNextStart();
		
		accTiltOffsets = new float[NUM_IMUS];
		accOffsets = new float[NUM_IMUS];
		gyroOffsets = new float[3][NUM_IMUS];
		theta = new float[3][NUM_IMUS];
		lastTheta = new float[NUM_IMUS];
		deltaTheta = new float[3][NUM_IMUS];
		accTilt = new float[NUM_IMUS];
		accSumSquare = new float[NUM_IMUS];
		accSumSquareChange = new float[NUM_IMUS];
		totalAcc = new float[NUM_IMUS];
		steadyChange = new float[NUM_IMUS];
		gyroState = new int[NUM_IMUS];
		accState = new int[NUM_IMUS];
		countRepeats = new int[NUM_IMUS];
		gyroPeak = new float[NUM_IMUS];
		gyroValley = new float[NUM_IMUS];
		accPeak = new float[NUM_IMUS];
		sequence = new int[NUM_IMUS];
		gyroChange = new boolean[NUM_IMUS];
		accChange = new boolean[NUM_IMUS];
		calibrationLetters = new char[2][NUM_IMUS];
		setPointsGyro = new float[2][NUM_IMUS];
		setPointsAcc = new float[2][NUM_IMUS];
		gyroLettersFound = new LetterPeak[NUM_IMUS];
		accLettersFound = new LetterPeak[NUM_IMUS];
		
		for (int i = 0; i < NUM_IMUS; i++) {
			for (int j = 0; j < 3; j++) {
				theta[j][i] = 0;
				deltaTheta[j][i] = 0;

			}
			lastTheta[i] = 0;
			accTilt[i] = 0;
			accSumSquare[i] = 0;
			accSumSquareChange[i] = 0;
			totalAcc[i] = 0;
			steadyChange[i] = 0;
			gyroState[i] = 0;
			accState[i] = 0;
			countRepeats[i] = 0;
			gyroPeak[i] = 0;
			gyroValley[i] = 0;
			accPeak[i] = 0;
			sequence[i] = 0;
			gyroChange[i] = false;
			accChange[i] = false;
			gyroLettersFound[i] = new LetterPeak();
			accLettersFound[i] = new LetterPeak();
			
		}
		
		countSinceLastGyroLetter = 0;
		countSinceLastAccLetter = 0;
		lastAccLetter = 0;
		t = 0;
		
	}
	
	public void establishReadings() {
		while (reader.readMessage() == null) {

			System.out.println("Closing port");
			reader.disconnectPort("COM3");
			try {
				Thread.sleep(200);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			System.out.println("Opening port");
			reader.connectPort("COM3");
		}
	}
	
	public void calibrate() {
		int calTime = 3;
		int throwAwayTime = 5;
		System.out.print("Calibrating...");
		
		//Throw away the first period of time to allow to settle
		for (int i = throwAwayTime; i > 0; i--) {
			System.out.print(i + calTime);
			System.out.print("...");
			
			for (int j = 0; j < (1000 / DELAY); j++) {
				reader.readMessage();
			}
		}
		
		float[][] sumGyroReadings = new float[3][NUM_IMUS];
		float[] sumAccReadings = new float[NUM_IMUS];
		
		for (int i = calTime; i> 0; i--) {
			System.out.print(i + "...");
			
			for (int j = 0; j < (1000 / DELAY); j++) {
				reading = reader.readMessage();
				for (int k = 0; k < NUM_IMUS; k++) {
					
					accOffsets[k] += (float)
							Math.sqrt(reading[k].getValue(3) * reading[k].getValue(3) + reading[k].getValue(4) * reading[k].getValue(4) + reading[k].getValue(5) * reading[k].getValue(5));
					sumAccReadings[k] += (float) (Math.atan2(reading[k].getValue(4), 
							reading[k].getValue(5)) * (180 / Math.PI));
					sumGyroReadings[0][k] += reading[k].getValue(0);
					sumGyroReadings[1][k] += reading[k].getValue(1);
					sumGyroReadings[2][k] += reading[k].getValue(2);
				}
			}
		}
		
		System.out.println("");
		
		for (int i = 0; i < NUM_IMUS; i++) {
			
			accOffsets[i] = accOffsets[i] / ((1000 / DELAY) * calTime);
			accTiltOffsets[i] = sumAccReadings[i] / ((1000 / DELAY) * calTime);
			
			gyroOffsets[0][i] = sumGyroReadings[0][i] / ((1000 / DELAY) * calTime);
			gyroOffsets[1][i] = sumGyroReadings[1][i] / ((1000 / DELAY) * calTime);
			gyroOffsets[2][i] = sumGyroReadings[2][i] / ((1000 / DELAY) * calTime);
			
		}
		
	}
	
	public void learnSetPoints() {
		boolean done = false;
		int gyroLetters = 0;
		int accLetters = 0;
		float[][] letterPeaksGyro = new float[3][NUM_IMUS];
		float[][] letterPeaksAcc = new float[3][NUM_IMUS];
		
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < NUM_IMUS; j++) {
				letterPeaksGyro[i][j] = 0;
				letterPeaksAcc[i][j] = 0;
			}
		}
		
		int numPoints = 3;
		
		for (int i = START_IMU; i < NUM_IMUS; i++) {
			done = false;
			gyroLetters = 0;
			accLetters = 0;
			System.out.println("Type " + fingerLetters[1][i] + " " + numPoints + " times");
			while (!done) {
				calibrationLoopBody(i);
				if (gyroLetters < 3 && calibrationLetters[0][i] != 0) {
					gyroLetters++;
					letterPeaksGyro[1][i] += calibrationPeakGyro[i];
				}
				if (accLetters < 3 && calibrationLetters[1][i] != 0) {
					accLetters++;
					letterPeaksAcc[1][i] += calibrationPeakAcc[i];
				}
				
				if (gyroLetters >= 3 && accLetters >= 3) {
					done = true;
					letterPeaksGyro[1][i] = (float) (letterPeaksGyro[1][i] / 3.0);
					letterPeaksAcc[1][i] = (float) (letterPeaksAcc[1][i] / 3.0);
				}
			}
			
			done = false;
			gyroLetters = 0;
			accLetters = 0;
			System.out.println("Type " + fingerLetters[0][i] + " " + numPoints + " times");
			while (!done) {
				calibrationLoopBody(i);
				if (gyroLetters < 3 && calibrationLetters[0][i] != 0) {
					gyroLetters++;
					letterPeaksGyro[0][i] += calibrationPeakGyro[i];
				}
				if (accLetters < 3 && calibrationLetters[1][i] != 0) {
					accLetters++;
					letterPeaksAcc[0][i] += calibrationPeakAcc[i];
				}
				
				if (gyroLetters >= 3 && accLetters >= 3) {
					done = true;
					letterPeaksGyro[0][i] = (float) (letterPeaksGyro[0][i] / 3.0);
					letterPeaksAcc[0][i] = (float) (letterPeaksAcc[0][i] / 3.0);
				}
			}
			
			done = false;
			gyroLetters = 0;
			accLetters = 0;
			done = false;
			System.out.println("Type " + fingerLetters[2][i] + " " + numPoints + " times");
			while (!done) {
				calibrationLoopBody(i);
				if (gyroLetters < 3 && calibrationLetters[0][i] != 0) {
					gyroLetters++;
					letterPeaksGyro[2][i] += calibrationPeakGyro[i];
				}
				if (accLetters < 3 && calibrationLetters[1][i] != 0) {
					accLetters++;
					letterPeaksAcc[2][i] += calibrationPeakAcc[i];
				}
				
				if (gyroLetters >= 3 && accLetters >= 3) {
					done = true;
					letterPeaksGyro[2][i] = (float) (letterPeaksGyro[2][i] / 3.0);
					letterPeaksAcc[2][i] = (float) (letterPeaksAcc[2][i] / 3.0);
				}
			}
			
			setPointsGyro[0][i] = (float) ((letterPeaksGyro[0][i] + letterPeaksGyro[1][i]) * 0.42);
			setPointsGyro[1][i] = (float) ((letterPeaksGyro[1][i] + letterPeaksGyro[2][i]) * 0.58);
			
			setPointsAcc[0][i] = (float) ((letterPeaksAcc[0][i] + letterPeaksAcc[1][i]) * 0.42);
			setPointsAcc[1][i] = (float) ((letterPeaksAcc[1][i] + letterPeaksAcc[2][i]) * 0.58);
		}

	}
	
	private void calibrationLoopBody(int imu) {

		reading = reader.readMessage();
		processGyroData();
		processAccData();
		sumSquareAcc();
		complementaryFilter();
			
		calibrationLetters[0][imu] = analyzeKeyboardGyro(imu)[imu];
		calibrationLetters[1][imu] = analyzeKeyboardAcc(imu)[imu];
		
		//System.out.println(theta[0][imu]);

	}
	
	public void start() {
		while (true) {
			
			reading = reader.readMessage();
			processGyroData();
			processAccData();
			sumSquareAcc();
			complementaryFilter();
			char letterGyro = analyzeKeyboardGyro(-1);
			char letterAcc = analyzeKeyboardAcc(-1);
			
			if (letterGyro !=0 && countSinceLastAccLetter > 10) {
				//System.out.print(letterGyro);
				countSinceLastGyroLetter = 0;
			}
			else {
				countSinceLastGyroLetter++;
			}
			
			if (letterAcc != 0 && countSinceLastGyroLetter > 50) {
				if (countSinceLastAccLetter >= 10 || letterAcc != lastAccLetter) {
					//System.out.print(letterAcc);
					countSinceLastAccLetter = 0;
					lastAccLetter = letterAcc;
				}
				else {
					//System.out.println("Double acc letter");
				}
			}
			else {
				countSinceLastAccLetter++;
			}
			
			t++;
			//System.out.println(t + "\t" + theta[0][2] + "\t" + accSumSquareChange[2]);
			//System.out.println(t + "\t" + theta[0][1] + "\t" + theta[0][2] + "\t" + accSumSquareChange[1] + "\t" + accSumSquareChange[2]);
		}
	}
	
	private int analyzeKeyboardGyro(int imu) {
		
		int foundIMUS = 0;
		
		int startingIndex;
		int endingIndex;
		
		//A non-negative imu value indicates that the setpoints are being found
		if (imu >= 0) {
			startingIndex = imu;
			endingIndex = imu + 1;
		}
		else {
			startingIndex = 0;
			endingIndex = NUM_IMUS;
		}
		
		for (int i = startingIndex; i < endingIndex; i++) {
			gyroChange[i] = false;
			
			//Rising
			if ((theta[0][i] - lastTheta[i]) > 0.5) {
				keyboardRising(i);
			}
			//Falling
			else if ((lastTheta[i] - theta[0][i]) > 0.5) {
				keyboardFalling(i);
			}
			else {
				steadyChange[i] += theta[0][i] - lastTheta[i];
				if (steadyChange[i] > 0.5) {
					keyboardRising(i);
				}
				else if (steadyChange[i] < -0.5) {
					keyboardFalling(i);
				}
				else {
				
					//If going from falling to static
					if (gyroState[i] == 2) {
						lastTheta[i] = theta[0][i];
					}
					//If going from rising to static, set the next value to the gyroPeak
					//This is a letter indication
					else if (gyroState[i] == 1) {
						gyroChange[i] = true;
						gyroPeak[i] = theta[0][i];
						lastTheta[i] = theta[0][i];
						sequence[i] = 1;
					}
				}
				gyroState[i] = 0;
			}
			
			if (sequence[i] == 2) {
				sequence[i] = 0;
				
				if ((gyroPeak[i] - gyroValley[i]) > 2.5) {
					if (gyroState[i] == 1) {
						gyroValley[i] = lastTheta[i];
					}

					if (imu >= 0) {
						gyroLettersFound[i].setLetter('!');
						gyroLettersFound[i].setPeak(gyroPeak[i]);
						foundIMUS = foundIMUS * 10;
						foundIMUS = foundIMUS + i + 1;
						return foundIMUS;
					}
					else {
						if (gyroPeak[i] > setPointsGyro[1][i]) {
							gyroLettersFound[i].setLetter(fingerLetters[0][i]);
							gyroLettersFound[i].setPeak(gyroPeak[i]);
						}
						else if (gyroPeak[i] > setPointsGyro[0][i]) {
							gyroLettersFound[i].setLetter(fingerLetters[1][i]);
						}
						else {
							gyroLettersFound[i].setLetter(fingerLetters[2][i]);
						}
						gyroLettersFound[i].setPeak(gyroPeak[i]);
						foundIMUS = foundIMUS * 10;
						foundIMUS = foundIMUS + i + 1;
					}
					
					gyroValley[i] = gyroPeak[i];
				}
			}
		}

		
		return foundIMUS;
	}
	
	private void keyboardRising(int imu) {
		steadyChange[imu] = 0;
		countRepeats[imu] = 0;
		
		//Only count the angle if not going from falling to rising
		if (gyroState[imu] != 2) {
			lastTheta[imu] = theta[0][imu];
		}
		
		gyroState[imu] = 1;
	}
	
	private void keyboardFalling(int imu) {
		steadyChange[imu] = 0;
		countRepeats[imu] = 0;
		
		//If there was a quick rise in angle then dropping down to the actual gyroPeak (noise), make the gyroPeak the next value
		//This also should be a letter indication
		if (gyroState[imu] == 1) {
			gyroPeak[imu] = theta[0][imu];
		}
		else if (gyroState[imu] == 2) {
			//Only set the gyroValley if it was previously dropping
			gyroValley[imu] = theta[0][imu];
		}
		
		if (sequence[imu] == 1) {
			sequence[imu] = 2;
		}
		lastTheta[imu] = theta[0][imu];
		gyroState[imu] = 2;
	}
	
	/**
	 * Integrate the gyro data over the delay time period to get the angle change
	 */
	private void processGyroData() {
		
		for (int i = 0; i < NUM_IMUS; i++) {
			deltaTheta[0][i] = (reading[i].getValue(0) - gyroOffsets[0][i]) * ((float) DELAY / 1000);
			deltaTheta[1][i] = (reading[i].getValue(1) - gyroOffsets[1][i]) * ((float) DELAY / 1000);
			deltaTheta[2][i] = (reading[i].getValue(2) - gyroOffsets[2][i]) * ((float) DELAY / 1000);
		}
	}
	
	/**
	 * Calculate the tilt angle from the g-force
	 */
	private void processAccData() {
		for (int i = 0; i < NUM_IMUS; i++) {
			accTilt[i] = (float) ((1 * (Math.atan2(reading[i].getValue(4), reading[i].getValue(5)))) * (180 / Math.PI) - accTiltOffsets[i]);
		}
	}
	
	/**
	 * Square the total acceleration and keep a running total
	 * Squaring the total acceleration is a high pass filter and lets us keep track of key "presses"
	 */
	private void sumSquareAcc() {
		for (int i = 0; i < NUM_IMUS; i++) {
			
			totalAcc[i] = (float)
					Math.sqrt(reading[i].getValue(3) * reading[i].getValue(3) + reading[i].getValue(4) * reading[i].getValue(4) + reading[i].getValue(5) * reading[i].getValue(5));
		
			accSumSquareChange[i] = accSumSquare[i];
			accSumSquare[i] += ((totalAcc[i] - accOffsets[i]) * (totalAcc[i] - accOffsets[i]));
			accSumSquareChange[i] = accSumSquare[i] - accSumSquareChange[i];
		}
	}
	
	private void complementaryFilter() {

		for (int i = 0; i < NUM_IMUS; i++) {
			
			if (totalAcc[i] > 0.5 && totalAcc[i] < 2.0) {
				theta[0][i] = (float) (0.97 * (theta[0][i] + deltaTheta[0][i]) + 0.03 * accTilt[i]);
				theta[1][i] = (float) (1.00 * (theta[1][i] + deltaTheta[1][i]));
				theta[2][i] = (float) (1.00 * (theta[2][i] + deltaTheta[2][i]));
			}
			else {
				theta[0][i] = (float) (1.00 * (theta[0][i] + deltaTheta[0][i]));
				theta[1][i] = (float) (1.00 * (theta[1][i] + deltaTheta[1][i]));
				theta[2][i] = (float) (1.00 * (theta[2][i] + deltaTheta[2][i]));
			}
		}
	}
	
	private int analyzeKeyboardAcc(int imu) {
		
		int foundIMUS = 0;
		
		int startingIndex;
		int endingIndex;
		
		if (imu >= 0) {
			startingIndex = imu;
			endingIndex = imu + 1;
		}
		else {
			startingIndex = 0;
			endingIndex = NUM_IMUS;
		}
		
		for (int i = startingIndex; i < endingIndex; i++) {
			if (accSumSquareChange[i] < 1) {
				if (accState[i] == 1) {
					countRepeats[i]++;
				
					if (countRepeats[i] == 3) {
						accChange[i] = true;
						accState[i] = 0;
						countRepeats[i] = 0;
					}
				}
			}
			else {
				accPeak[i] = theta[0][i];
				accState[i] = 1;
				countRepeats[i] = 0;
			}
			
			if (accChange[i]) {
				//Place holder for calibration
				if (imu >= 0) {
					accLettersFound[i].setLetter('!');
				}
				else {
					if (accPeak[i] > setPointsAcc[1][i]) {
						accLettersFound[i].setLetter(fingerLetters[0][i]);
					}
					else if (accPeak[2] > setPointsAcc[0][i]) {
						accLettersFound[i].setLetter(fingerLetters[1][i]);
					}
					else {
						accLettersFound[i].setLetter(fingerLetters[2][i]);
					}
					
				}
				accLettersFound[i].setPeak(accPeak[i]);
				foundIMUS = foundIMUS * 10;
				foundIMUS = foundIMUS + i + 1;
				
				accChange[i] = false;
			}
		}

		
		return foundIMUS;
	}
	
}
