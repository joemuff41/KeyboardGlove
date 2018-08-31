
public class IMUReadingSixDOF {

	static final int DOF = 6;
	
	int imu;
	float values[];
	
	public IMUReadingSixDOF (int aIMU) {
		imu = aIMU;
		values = new float[DOF];
		
		for (int i = 0; i < DOF; i++) {
			values[i] = 0;
		}
	}
	
	public int getIMU() {
		return imu;
	}
	
	public void setValue(int dimension, float aValue) {
		values[dimension] = aValue;
	}
	
	public float getValue(int dimension) {
		return values[dimension];
	}
	
	public String toString() {
		String imuString = imu + "{";
		for (int i = 0; i < DOF - 1; i++) {
			imuString = imuString + String.format("%.3f", values[i]) + "\t";
		}
		imuString = imuString + String.format("%.3f", values[DOF - 1]) + "}";
		
		return imuString;
	}
}