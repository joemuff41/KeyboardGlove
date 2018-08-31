
public class LetterPeak {

	private char letter;
	private float peak;
	
	public LetterPeak(char aLetter, float aPeak) {
		letter = aLetter;
		peak = aPeak;
	}
	
	public LetterPeak() {
		letter = 0;
		peak = 0;
	}
	
	public void setLetter(char aLetter) {
		letter = aLetter;
	}
	
	public void setPeak(float aPeak) {
		peak = aPeak;
	}
	
	public char getLetter() {
		return letter;
	}
	
	public float getPeak() {
		return peak;
	}
}
