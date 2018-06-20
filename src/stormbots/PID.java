package stormbots;

public class PID {
	
	private double kp;
	private double ki;
	private double kd;
	private double pidI;
	private double pidPrevX;
	
	public PID(double d, double ki, double kd) {
		this.kp = d;
		this.ki = ki;
		this.kd = kd;
		this.pidI = 0;
		this.pidPrevX = 0;
	}
	
	public double nextValue(double x) {
		double p;
		double i;
		double d;
		p = x*this.kp;
		pidI = x+pidI;
		i = pidI*this.ki;
		d = (x-pidPrevX)*this.kd;
		pidPrevX = x;
		return p+i+d;
	}
	
	public void clear() {
		pidI = 0;
		pidPrevX = 0;
	}

}
