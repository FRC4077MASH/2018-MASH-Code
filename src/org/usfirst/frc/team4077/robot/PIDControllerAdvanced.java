package org.usfirst.frc.team4077.robot;

public class PIDControllerAdvanced {
	private long lastTime;
	private double input, output, setpoint;
	private double outputSum, lastInput;
	private double kp, ki, kd;
	private long sampleTime;
	private double outMin, outMax;
	private boolean inAuto = false;

	public final static int MANUAL = 0;
	public final static int AUTOMATIC = 1;

	public final static int DIRECT = 0;
	public final static int REVERSE = 1;
	private int controllerDirection = DIRECT;

	public final static int P_ON_M = 0;
	public final static int P_ON_E = 1;
	private boolean pOnE = true;
	
	public PIDControllerAdvanced(int controllerDirection, long sampleTime, double outMin, double outMax, int mode) {
		setOutputLimits(outMin, outMax);
		setSampleTime(sampleTime);
		setMode(mode);		
		setControllerDirection(controllerDirection);
	}

	public double compute(double inputVal) {
		input = inputVal;
		if (!inAuto) {
			return output;
		}

		long now = System.currentTimeMillis();
		long timeChange = (now - lastTime);

		if (timeChange >= sampleTime) {
			double error = setpoint - input;
			double dInput = (input - lastInput);
			outputSum += (ki * error);

			if (!pOnE) {
				outputSum -= kp * dInput;
			}

			if (outputSum > outMax) {
				outputSum = outMax;
			} else if (outputSum < outMin) {
				outputSum = outMin;
			}

			if (pOnE) {
				output = kp * error;
			} else {
				output = 0;
			}

			output += outputSum - kd * dInput;
			if (output > outMax) {
				output = outMax;
			} else if (output < outMin) {
				output = outMin;
			}
			
			lastInput = input;
			lastTime = now;
		}
		
		return output;
	}

	public void setTunings(double Kp, double Ki, double Kd, int pOn) {
		if (Kp < 0 || Ki < 0 || Kd < 0) {
			return;
		}

		pOnE = pOn == P_ON_E;

		double sampleTimeInSec = ((double) sampleTime) / 1000;
		kp = Kp;
		ki = Ki * sampleTimeInSec;
		kd = Kd / sampleTimeInSec;

		if (controllerDirection == REVERSE) {
			kp = -kp;
			ki = -ki;
			kd = -kd;
		}
	}

	public void setSampleTime(long newSampleTime) {
		if (newSampleTime > 0) {
			double ratio = (double) newSampleTime / (double) sampleTime;

			ki *= ratio;
			kd /= ratio;
			sampleTime = newSampleTime;
		}
	}

	public void setOutputLimits(double min, double max) {
		if (min > max) {
			return;
		}

		outMin = min;
		outMax = max;

		if (output > outMax) {
			output = outMax;
		} else if (output < outMin) {
			output = outMin;
		}
	}

	public void setMode(int mode) {
		boolean newAuto = mode == AUTOMATIC;

		if (newAuto == !inAuto) {
			initialize();
		}
		inAuto = newAuto;
	}

	public void initialize() {
		lastInput = input;

		outputSum = output;
		if (outputSum > outMax) {
			outputSum = outMax;
		} else if (outputSum < outMin) {
			outputSum = outMin;
		}
	}

	public void setControllerDirection(int direcion) {
		controllerDirection = direcion;
	}
	
	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
	}
	
	//Gets
	public double getKp() {
		return kp;
	}
	
	public double getKi() {
		return ki;
	}
	
	public double getKd() {
		return kd;
	}
	
	public double getSetpoint() {
		return setpoint;
	}
	
	public int getMode() {
			return inAuto ? AUTOMATIC : MANUAL;
	}
	
	public int getDirection() {
		return controllerDirection;
	}
	
}