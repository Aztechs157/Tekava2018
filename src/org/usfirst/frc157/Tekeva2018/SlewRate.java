package org.usfirst.frc157.Tekeva2018;

import edu.wpi.first.wpilibj.Timer;

public class SlewRate {
	
	private double initTime;
	private double lastRate;
	private double maxAccel;
	
	public SlewRate(double maxAccel) {
		initTime = Timer.getFPGATimestamp();
		this.maxAccel = maxAccel;
		lastRate = 0;
	}
	public double rateCalculate(double desired) {
		double absDesired = Math.abs(desired);
		double deltaTime = Timer.getFPGATimestamp();
		double desiredAccel = (absDesired - lastRate)/deltaTime;
		double addedRate;
		double newRate;
		if(desiredAccel>maxAccel) { 
			addedRate = maxAccel*deltaTime;
			newRate = addedRate+lastRate;
		}
		else {
			newRate = desired;
		}
		lastRate = newRate;
		return ((desired>=0)? 1: -1)*newRate;
	}
	public void reinit() {
		initTime = Timer.getFPGATimestamp();
		lastRate = 0;
	}
}
