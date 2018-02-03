package src.org.usfirst.frc157.Tekeva2018;

import edu.wpi.first.wpilibj.Timer;

public class PID {
	private double p;
	private double i;
	private double d;
	private double epsilonInner;
	private double epsilonOuter;
	private double lastTime;
	private double lastValue;
	private double sigma;
	private double dInner;
	private double dOuter;
	public PID (double p, double i, double d, double epsilonInner, double epsilonOuter, double dInner, double dOuter) {
		this.p = p;
		this.i = i;
		this.d = d;
		this.epsilonInner = epsilonInner;
		this.epsilonOuter = epsilonOuter;
		this.dInner = dInner;
		this.dOuter = dOuter;
		lastTime = Timer.getFPGATimestamp();
		lastValue = 0;
		sigma = 0;
	}
	public double pidCalculate(double target, double value) {
		double fDeltaTime = (double)(Timer.getFPGATimestamp() - lastTime) / 1000.0;
		lastTime = Timer.getFPGATimestamp();
		double fDeltaPV = 0;
		if(fDeltaTime > 0)
			fDeltaPV = (value - lastValue) / fDeltaTime;
		lastValue = value;

		double fError = target - value;

		if(Math.abs(fError) > epsilonInner && Math.abs(fError) < epsilonOuter)
			sigma += fError * fDeltaTime;

		if (Math.abs (fError) > epsilonOuter)
			sigma = 0;

		double fOutput = fError * p
		+ sigma * i
		- (((Math.abs(target) - Math.abs(fError) > 0 && Math.abs(fError) < dInner) || (Math.abs(target) - Math.abs(fError) < 0 && Math.abs(fError) < dOuter))? (fDeltaPV * d) : 0);

		fOutput = Math.abs(fOutput) > 1 ? 1 * fOutput/Math.abs(fOutput) : fOutput;
		return fOutput;
	}
}
