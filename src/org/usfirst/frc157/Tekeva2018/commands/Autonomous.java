package org.usfirst.frc157.Tekeva2018.commands;

import org.usfirst.frc157.Tekeva2018.PID;
import org.usfirst.frc157.Tekeva2018.Robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class Autonomous extends Command
{

    public enum autonState 
    {
        driveForward, turnRight;
    }

    private double startTime;
    private autonState state;
    private PID drivePID;
    private PID gyroDrivePID;
    private PID gyroPID;
    private double drivePower = 0;
    private double leftPower = 0;
    private double rightPower = 0;
    private double initAngle = 0;
    private int repsAtTarget = 0;
    private double a = 0;
    private double b = 0;
    private double c = 0;
    private double x = 0;
    private double y = 0;
    private double angle = 0;

    public Autonomous()
    {
        requires(Robot.drive);
        startTime = Timer.getFPGATimestamp();
        state = autonState.driveForward;
        drivePID = new PID(0.16, 0, 0.000005, 999999, 99999, 999999, 9999999);
        gyroDrivePID = new PID(0.05, 0, 0, 999999, 99999, 999999, 9999999);
        gyroPID = new PID(0.01, 0, 0.000002, 9999999, 9999999, 9999999, 999999);
    }

    @Override
    protected void execute()
    {
        switch (state)
        {

            case driveForward:
                double encoder = -Robot.drive.getRightEncoder();

                drivePower = drivePID.pidCalculate(2500, encoder);

                x = xEllipseCalculate(3000, 3000, encoder);
                y = yEllipseCalculate(3000, 3000, x);
                angle = angleEllipseCalculate(3000, 3000, x);

                leftPower = drivePower - gyroDrivePID.pidCalculate(angle + initAngle, Robot.drive.getAngle());
                leftPower = ((leftPower > 0) ? 1 : -1) * Math.min(1, Math.abs(leftPower));

                rightPower = drivePower + gyroDrivePID.pidCalculate(angle + initAngle, Robot.drive.getAngle());
                rightPower = ((rightPower > 0) ? 1 : -1) * Math.min(1, Math.abs(rightPower));

                Robot.drive.AutoDrive(leftPower, rightPower);
                if (Math.abs(encoder - 2500) < 3.0)
                {
                    repsAtTarget++;
                    if (repsAtTarget >= 10)
                    {
                        state = autonState.turnRight;
                        repsAtTarget = 0;
                    }
                }
                else
                {
                    repsAtTarget = 0;
                }
                break;

            case turnRight:
                drivePower = gyroPID.pidCalculate(90, Robot.drive.getAngle());
                System.out.println("Angle: " + Robot.drive.getAngle() + "\nPower: " + drivePower);
                Robot.drive.AutoDrive(-drivePower, drivePower);
                if (Math.abs(Robot.drive.getAngle() - 90) < 2.0)
                {
                    repsAtTarget++;
                    if (repsAtTarget >= 10)
                    {
                        state = autonState.turnRight;
                    }
                }
                else
                {
                    repsAtTarget = 0;
                }
                break;

        }
    }

    public double xCalculate(double a, double b, double distance) {
		double sum = 0;
		double curX = 0;
		double calcSlice = 0;
		double deltaX = 0.05;
		while (Math.abs(sum-distance)>0.2) {
			System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
			calcSlice = Math.sqrt(1+Math.pow(2*a*curX + b, 2))*deltaX;
			sum+=(calcSlice);

			curX+=deltaX;
		}
		System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
		return curX;
	}
    public double yCalculate(double a, double b, double c, double x) {
        double y = a*Math.pow(x, 2) + b*x + c;
        return y;
    }
    public double angleCalculate(double a, double b, double c, double x) {
        double slope = 2*a*x+b;
        double angle = Math.toDegrees(Math.atan(slope));
        return angle;
    }
    public double xEllipseCalculate(double a, double b, double distance) {
		double sum = 0;
		double curX = 0;
		double calcSlice = 0;
		double deltaX = 0.01;
		while (Math.abs(sum-distance)>0.01) {
			System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
			calcSlice = Math.sqrt(1+Math.pow((-b*curX)/Math.sqrt(1-Math.pow(curX,2)/(a*a)),2))*deltaX;
			sum+=(calcSlice);

			curX+=deltaX;
		}
		System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
		return (curX > a)? a : curX;
	}

    public double yEllipseCalculate(double a, double b, double x) {
        double y = Math.sqrt((a*a*b*b-b*b*x*x)/(a*a));
        return y;
    }
    public double angleEllipseCalculate(double a, double b, double x) {
		double slope = (-b*x)/Math.sqrt(1-Math.pow(x,2)/(a*a));
		double angle = Math.toDegrees(Math.atan(slope));
		return angle;
	}
    public double xSinCalculate(double a, double b, double distance) {
		double sum = 0;
		double curX = 0;
		double calcSlice = 0;
		double deltaX = 0.01;
		while (Math.abs(sum-distance)>0.01) {
			System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
			calcSlice = Math.sqrt(1+Math.pow(a*b*Math.acos(b*curX),2))*deltaX;
			sum+=(calcSlice);

			curX+=deltaX;
		}
		System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
		return (curX > a)? a : curX;
	}

    public double ySinCalculate(double a, double b, double x) {
        double y = a*Math.sin(b*x);
        return y;
    }
    public double angleSinCalculate(double a, double b, double x) {
		double slope = a*b*Math.acos(b*x);
		double angle = Math.toDegrees(Math.atan(slope));
		return angle;
	}
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return false;
    }

}
