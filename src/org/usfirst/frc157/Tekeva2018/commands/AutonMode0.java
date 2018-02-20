
package org.usfirst.frc157.Tekeva2018.commands;

import org.usfirst.frc157.Tekeva2018.PID;
import org.usfirst.frc157.Tekeva2018.Robot;
import org.usfirst.frc157.Tekeva2018.subsystems.PathManager;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class AutonMode0 extends Command
{

    public enum autonState 
    {
        driveArc1, wait1000Msec1, driveBack3, wait1000Msec2, turnRight90, driveArc2;
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
    private int quadrant = 0;
    private double a = 0;
    private double b = 0;
    private double c = 0;
    private double x = 0;
    private double y = 0;
    private double angle = 0;
    private double encoder;
    private int target;
    private  PathManager pathManager;
    private boolean pathOpen = false;
    private boolean autonFinished = false;
    private int ellipseX;
    private int ellipseY;
    
    public AutonMode0()
    {
        requires(Robot.drive);
        startTime = Timer.getFPGATimestamp();
        state = autonState.driveArc1;
        drivePID = new PID(0.16, 0, 0.000007, 999999, 99999, 999999, 9999999);
        gyroDrivePID = new PID(0.01, 0, 0.000001, 999999, 99999, 999999, 9999999);
        gyroPID = new PID(0.03, 0, 0.000003, 9999999, 9999999, 9999999, 999999);
        System.out.println("I got called"); 
       /* try {
        	pathManager = new PathManager();
        	pathOpen = true;
        }
        catch(Exception e) {
        	System.out.println(e.toString());
        }*/
    }

    @Override
    protected void execute()
    {
    	/*if (pathOpen) {
    		pathManager.update(-(Robot.drive.getRightEncoder()+Robot.drive.getLeftEncoder())/2.0, Robot.drive.getAngle());
    	}*/
    	switch (state)
        {

            case driveArc1:
                encoder = -(Robot.drive.getRightEncoder()+Robot.drive.getLeftEncoder())/2.0;
                target = 265;
                drivePower = drivePID.pidCalculate(target, encoder);
                ellipseX = 570;
                ellipseY = 30;
                x = xEllipseCalculate(ellipseX, ellipseY, encoder);
                y = yEllipseCalculate(ellipseX, ellipseY, x);
                angle = -angleEllipseCalculate(ellipseX, ellipseY, x);
                /*x = xSinCalculate(48,1/48.0, encoder);
                y = ySinCalculate(48,1/48.0, x);
                angle = angleSinCalculate(48,1/48.0, x);*/
                System.out.println("Right Encoder: "+Robot.drive.getRightEncoder()+"\nLeft Encoder: "+Robot.drive.getLeftEncoder());
                System.out.println("\nEncoder: " + encoder + "\nGyro: " + Robot.drive.getAngle() + "\nAngle: " + angle);

                leftPower = drivePower - gyroDrivePID.pidCalculate(angle + initAngle, Robot.drive.getAngle());
                leftPower = ((leftPower > 0) ? 1 : -1) * Math.min(1, Math.abs(leftPower));

                rightPower = drivePower + gyroDrivePID.pidCalculate(angle + initAngle, Robot.drive.getAngle());
                rightPower = ((rightPower > 0) ? 1 : -1) * Math.min(1, Math.abs(rightPower));

                Robot.drive.AutoDrive(leftPower, rightPower);
                if (Math.abs(encoder - target) < 5.0)
                {
                    repsAtTarget++;
                    if (repsAtTarget >= 5)
                    {
                    	reset();
                        state = autonState.wait1000Msec1;
                        System.out.println("broken 1");
                    }
                }
                else
                {
                    repsAtTarget = 0;
                }
                break;
            case driveArc2:
                double encoder = -(Robot.drive.getRightEncoder()+Robot.drive.getLeftEncoder())/2.0;
                target = 55;
                drivePower = drivePID.pidCalculate(target, encoder);
                ellipseX = 8;
                ellipseY = 54;
                x = xEllipseCalculate(ellipseX, ellipseY, encoder);
                y = yEllipseCalculate(ellipseX, ellipseY, x);
                angle = -angleEllipseCalculate(ellipseX, ellipseY, x);
                /*x = xSinCalculate(48,1/48.0, encoder);
                y = ySinCalculate(48,1/48.0, x);
                angle = angleSinCalculate(48,1/48.0, x);*/
                System.out.println("Right Encoder: "+Robot.drive.getRightEncoder()+"\nLeft Encoder: "+Robot.drive.getLeftEncoder());
                System.out.println("\nEncoder: " + encoder + "\nGyro: " + Robot.drive.getAngle() + "\nAngle: " + angle);

                leftPower = drivePower - gyroDrivePID.pidCalculate(angle + initAngle, Robot.drive.getAngle());
                leftPower = ((leftPower > 0) ? 1 : -1) * Math.min(1, Math.abs(leftPower));

                rightPower = drivePower + gyroDrivePID.pidCalculate(angle + initAngle, Robot.drive.getAngle());
                rightPower = ((rightPower > 0) ? 1 : -1) * Math.min(1, Math.abs(rightPower));

                Robot.drive.AutoDrive(leftPower, rightPower);
                if (Math.abs(encoder - target) < 3.0)
                {
                    repsAtTarget++;
                    if (repsAtTarget >= 5)
                    {

                    	autonFinished = true;
                        repsAtTarget = 0;
                    }
                }
                else
                {
                    repsAtTarget = 0;
                }
                break;
            case turnRight90:
                drivePower = gyroPID.pidCalculate(90, Robot.drive.getAngle());
                System.out.println("Angle: " + Robot.drive.getAngle() + "\nPower: " + drivePower);
                Robot.drive.AutoDrive(-drivePower, drivePower);
                if (Math.abs(Robot.drive.getAngle() - 90) < 2.0)
                {
                    repsAtTarget++;
                    if (repsAtTarget >= 10)
                    {	
                    	reset();
                        state = autonState.driveArc2;
                    }
                }
                else
                {
                    repsAtTarget = 0;
                }
                break;
 

            case driveBack3:
        		System.out.println("driveBack3 called");
            	encoder = -(Robot.drive.getRightEncoder()+Robot.drive.getLeftEncoder())/2.0;
                target = -25;
                drivePower = drivePID.pidCalculate(target, encoder);
                
                System.out.println("Right Encoder: "+Robot.drive.getRightEncoder()+"\nLeft Encoder: "+Robot.drive.getLeftEncoder());
                System.out.println("\nEncoder: " + encoder + "\nGyro: " + Robot.drive.getAngle() + "\nAngle: " + angle);

                leftPower = drivePower - gyroDrivePID.pidCalculate(initAngle, Robot.drive.getAngle());
                leftPower = ((leftPower > 0) ? 1 : -1) * Math.min(1, Math.abs(leftPower));

                rightPower = drivePower + gyroDrivePID.pidCalculate(initAngle, Robot.drive.getAngle());
                rightPower = ((rightPower > 0) ? 1 : -1) * Math.min(1, Math.abs(rightPower));

                Robot.drive.AutoDrive(leftPower, rightPower);
                if (Math.abs(encoder - target) < 3.0)
                {
                    repsAtTarget++;
                    if (repsAtTarget >= 5)
                    {
                    	reset();
                    	state = autonState.wait1000Msec2;
                		System.out.println("moving to wait1000Msec2");
                    }
                }
                else
                {
                    repsAtTarget = 0;
                }
                break;
            case wait1000Msec1:
            	System.out.println("wait1000Msec1");
            	repsAtTarget++;
            	System.out.println(repsAtTarget);
            	if(repsAtTarget>=100) {
            		state = autonState.driveBack3;
            		System.out.println("moving to driveBack3");
            		repsAtTarget = 0;
            	}
            	break;
            case wait1000Msec2:
        		System.out.println("wait1000Msec2 called");
            	repsAtTarget++;
            	System.out.println(repsAtTarget);
            	if(repsAtTarget>=100) {
            		state = autonState.turnRight90;
            		System.out.println("moving to turnRight90");
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
			//System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
			calcSlice = Math.sqrt(1+Math.pow(2*a*curX + b, 2))*deltaX;
			sum+=(calcSlice);

			curX+=deltaX;
		}
		//System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
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
    	double semiperimeter = Math.PI*Math.sqrt((a*a+b*b)/2)/2.0;
		quadrant = (int)(distance/semiperimeter)+1;
		System.out.println(quadrant);
		distance = distance-((quadrant-1)*semiperimeter);
		if (quadrant == 2||quadrant == 4) {
			distance = semiperimeter-distance;
		}
		System.out.println(distance);
		double origA = a;
		b = b / a;
		distance = distance / a;
		a = 1;
		double sum = 0;
		double curX = 0;
		double calcSlice = 0;
		double deltaX = 0.001;
		while (Math.abs(sum - distance) > 0.001) {
			// System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
			calcSlice = Math.sqrt(1 + Math.pow(-curX/(a*Math.sqrt(a*a-curX*curX)), 2)) * deltaX;
			sum += (calcSlice);

			curX += deltaX;
		}
		// System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
		double returnVal = (curX > a) ? a*origA : curX*origA;
		return (quadrant == 3||quadrant == 4)? -returnVal: returnVal;
	}

	public double yEllipseCalculate(double a, double b, double x) {
		double origA = a;
		b = b / a;
		x = x / a;
		a = 1;
		double y = Math.sqrt((a * a * b * b - b * b * x * x) / (a * a));
		return origA*y;
	}

	public double angleEllipseCalculate(double a, double b, double x) {
		x = Math.abs(x);
		b = b / a;
		x = x / a;
		a = 1;
		double slope = -x/(a*Math.sqrt(a*a-x*x));
		double angle = Math.toDegrees(Math.atan(slope)); //((quadrant==1||quadrant==3)? 1:-1)
		if (quadrant == 2) {
			angle = ((angle>=0)? 1: -1) *(180-Math.abs(angle));
		}
		return angle;
	}
    public double xSinCalculate(double a, double b, double distance) {
		double sum = 0;
		double curX = 0;
		double calcSlice = 0;
		double deltaX = 0.001;
		while (Math.abs(sum-distance)>0.01) {
			//System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
			calcSlice = Math.sqrt(1+Math.pow(a*b*Math.acos(b*curX),2))*deltaX;
			sum+=(calcSlice);

			curX+=deltaX;
		}
		//System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
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
        return autonFinished;
    }
    public void reset() {
    	 Robot.drive.AutoDrive(0,0);
         Robot.drive.resetLeftEncoder();
     	 Robot.drive.resetRightEncoder();
         repsAtTarget = 0;
         initAngle = Robot.drive.getAngle();
    }
}
