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
                System.out.println("Encoder: " + encoder);
                drivePower = drivePID.pidCalculate(2500, encoder);
                leftPower = drivePower - gyroDrivePID.pidCalculate(initAngle, Robot.drive.getAngle());
                leftPower = ((leftPower > 0) ? 1 : -1) * Math.min(1, Math.abs(leftPower));
                rightPower = drivePower + gyroDrivePID.pidCalculate(initAngle, Robot.drive.getAngle());
                rightPower = ((rightPower > 0) ? 1 : -1) * Math.min(1, Math.abs(rightPower));
                System.out.println("Left Power: " + leftPower + "\nRight Power: " + rightPower);
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

    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return false;
    }

}
