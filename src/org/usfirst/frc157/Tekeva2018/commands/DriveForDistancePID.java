// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc157.Tekeva2018.commands;
import edu.wpi.first.wpilibj.Timer;  
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc157.Tekeva2018.PID;
import org.usfirst.frc157.Tekeva2018.Robot;
import org.usfirst.frc157.Tekeva2018.RobotMap;
import org.usfirst.frc157.Tekeva2018.commands.Autonomous.autonState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 *
 */
public class DriveForDistancePID extends Command
{

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private double leftSpeed;
    private double rightSpeed;
    private double dist;
    private boolean finished;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public DriveForDistancePID(double lspeed, double rspeed, double leftDist, double rightDist)
    {

        requires(Robot.drive);
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        Robot.drive.AutoDrive(0.0, 0.0);
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
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
    boolean isFinished = false;
    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        Robot.drive.resetLeftEncoder();
        Robot.drive.resetRightEncoder();
        requires(Robot.drive);
        startTime = Timer.getFPGATimestamp();
        //state = autonState.driveForward;
        drivePID = new PID(0.16, 0, 0.000005, 999999, 99999, 999999, 9999999);
        gyroDrivePID = new PID(0.05, 0, 0, 999999, 99999, 999999, 9999999);
        gyroPID = new PID(0.01, 0, 0.000002, 9999999, 9999999, 9999999, 999999);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        //System.out.println("meme");
        //Robot.drive.AutoDrive(leftSpeed, -1 * rightSpeed);
        double encoder = -Robot.drive.getRightEncoder();
        System.out.println("Encoder: " + encoder);
        drivePower = drivePID.pidCalculate(1200, encoder); // (Distance, encoder)
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
                isFinished = true;
                //state = autonState.turnRight;
                repsAtTarget = 0;
            }
        }
        else
        {
            repsAtTarget = 0;
        }

     }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished()
    {
        return isFinished;
    }

    // Called once after isFinished returns true
    @Override
    protected void end()
    {
        Robot.drive.AutoDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
    }
}
