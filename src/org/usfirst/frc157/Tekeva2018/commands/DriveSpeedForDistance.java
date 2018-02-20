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

import org.usfirst.frc157.Tekeva2018.Robot;
import org.usfirst.frc157.Tekeva2018.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 *
 */
public class DriveSpeedForDistance extends Command
{

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private double leftSpeed;
    private double rightSpeed;
    private double dist;
    private boolean finished;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public DriveSpeedForDistance(double lspeed, double rspeed, double leftDist, double rightDist)
    {

        requires(Robot.drive);
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        Robot.drive.AutoDrive(0.0, 0.0);
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        leftSpeed = lspeed;
        rightSpeed = rspeed;
        dist = leftDist;
        // Sets
        Robot.drive.AutoDrive(rightSpeed, leftSpeed);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        Robot.drive.resetLeftEncoder();
        Robot.drive.resetRightEncoder();

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        //System.out.println("meme");
        //Robot.drive.AutoDrive(leftSpeed, -1 * rightSpeed);
        double leftEncoder = -1 * Robot.drive.getLeftEncoder();
        double rightEncoder = Robot.drive.getRightEncoder();
        if ((leftEncoder > rightEncoder))
        {
            Robot.drive.AutoDrive(rightSpeed, (leftSpeed*-1)*0.66);
        }
        else if ((rightEncoder > leftEncoder))
        {
            Robot.drive.AutoDrive(rightSpeed*0.66, -1 * leftSpeed);
        }
        else
        {
            Robot.drive.AutoDrive( rightSpeed, -1 * leftSpeed);
        }

     }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished()
    {
        System.out.println("right: " + Robot.drive.getRightEncoder());
        System.out.println("left: " + -1 *Robot.drive.getLeftEncoder());
        finished = Robot.drive.getLeftEncoder() * -1 >= dist || Robot.drive.getRightEncoder() >= dist;
        return finished;
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
