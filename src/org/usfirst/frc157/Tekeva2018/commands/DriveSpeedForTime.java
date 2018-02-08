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
public class DriveSpeedForTime extends Command
{

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private double leftSpeed;
    private double rightSpeed;
    private double driveTime;
    private double stopTime;
    
    private boolean finished;

    
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    private final WPI_TalonSRX driveLeft1 = RobotMap.driveDriveLeft1;
    private final WPI_TalonSRX driveRight1 = RobotMap.driveDriveRight1;
    public DriveSpeedForTime(double lspeed, double rspeed, double timeSeconds) 
    {
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        driveLeft1.set(ControlMode.PercentOutput, 0.0);
        driveRight1.set(ControlMode.PercentOutput, 0.0);
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.drive);
        leftSpeed = lspeed;
        rightSpeed = rspeed;
        driveTime = timeSeconds;
        // Sets
        driveRight1.set(rightSpeed);
        driveLeft1.set(leftSpeed);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() 
    {
        stopTime = Timer.getFPGATimestamp() + driveTime;

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() 
    {
        System.out.println("meme");
        Robot.drive.AutoDrive(leftSpeed, rightSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() 
    {
        
        finished = Timer.getFPGATimestamp() > stopTime;
        return finished;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() 
    {
        
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() 
    {
    }
}
