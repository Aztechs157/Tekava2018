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
import edu.wpi.first.wpilibj.Encoder;
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
public class RaiseElevatorForDistance extends Command
{

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private double speed;
    private double pos;
    private boolean finished;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    //private final WPI_TalonSRX elevatorMotor = RobotMap.forkliftElevatorElevatorMotor;
    private final Encoder forkEncoder = RobotMap.forkEncoder;
    public RaiseElevatorForDistance(double risespeed, double position)
    {


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        //elevatorMotor.set(ControlMode.PercentOutput, 0.0);
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.forkliftElevator);
        speed = risespeed;
        pos = position;
        // Sets
        //elevatorMotor.set(risespeed);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        //Robot.drive.AutoDrive(leftSpeed, -1 * rightSpeed);
        double change = speed;
        if (change > 0.0)
        {
            Robot.forkliftElevator.move(change, Robot.forkliftElevator.HIGH);
        }
        else if (change < 0.0)
        {
            Robot.forkliftElevator.move(change, Robot.forkliftElevator.LOW);
        }
        else
        {
            Robot.forkliftElevator.stop();
        }
        //from top to bottom it's 52.5"
        //startpos == 7.75
     }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished()
    {
        finished = Robot.forkliftElevator.getEncoder() >= pos;
        return finished;
    }

    // Called once after isFinished returns true
    @Override
    protected void end()
    {
        Robot.forkliftElevator.AutoRaise(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
    }
}
