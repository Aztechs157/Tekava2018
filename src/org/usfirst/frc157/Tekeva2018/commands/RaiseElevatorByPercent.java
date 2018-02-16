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
import edu.wpi.first.wpilibj.DigitalInput;  
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc157.Tekeva2018.Robot;
import org.usfirst.frc157.Tekeva2018.RobotMap;
import org.usfirst.frc157.Tekeva2018.subsystems.ForkliftElevator.switches;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 *
 */
public class RaiseElevatorByPercent extends Command
{

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private double raisetime;
    private boolean finished;
    private double stopTime;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    private final WPI_TalonSRX elevatorMotor = RobotMap.ElevatorMotor;
    private final DigitalInput highSwitch = RobotMap.forkliftElevatorHighSwitch;
    private final DigitalInput lowSwitch = RobotMap.forkliftElevatorLowSwitch;
    public RaiseElevatorByPercent(double percent)
    {
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        elevatorMotor.set(ControlMode.PercentOutput, 0.0);
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.forkliftElevator);
        raisetime = percent;
    }

    public enum switches 
    {
            HIGH,
            LOW
    };
    public switches HIGH = switches.HIGH;
    public switches LOW = switches.LOW;
    switches direction;
    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        stopTime = Timer.getFPGATimestamp() + (4.386974299*Math.abs(raisetime));   
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        double change = raisetime;

        if (change > 0.0)
        {
            Robot.forkliftElevator.move(1, Robot.forkliftElevator.HIGH);
        }
        else 
        {
            Robot.forkliftElevator.move(-1, Robot.forkliftElevator.LOW);
        }
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
        Robot.forkliftElevator.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
    }
}
