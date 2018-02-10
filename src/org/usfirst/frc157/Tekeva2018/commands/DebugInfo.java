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
import edu.wpi.first.wpilibj.command.Command; 

import java.text.DecimalFormat;

import org.usfirst.frc157.Tekeva2018.Robot;
import java.text.DecimalFormat;

import org.usfirst.frc157.Tekeva2018.Robot;
import org.usfirst.frc157.Tekeva2018.RobotMap;
import org.usfirst.frc157.Tekeva2018.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import org.usfirst.frc157.Tekeva2018.OI;

import com.ctre.phoenix.motorcontrol.ControlMode;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 *
 */
public class DebugInfo extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private final AnalogPotentiometer analogPotentiometer = RobotMap.motorControllerTeAnalogPotentiometer;
    private final AnalogPotentiometer analogpot = RobotMap.autoSelect;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public DebugInfo() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.forkliftElevator);
        requires(Robot.forkliftForks);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        DecimalFormat numberFormat = new DecimalFormat("0.00");
        double poten = analogpot.get();
        double potentiometer = analogPotentiometer.get();
        System.out.println("auto Select mode: " + (numberFormat.format(poten)));
        System.out.println(numberFormat.format(potentiometer));
        System.out.println(Robot.forkliftElevator.debugPrint());
        System.out.println(Robot.forkliftForks.debugPrint());
        System.out.println(Robot.drive.debugPrint());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
