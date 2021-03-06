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
import org.usfirst.frc157.Tekeva2018.OI;
import org.usfirst.frc157.Tekeva2018.PID;
import org.usfirst.frc157.Tekeva2018.Robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RaiseElevatorToPoistion extends Command
{

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private boolean finished;
    private double endpos;
    private OI.ElvPos dersiredPos;
    private PID liftPID = new PID(2,0,0,99999,99999,99999,99999);
    Preferences pref;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public RaiseElevatorToPoistion(OI.ElvPos pos)
    {
        dersiredPos = pos;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.forkliftElevator);
        //Robot.forkliftElevator

    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        switch (dersiredPos)
        {
            case GROUND:
                endpos = 0;
                break;
            case SWITCH:
                endpos = 11;
                break;
            case SCALELOW:
                endpos = 40.5;
                break;
            case SCALEMID:
                endpos = 52.25;
                break;
            case SCALEHIGH:
                endpos = 64.5;
                break;
            case MAX:
                endpos = 99.25;
                break;
        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        double encoder = Robot.forkliftElevator.getEncoder();
        /*
        double speed = 1;
        if (Math.abs(endpos - encoder) <= 5) speed = speed/2;
        if (encoder > endpos)
        {
            Robot.forkliftElevator.move(-speed, Robot.forkliftElevator.LOW);
        }
        else if (encoder < endpos)
        {
            Robot.forkliftElevator.move(speed, Robot.forkliftElevator.HIGH);
        }
        else
        {
            Robot.forkliftElevator.stop();
        }*/
        double power = liftPID.pidCalculate(endpos, encoder);
        Robot.forkliftElevator.move(power, Robot.forkliftElevator.HIGH);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished()
    {
        double encoder = Robot.forkliftElevator.getEncoder();
        boolean finished = encoder >= endpos - .2 && encoder <= endpos + .2;
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
