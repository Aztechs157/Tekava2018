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

import edu.wpi.first.wpilibj.DriverStation;  
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc157.Tekeva2018.Robot;
import org.usfirst.frc157.Tekeva2018.RobotMap;
import org.usfirst.frc157.Tekeva2018.OI.ElvPos;
import org.usfirst.frc157.Tekeva2018.subsystems.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 *
 */
public class Autogroup extends CommandGroup 
{

    public String getGameData;
    private double startPos = 7.75;
    private double ground = startPos;
    private double maxExtenstion = 105;
    private double switchHeight = 18.75-startPos;
    private double scaleLow = (4*12)-startPos;
    private double scaleMed = (5*12)-startPos;
    private double scaleHigh = (6*12)-startPos;
    public int startingPoint = 2; //sp = Start Pos. and change value to starting posingtion (1 = left) (2 = middle) (3 = right)
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PARAMETERS
    public Autogroup()
    {

        getGameData = DriverStation.getInstance().getGameSpecificMessage();
        System.out.println(getGameData);

        addSequential(new RaiseElevatorToPoistion(ElvPos.SWITCH));
        if (getGameData == "LLL"|| getGameData == "Rll")
        {
            // StartPos: 1 Scale: L Switch: L Alliance: B

            // drives right before white no pass line
            if (startingPoint == 1)
            {
                addParallel(new RaiseElevatorByPercent(1.0));
                addSequential(new DriveSpeedForDistance(-1*0.4, -1*0.4, 197.5, 197.5)); //TODO CHANGE 0.4, 0.4 TO AROUND 1 FOR COMP!!!
                // Turns 90^o towards
                addSequential(new DriveSpeedForDistance(0, 0.4, 1, 1)); //TODO change to a rotateByDegree func.
                //Turn 90^o CC
                addSequential(new DriveSpeedForDistance(0.4, 0.0, 1, 1)); //TODO change to a rotateByDegree func.\
            }
            else if (startingPoint == 2)
            {
                System.out.println("MLL");
            //StartPos: 2 Scale: L Switch: L Alliance: B
            addParallel(new RaiseElevatorByPercent(0.5));
            addSequential(new DriveSpeedForDistance(-1*0.4, -1*0.4, 124.5, 124.5)); //TODO CHANGE 0.4, 0.4 TO AROUND 1 FOR COMP!!!
            }
        }

        if (getGameData == "LRR" || getGameData == "RRR")
        {
            //StartPos: 2 Scale: R Scale: R Alliance: B
            addParallel(new RaiseElevatorByPercent(0.5));
            addSequential(new DriveSpeedForDistance(-1*0.4, -1*0.4, 124.5, 124.5)); //TODO CHANGE 0.4, 0.4 TO AROUND 1 FOR COMP!!! 
        }

        if (getGameData == "LRL" || getGameData == "RRL")
        {
            //StartPos: 2 Scale: R Switch: L Alliance: B
            addParallel(new RaiseElevatorByPercent(0.5));
            addSequential(new DriveSpeedForDistance(-1*0.4, -1*0.4, 124.5, 124.5)); //TODO CHANGE 0.4, 0.4 TO AROUND 1 FOR COMP!!!
            //addSequentaial(new OpenForks(0.50));
        }

        if (getGameData == "LLR" || getGameData == "RLR")
        {
        //StartPos: 2 Scale: L Switch: R Alliance: B
        addParallel(new RaiseElevatorByPercent(0.5));
        addSequential(new DriveSpeedForDistance(-1*0.4, -1*0.4, 124.5, 124.5)); //TODO CHANGE 0.4, 0.4 TO AROUND 1 FOR COMP!!!
        }

        addSequential(new DriveSpeedForDistance(-1*0.2, -1*0.2, 50, 50));
        addSequential(new Autonomous());
        addParallel(new RaiseElevatorByPercent(-1*0.5));


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PARAMETERS
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=COMMAND_DECLARATIONS

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=COMMAND_DECLARATIONS

    } 
}
