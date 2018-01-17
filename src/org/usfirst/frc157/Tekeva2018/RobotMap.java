// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc157.Tekeva2018;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static Encoder driveLeftQuad;
    public static Encoder driveRightQuad;
    public static WPI_TalonSRX driveDriveLeft1;
    public static WPI_TalonSRX driveDriveLeft2;
    public static WPI_TalonSRX driveDriveRight1;
    public static WPI_TalonSRX driveDriveRight2;
    public static WPI_TalonSRX forkliftForksForkMotor;
    public static DigitalInput forkliftForksOpenSwitch;
    public static DigitalInput forkliftForksCloseSwitch;
    public static WPI_TalonSRX forkliftElevatorElevatorMotor;
    public static DigitalInput forkliftElevatorHighSwitch;
    public static DigitalInput forkliftElevatorLowSwitch;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveLeftQuad = new Encoder(1, 2, false, EncodingType.k4X);
        LiveWindow.addSensor("Drive", "LeftQuad", driveLeftQuad);
        driveLeftQuad.setDistancePerPulse(1.0);
        driveLeftQuad.setPIDSourceType(PIDSourceType.kRate);
        driveRightQuad = new Encoder(3, 4, false, EncodingType.k4X);
        LiveWindow.addSensor("Drive", "RightQuad", driveRightQuad);
        driveRightQuad.setDistancePerPulse(1.0);
        driveRightQuad.setPIDSourceType(PIDSourceType.kRate);
        driveDriveLeft1 = new WPI_TalonSRX(11);
        
        
        driveDriveLeft2 = new WPI_TalonSRX(9);
        
        
        driveDriveRight1 = new WPI_TalonSRX(2);
        
        
        driveDriveRight2 = new WPI_TalonSRX(3);
        
        
        forkliftForksForkMotor = new WPI_TalonSRX(6);
        
        
        forkliftForksOpenSwitch = new DigitalInput(7);
        LiveWindow.addSensor("ForkliftForks", "OpenSwitch", forkliftForksOpenSwitch);
        
        forkliftForksCloseSwitch = new DigitalInput(6);
        LiveWindow.addSensor("ForkliftForks", "CloseSwitch", forkliftForksCloseSwitch);
        
        forkliftElevatorElevatorMotor = new WPI_TalonSRX(1);
        
        
        forkliftElevatorHighSwitch = new DigitalInput(9);
        LiveWindow.addSensor("ForkliftElevator", "HighSwitch", forkliftElevatorHighSwitch);
        
        forkliftElevatorLowSwitch = new DigitalInput(8);
        LiveWindow.addSensor("ForkliftElevator", "LowSwitch", forkliftElevatorLowSwitch);
        

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}
