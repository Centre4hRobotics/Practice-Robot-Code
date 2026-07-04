package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.DriveOverseer;

public class DriveWithJoystick extends Command{

    DriveOverseer drive;
    CommandXboxController controller;

    public DriveWithJoystick(DriveOverseer drive, CommandXboxController controller) {
        
        this.drive = drive;
        this.controller = controller;

        addRequirements(drive);
    }

    public void initialize() {}

    public void execute() {

        Translation2d temp = new Translation2d(-controller.getLeftY(), controller.getLeftX());

        drive.drive(MotorConstants.maxDrivingVelocity.times(temp.getNorm()), temp.getAngle().getMeasure(), MotorConstants.maxDrivingAngularVelocity.times(controller.getRightX()), controller.getRightTriggerAxis());
    }

    public void end(boolean interrupted) {}

    public boolean isFinished() {
        return false;
    }
    
}
