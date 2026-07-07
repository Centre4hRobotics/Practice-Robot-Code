// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.OperatorConstants;
<<<<<<< HEAD
<<<<<<< HEAD
import frc.robot.commands.DriveWithJoystick;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.DriveOverseer;
import frc.robot.subsystems.Vision;
=======
=======
>>>>>>> 05d414eca19921682caf248b2a2536729e2d60d1
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.VisionCoprocessor;
import frc.robot.subsystems.VisionPhoton;
<<<<<<< HEAD
>>>>>>> 05d414eca19921682caf248b2a2536729e2d60d1
=======
>>>>>>> 05d414eca19921682caf248b2a2536729e2d60d1
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
<<<<<<< HEAD
  Vision vision = new Vision();
  Chassis chassis;
  DriveOverseer drive;
=======
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Vision visionSubsystem;
<<<<<<< HEAD
>>>>>>> 05d414eca19921682caf248b2a2536729e2d60d1
=======
>>>>>>> 05d414eca19921682caf248b2a2536729e2d60d1

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    try {
      chassis = new Chassis(MotorConstants.driveIDs, MotorConstants.turnIDs, MotorConstants.encoderIDs);
    }
    catch (Exception e) {
      e.printStackTrace();
    }

    drive = new DriveOverseer(chassis, vision, 11);

    // Configure the trigger bindings
    configureBindings();

    if (VisionConstants.usePhotonVision) {
      visionSubsystem = new VisionPhoton();
    } else {
      visionSubsystem = new VisionCoprocessor();
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    drive.setDefaultCommand(new DriveWithJoystick(drive, m_driverController));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
