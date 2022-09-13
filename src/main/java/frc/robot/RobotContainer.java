// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Joystick joystick = new Joystick(2);

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final DriveCommand m_driveCommand = new DriveCommand(m_drivetrainSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    setTeleopDefaultCommands();
  }
  
  private void configureButtonBindings() {
    m_driveCommand.setSuppliers(
      () -> DriveUtil.powCopySign(-joystick.getRawAxis(1), 1),
      () -> DriveUtil.powCopySign(joystick.getRawAxis(2), 1)
    );
    //m_drivetrainSubsystem.drivePO(new Joystick(2)getRawAxis(1), new Joystick(2).getRawAxis(1));
    //m_drivetrainSubsystem.drivePO(0.1, 0.1);
  }


  public void setTeleopDefaultCommands()
  {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrainSubsystem, m_driveCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
