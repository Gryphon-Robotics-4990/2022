// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.DriveUtil;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {

  private final DrivetrainSubsystem m_drivetrainSubsystem;
  //private final Constants constants = new Constants();

  private DoubleSupplier m_speedSupplier;
  private DoubleSupplier m_rotationSupplier;
  


  public DriveCommand(DrivetrainSubsystem drivetrainSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
  }
  public void setSuppliers(DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier) {
    m_speedSupplier = speedSupplier;
    m_rotationSupplier = rotationSupplier;
  }

  @Override
  public void execute() {
    double[] speeds = DriveUtil.arcadeToTankDrive(m_speedSupplier.getAsDouble(), m_rotationSupplier.getAsDouble() * 0.4);
    m_drivetrainSubsystem.driveVelocity(speeds[0], speeds[1]);
    
  }


  @Override
  public void end(boolean interrupted) {
    //m_drivetrainSubsystem.drivePO(0, 0);
  }


}
