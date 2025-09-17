// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Setpoints_CMD;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SetpointConstants;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffector_Setpoint_CMD extends Command {
  /** Creates a new EndEffector_L1. */
  EndEffectorSubsystem m_endEffectorSubsystem;
  double m_endEffectorSetpoint;
  public EndEffector_Setpoint_CMD(EndEffectorSubsystem endEffectorSubsystem, double endEffectorSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_endEffectorSubsystem = endEffectorSubsystem;
    m_endEffectorSetpoint = endEffectorSetpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_endEffectorSubsystem.setPivotPosition(m_endEffectorSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_endEffectorSubsystem.EndEffectorPivotStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
