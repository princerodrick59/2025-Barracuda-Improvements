// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign;



import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionPIDCommand extends Command {
  /** Creates a new PositionPIDCommand. */
  private SwerveSubsystem m_swerveSubsystem;
  private AlignToReef m_alignToReef;

  public final Pose2d m_targetPose;
  private PPHolonomicDriveController m_driverController = Constants.AutoAlignConstants.kAutoAlignController;

  private final Timer timer = new Timer();

  private final Debouncer endTriggerDebouncer = new Debouncer(0.04);

  private PositionPIDCommand(SwerveSubsystem swerveSubsystem, Pose2d targetPose, AlignToReef alignToReef) {
    m_swerveSubsystem = swerveSubsystem;
    m_targetPose = targetPose;
    m_alignToReef = alignToReef;

  }

  public static Command generateCommand(SwerveSubsystem swerveSubsystem, Pose2d targetPose, Time timeout, AlignToReef kAlignToReef) {
    return new PositionPIDCommand(swerveSubsystem, targetPose, kAlignToReef)
        .withTimeout(timeout)
        .beforeStarting(kAlignToReef.setAutoAdjustActive(true))
        .andThen(kAlignToReef.setAutoAdjustActive(false))
        .finallyDo(() -> {
          swerveSubsystem.drive(new ChassisSpeeds(0,0,0));
          swerveSubsystem.lock();
         });
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
    targetState.pose = m_targetPose;

    m_swerveSubsystem.drive(
      m_driverController.calculateRobotRelativeSpeeds(m_swerveSubsystem.getPose(), targetState));

      Logger.recordOutput("Auto Adjust X Error", m_swerveSubsystem.getPose().getX()-m_targetPose.getX());
      Logger.recordOutput("Auto Adjust Y Error", m_swerveSubsystem.getPose().getY()-m_targetPose.getY());
      Logger.recordOutput("Auto Adjust Rot Error", m_swerveSubsystem.getPose().getRotation().getDegrees()-m_targetPose.getRotation().getDegrees());
  }


  // Logger.recordOutput("Auto Adjust X Error", m_swerveSubsystem.getPose().getX()-targetPose.getX());
  // Logger.recordOutput("Auto Adjust Y Error", m_swerveSubsystem.getPose().getY()-targetPose.getY());
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      timer.stop();

      Pose2d diffPose2d = m_swerveSubsystem.getPose().relativeTo(m_targetPose);

      System.out.println("Adjustments to alignment took: " + timer.get() + " seconds and interrupted = " + interrupted
          + "\nPosition offset: " + Inches.convertFrom(diffPose2d.getTranslation().getNorm(), Meters) + " inches"
          + "\nRotation offset: " + diffPose2d.getRotation().getMeasure().in(Degrees) + " deg"
          );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    Pose2d diffPose2d = m_swerveSubsystem.getPose().relativeTo(m_targetPose);

    var rotation = MathUtil.isNear(
      0,
      diffPose2d.getRotation().getRotations(), 
      Rotation2d.fromDegrees(1).getDegrees(),
      0,
      1
    );

    var position = diffPose2d.getTranslation().getNorm() < Inches.of(.5).in(Meters);

    return endTriggerDebouncer.calculate(
      rotation && position
    );


    
  }
}