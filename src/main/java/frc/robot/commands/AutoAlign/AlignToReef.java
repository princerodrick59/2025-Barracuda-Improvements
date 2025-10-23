// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign;

import static edu.wpi.first.units.Units.*;


import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ReefConstants;
import frc.robot.commands.Sequential.Reef.L1_CMD;
import frc.robot.commands.Sequential.Reef.L3_CMD;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  /** Creates a new AlignToReef. */

  //Subsystems
  private SwerveSubsystem m_swerveSubsystem;

  public enum ReefSide {
    LEFT, 
    RIGHT
  }

  public enum ReefLevel {
    L1,
    L2,
    L3,
    L4,
    STOW
  }

  public ReefLevel desiredLevel = ReefLevel.STOW;

  private boolean isAutoAdjustActive = false;

  public ArrayList<Pose2d> allReefPoses = new ArrayList<Pose2d>();
  public ArrayList<Pose2d> leftReefPoses = new ArrayList<Pose2d>();
  public ArrayList<Pose2d> rightReefPoses = new ArrayList<Pose2d>();
  public ArrayList<Pose2d> POVBasedLeftReefPoses = new ArrayList<Pose2d>();
  public ArrayList<Pose2d> POVBasedRightReefPoses = new ArrayList<Pose2d>();

  private ElevatorSubsystem m_elevatorSubsystem;
  private EndEffectorSubsystem m_endEffectorSubsystem;

  public AlignToReef(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSubsystem = swerveSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_endEffectorSubsystem = endEffectorSubsystem;

    addRequirements(m_swerveSubsystem);

    //add all reef poses to list
    allReefPoses.add(ReefConstants.kAlpha_Reef);
    allReefPoses.add(ReefConstants.kBravo_Reef);
    allReefPoses.add(ReefConstants.kCharlie_Reef);
    allReefPoses.add(ReefConstants.kDelta_Reef);
    allReefPoses.add(ReefConstants.kEcho_Reef);
    allReefPoses.add(ReefConstants.kFoxtrot_Reef);
    allReefPoses.add(ReefConstants.kGolf_Reef);
    allReefPoses.add(ReefConstants.kHotel_Reef);
    allReefPoses.add(ReefConstants.kIndia_Reef);
    allReefPoses.add(ReefConstants.kJuliet_Reef);
    allReefPoses.add(ReefConstants.kKilo_Reef);
    allReefPoses.add(ReefConstants.kLima_Reef);

    //add left reef poses to list
    leftReefPoses.add(ReefConstants.kAlpha_Reef);
    leftReefPoses.add(ReefConstants.kCharlie_Reef);
    leftReefPoses.add(ReefConstants.kEcho_Reef);
    leftReefPoses.add(ReefConstants.kGolf_Reef);
    leftReefPoses.add(ReefConstants.kIndia_Reef);
    leftReefPoses.add(ReefConstants.kKilo_Reef);

    //add right reef poses to list
    rightReefPoses.add(ReefConstants.kBravo_Reef);
    rightReefPoses.add(ReefConstants.kDelta_Reef);
    rightReefPoses.add(ReefConstants.kFoxtrot_Reef);
    rightReefPoses.add(ReefConstants.kHotel_Reef);
    rightReefPoses.add(ReefConstants.kJuliet_Reef);
    rightReefPoses.add(ReefConstants.kLima_Reef);

    //add POV based left reef poses to list
    POVBasedLeftReefPoses.add(ReefConstants.kAlpha_Reef);
    POVBasedLeftReefPoses.add(ReefConstants.kCharlie_Reef);
    POVBasedLeftReefPoses.add(ReefConstants.kFoxtrot_Reef);
    POVBasedLeftReefPoses.add(ReefConstants.kHotel_Reef);
    POVBasedLeftReefPoses.add(ReefConstants.kJuliet_Reef);
    POVBasedLeftReefPoses.add(ReefConstants.kKilo_Reef);

    //add POV based right reef poses to list
    POVBasedRightReefPoses.add(ReefConstants.kBravo_Reef);
    POVBasedRightReefPoses.add(ReefConstants.kDelta_Reef);
    POVBasedRightReefPoses.add(ReefConstants.kEcho_Reef);
    POVBasedRightReefPoses.add(ReefConstants.kGolf_Reef);
    POVBasedRightReefPoses.add(ReefConstants.kIndia_Reef);
    POVBasedRightReefPoses.add(ReefConstants.kLima_Reef);
  }
  /**
   * Method to generate a command to follow a path to a waypoint | 
   * Auto adjusts to the waypoint upon arrival | 
   * Auto adjusts override if within 0.25 meters of waypoint | 
   * Moves elevator to desired reef level if within 1 meter of waypoint | 
   * @param waypoint The target waypoint to align to
   * @return Command to follow the path to the waypoint
   */
  public Command getPathFromWaypoint(Pose2d waypoint) {
    // Create waypoints for pathplanner path
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(m_swerveSubsystem.getPose().getTranslation(), getPathVelocityHeading(m_swerveSubsystem.getFieldVelocity(), waypoint)),
      waypoint
    );
    
    // Create path constraints
    PathConstraints pathConstraints = new PathConstraints(1.5, 3, 180, 360);

    // Create pathplanner path
    PathPlannerPath path = new PathPlannerPath(
                                              waypoints, 
                                              pathConstraints, 
                                              new IdealStartingState(getVelocityMagnitude(m_swerveSubsystem.getFieldVelocity()), m_swerveSubsystem.getHeading()),
                                              new GoalEndState(0.0, waypoint.getRotation()));


    path.preventFlipping = true;
    // Build and return command
    return (AutoBuilder.followPath(path)
            .andThen(
                    // Auto Adjust after reaching the waypoint
                    PositionPIDCommand.generateCommand(m_swerveSubsystem, waypoint, Seconds.of(2),this)))
            .alongWith(
                    // Auto adjust override if closer than 0.25 meters to waypoint
                    PositionPIDCommand.generateCommand(m_swerveSubsystem, waypoint, Seconds.of(2),this)
                    //Print Starting
                    .beforeStarting(Commands.print("Starting final approach PID"))
                    //Print Ending
                    .andThen(Commands.print("Ending final approach PID"))

                    .onlyIf(
                            // Condition to only run if within 0.25 meters of waypoint
                            ()-> m_swerveSubsystem.getPose().getTranslation().getDistance(waypoints.get(1).anchor()) < 0.25)
                            // Run repeatedly to check condition
                            .repeatedly())
                    
            .alongWith(
                    // Command to move elevator to desired reef level if closer than 1 meter to waypoint
                    getDesiredReefCommand(desiredLevel)
                    //Print Starting
                    .beforeStarting(Commands.print("Moving to " + desiredLevel.toString() + " Reef Position"))

                    .onlyIf(
                            // Condition to only run if within 1 meter of waypoint
                            ()-> m_swerveSubsystem.getPose().getTranslation().getDistance(waypoints.get(1).anchor()) < 1)
                            // Run repeatedly to check condition
                            .repeatedly())
            // Until interrupted
            .finallyDo((Interupt) -> {
              if (Interupt) {
                // Stop the robot if interrupted
                m_swerveSubsystem.drive(new ChassisSpeeds(0,0,0));
              }
    });
  }
  


  // Method to get Velocity Magnitude from ChassisSpeeds
  private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
    return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  
  }

  // Method to get the heading based on the current velocity of the robot
  private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d targetPose){
    if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) { // If the robot is moving slower than 0.25 m/s, face the target
      var diff = targetPose.minus(m_swerveSubsystem.getPose()).getTranslation();
      return (diff.getNorm() < 0.01) ? targetPose.getRotation() : diff.getAngle(); // If the robot is within 1 cm of the target, keep the target rotation
    }
    return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
  }

  private Pose2d getClosestBranch(Pose2d currentPose){
  return currentPose.nearest(allReefPoses);
  }

  private Pose2d getClosestLeftBranch(Pose2d currentPose){
    return currentPose.nearest(leftReefPoses);
  }

  private Pose2d getClosestRightBranch(Pose2d currentPose){
    return currentPose.nearest(rightReefPoses);
  }


  public Command AlignToTheClosestReefBranch(){
    return Commands.defer(()-> {
      return getPathFromWaypoint(getClosestBranch(m_swerveSubsystem.getPose()));
    }, Set.of());
  }

  private Command AlignToTheClosestLeftReefBranch(){
    return Commands.defer(()->{
    return getPathFromWaypoint(getClosestLeftBranch(m_swerveSubsystem.getPose()));
    }, Set.of());
  }

  private Command AlignToTheClosestRightReefBranch(){
    return Commands.defer(()->{
      return getPathFromWaypoint(getClosestRightBranch(m_swerveSubsystem.getPose()));
    }, Set.of());
  }

  public Command AlignToLeftHP(){
    return Commands.defer(()->{
      return getPathFromWaypoint(new Pose2d(1.091,7.052,Rotation2d.fromDegrees(-55)));
    }, Set.of());
  }

  public Command AlignToTheClosestBranch(ReefSide side){
    switch (side) {
      case LEFT:
        return AlignToTheClosestLeftReefBranch();
      case RIGHT:
        return AlignToTheClosestRightReefBranch();
      default:
      return AlignToTheClosestRightReefBranch();
    }
  }

  public Command setDesiredReefLevel(ReefLevel level){
    return Commands.runOnce(() -> desiredLevel = level)
          .alongWith(Commands.print("Set desired reef level to " + level.toString()));
  }

  private Command getDesiredReefCommand(ReefLevel level){
    switch (level) {
      case L1:
        return new L1_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);
      case L2:
        return new L3_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);
      case L3:
        return new L3_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);
      case L4:
        return new L3_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);
      default:
        return Commands.none();    
    }
  } 

  public Command setAutoAdjustActive(boolean isActive){
    return Commands.runOnce(()-> isAutoAdjustActive = isActive);
  }

  @AutoLogOutput
  public boolean getIsAutoAdjustActive(){
    return isAutoAdjustActive;
  }


}
