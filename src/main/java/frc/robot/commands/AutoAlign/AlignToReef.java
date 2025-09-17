// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ReefConstants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  /** Creates a new AlignToReef. */

  // Subsystems
  private SwerveSubsystem m_swerveSubsystem;

  // Reef Side Enum
  public enum ReefSide {
    LEFT,
    RIGHT
  }

  // List of all reef branch poses
  public ArrayList<Pose2d> allReefPoses = new ArrayList<Pose2d>();

  // List of left reef branch poses
  public ArrayList<Pose2d> leftReefPoses = new ArrayList<Pose2d>();

  // List of right reef branch poses
  public ArrayList<Pose2d> rightReefPoses = new ArrayList<Pose2d>();

  // List of POV based left reef branch poses
  public ArrayList<Pose2d> POVBasedLeftReefPoses = new ArrayList<Pose2d>();

  // List of POV based right reef branch poses
  public ArrayList<Pose2d> POVBasedRightReefPoses = new ArrayList<Pose2d>();
  
  public AlignToReef(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSubsystem = swerveSubsystem;
  
    addRequirements(m_swerveSubsystem);

    // Add all reef poses to list
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

    // Add left reef poses to list
    leftReefPoses.add(ReefConstants.kAlpha_Reef);
    leftReefPoses.add(ReefConstants.kCharlie_Reef);
    leftReefPoses.add(ReefConstants.kEcho_Reef);
    leftReefPoses.add(ReefConstants.kGolf_Reef);
    leftReefPoses.add(ReefConstants.kIndia_Reef);
    leftReefPoses.add(ReefConstants.kKilo_Reef);

    // Add right reef poses to list
    rightReefPoses.add(ReefConstants.kBravo_Reef);
    rightReefPoses.add(ReefConstants.kDelta_Reef);
    rightReefPoses.add(ReefConstants.kFoxtrot_Reef);
    rightReefPoses.add(ReefConstants.kHotel_Reef);
    rightReefPoses.add(ReefConstants.kJuliet_Reef);
    rightReefPoses.add(ReefConstants.kLima_Reef);


    // Add POV based left reef poses to list
    POVBasedLeftReefPoses.add(ReefConstants.kAlpha_Reef);
    POVBasedLeftReefPoses.add(ReefConstants.kCharlie_Reef);
    POVBasedLeftReefPoses.add(ReefConstants.kFoxtrot_Reef);
    POVBasedLeftReefPoses.add(ReefConstants.kHotel_Reef);
    POVBasedLeftReefPoses.add(ReefConstants.kJuliet_Reef);
    POVBasedLeftReefPoses.add(ReefConstants.kKilo_Reef);

    // Add POV based right reef poses to list
    POVBasedRightReefPoses.add(ReefConstants.kBravo_Reef);
    POVBasedRightReefPoses.add(ReefConstants.kDelta_Reef);
    POVBasedRightReefPoses.add(ReefConstants.kEcho_Reef);
    POVBasedRightReefPoses.add(ReefConstants.kGolf_Reef);
    POVBasedRightReefPoses.add(ReefConstants.kIndia_Reef);
    POVBasedRightReefPoses.add(ReefConstants.kLima_Reef);


    
  }

  // Method to generate a path from the current robot pose to a waypoint
  public Command getPathFromWaypoint(Pose2d waypoint){

    // Generate waypoints from current robot pose to target waypoint
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(m_swerveSubsystem.getPose().getTranslation(), getPathVelocityHeading(m_swerveSubsystem.getFieldVelocity(), waypoint)),
      waypoint
    );
    // Create path constraints
    PathConstraints pathConstraints = new PathConstraints(1.5, 3, 180, 360);

    // Create path from waypoints and constraints
    PathPlannerPath path = new PathPlannerPath(
                                              waypoints,
                                              pathConstraints,
                                              new IdealStartingState(getVelociyMagnitude(m_swerveSubsystem.getFieldVelocity()), m_swerveSubsystem.getHeading()),
                                              new GoalEndState(0.0, waypoint.getRotation()));
                                              
    // Prevent path flipping
    path.preventFlipping = true;

    // Return command to follow path
    return AutoBuilder.followPath(path);

                                            
  }

  // Method to get Velocity Magnitude from ChassisSpeeds
  private LinearVelocity getVelociyMagnitude(ChassisSpeeds cs){
    return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  
  }

  // Method to get the heading based on the current velocity of the robot
  private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d targetPose){
    if (getVelociyMagnitude(cs).in(MetersPerSecond) < 0.25) { // If the robot is moving slower than 0.25 m/s, face the target
      var diff = targetPose.minus(m_swerveSubsystem.getPose()).getTranslation();
      return (diff.getNorm() < 0.01) ? targetPose.getRotation() : diff.getAngle(); // If the robot is within 1 cm of the target, keep the target rotation
    }
    return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
  }

  // Method to get the closest reef branch to the current robot pose
  private Pose2d getClosestBranch(Pose2d currentPose){
    return currentPose.nearest(allReefPoses);
  }

  // Method to get the closest left reef branch to the current robot pose
  private Pose2d getClosestLeftBranch(Pose2d currentPose){
    return currentPose.nearest(leftReefPoses);
  }

  // Method to get the closest right reef branch to the current robot pose
  private Pose2d getClosestRightBranch(Pose2d currentPose){
    return currentPose.nearest(rightReefPoses);
  }

  // Method to get the closest left reef branch based on POV
  private Pose2d getClosestPOVBasedLeftBranch(Pose2d currentPose){
    return currentPose.nearest(POVBasedLeftReefPoses);
  }

  // Method to get the closest right reef branch based on POV
  private Pose2d getClosestPOVBasedRightBranch(Pose2d currentPose){
    return currentPose.nearest(POVBasedRightReefPoses);
  }

  // Command to align to the closest reef branch
  private Command AlignToTheClosestReefBranch(){
    return Commands.defer(()-> {//Delay the calling of the command until the method is called/ button is pressed
      return getPathFromWaypoint(getClosestBranch(m_swerveSubsystem.getPose()));
    }, Set.of());
  } 

  // Command to align to the closest left reef branch
  private Command AlignToTheClosestLeftReefBranch(){
    return Commands.defer(()-> {//Delay the calling of the command until the method is called/ button is pressed
      return getPathFromWaypoint(getClosestLeftBranch(m_swerveSubsystem.getPose()));
    }, Set.of());
  }

  // Command to align to the closest right reef branch
  private Command AlignToTheClosestRightReefBranch(){
    return Commands.defer(()-> {//Delay the calling of the command until the method is called/ button is pressed
      return getPathFromWaypoint(getClosestRightBranch(m_swerveSubsystem.getPose()));
    }, Set.of());
  }

  // Command to align to the closest left reef branch based on POV
  private Command AlignToTheClosestPOVBasedLeftReefBranch(){
    return Commands.defer(()-> {//Delay the calling of the command until the method is called/ button is pressed
      return getPathFromWaypoint(getClosestPOVBasedLeftBranch(m_swerveSubsystem.getPose()));
    }, Set.of());
  }

  // Command to align to the closest right reef branch based on POV
  private Command AlignToTheClosestPOVBasedRightReefBranch(){
    return Commands.defer(()-> {//Delay the calling of the command until the method is called/ button is pressed
      return getPathFromWaypoint(getClosestPOVBasedRightBranch(m_swerveSubsystem.getPose()));
    }, Set.of());
  }

  // Return Command based on what reef side is passed in
  public Command AlignToTheClosestPOVBasedBranch(ReefSide side){
    switch (side) {
      case LEFT:
        return AlignToTheClosestPOVBasedLeftReefBranch();
      case RIGHT:
        return AlignToTheClosestPOVBasedRightReefBranch();
      default:
      return AlignToTheClosestPOVBasedRightReefBranch();
    }
  }

  // Return Command based on what reef side is passed in
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





  


}


