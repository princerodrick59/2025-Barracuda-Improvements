package frc.robot.commands.AutoAlign;



import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;



import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;

public class PositionPIDCommand extends Command{
    
    public SwerveSubsystem mSwerve;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = 
    new PPHolonomicDriveController(
        new PIDConstants(1,0,0), 
        new PIDConstants(1,0,0));

    private final Timer timer = new Timer();

    private final Debouncer endTriggerDebouncer = new Debouncer(0.04);



   



    private PositionPIDCommand(SwerveSubsystem mSwerve, Pose2d goalPose) {
        this.mSwerve = mSwerve;
        this.goalPose = goalPose;
    }

    public static Command generateCommand(SwerveSubsystem swerve, Pose2d goalPose, Time timeout){
        return new PositionPIDCommand(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.drive(new ChassisSpeeds(0,0,0));
            swerve.lock();
        });
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        mSwerve.drive(
            mDriveController.calculateRobotRelativeSpeeds(
                mSwerve.getPose(), goalState
            )
        );

        Logger.recordOutput("Auto Adjust X Error", mSwerve.getPose().getX() - goalPose.getX());
        Logger.recordOutput("Auto Adjust Y Error", mSwerve.getPose().getY() - goalPose.getY());
        Logger.recordOutput("Auto Adjust Rot Error", mSwerve.getPose().getRotation().getDegrees() - goalPose.getRotation().getDegrees());


    
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

        System.out.println("Adjustments to alignment took: " + timer.get() + " seconds and interrupted = " + interrupted
            + "\nPosition offset: " + Inches.convertFrom(diff.getTranslation().getNorm(), Meters) + " inches"
            + "\nRotation offset: " + diff.getRotation().getMeasure().in(Degrees) + " deg"
        );

    }

    @Override
    public boolean isFinished() {
        
        Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

        var rotation = MathUtil.isNear(
            0.0, 
            diff.getRotation().getRotations(), 
            Rotation2d.fromDegrees(1.0).getDegrees(), // 1 degree tolerance
            0.0, 
            1.0
        );

        var position = diff.getTranslation().getNorm() < Inches.of(0.5).in(Meters); // 0.5 inch tolerance

        return endTriggerDebouncer.calculate(
            rotation && position
        );
        
       
    }
}