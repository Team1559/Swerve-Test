package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase{
    private final SwerveModuleIo[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator estimator;
    private final Supplier<Rotation2d> heading;

    public SwerveDrive (Supplier<Rotation2d> heading, SwerveModuleIo[] modules) {
        this.heading = heading;
        this.modules = modules;
        Translation2d[] locations = new Translation2d[modules.length];
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < locations.length; i ++){
            locations[i] = modules[i].getLocation();
            positions[i] = modules[i].getPosition();
        }
        this.kinematics = new SwerveDriveKinematics(locations);
        this.odometry = new SwerveDriveOdometry(kinematics, heading.get(), positions);
        this.estimator = new SwerveDrivePoseEstimator(kinematics, heading.get(), positions, new Pose2d());
    }

    public void driveFieldOriented(ChassisSpeeds speeds){
        Pose2d position = getPosition(); //hey doesn't have to be a variable :) ok thanks pookie
        driveRobotOriented(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, position.getRotation()));
    }

    public void driveRobotOriented(ChassisSpeeds speeds){
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        for(int i = 0; i < modules.length; i ++){
            modules[i].setState(SwerveModuleState.optimize(states[i], modules[i].getAngle()));
        }
        
    }

    public Pose2d getPosition(){
        return estimator.getEstimatedPosition();
    }

    public void updateOdometry(){
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < positions.length; i ++){
            positions[i] = modules[i].getPosition();
        }
        odometry.update(heading.get(), positions);
    }

    public void addVisionMeasurement(Pose2d estimatedPose2d, double timestamp, Matrix<N3,N1> standardDeviation){
        estimator.addVisionMeasurement(estimatedPose2d, timestamp, standardDeviation);
    }
}
