package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface SwerveModuleIo {
    void setSpeed(double speed);

    void setAngle(Rotation2d angle);

    Translation2d getLocation();

    double getSpeed();

    Rotation2d getAngle();

    double getDistanceTraveled();

    double getSteerMotorTemperature();

    double getDriveMotorTemperature();

    double getSteerMotorCurrent();

    double getDriveMotorCurrent();

    double getSteerMotorVelocity();

    double getDriveMotorVelocity();
}