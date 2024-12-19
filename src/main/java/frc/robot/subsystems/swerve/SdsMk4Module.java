package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SdsMk4Module implements SwerveModuleIo {

    public enum GearRatio {
        L1(50d/14*19/25*45/15),
        L2(50d/14*17/27*45/15),
        L3(50d/14*16/28*45/15),
        L4(48d/16*16/28*45/15);

        private final double value;
        private GearRatio(double value){
            this.value = value;
        }
    }

    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    public static final double STEER_GEAR_RATIO = (60 / 10 * 32 / 15);

    private final GearRatio driveGearRatio;
    private final TalonFX steerMotor;
    private final TalonFX driveMotor;
    private final CANcoder cancoder;
    private final Rotation2d cancoderOffset;
    private final Translation2d location;

    public SdsMk4Module (TalonFX steerMotor, TalonFX driveMotor, CANcoder cancoder, GearRatio driveGearRatio, Rotation2d cancoderOffset, Translation2d location){
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;
        this.cancoder = cancoder;
        this.driveGearRatio = driveGearRatio;
        this.cancoderOffset = cancoderOffset;
        this.location = location;
    }

    @Override
    public void setSpeed(double speed) {
        VelocityVoltage control = new VelocityVoltage(speed * (1/(2 * WHEEL_RADIUS * Math.PI)) * driveGearRatio.value);
        driveMotor.setControl(control);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        PositionVoltage control = new PositionVoltage(angle.getRotations() * STEER_GEAR_RATIO);
        steerMotor.setControl(control);
    }

    @Override
    public Translation2d getLocation() {
        return location;
    }

    @Override
    public double getSpeed() {
        return driveMotor.getVelocity().getValueAsDouble() / driveGearRatio.value * (2 * WHEEL_RADIUS * Math.PI);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble()).minus(cancoderOffset);
    }

    @Override
    public double getDistanceTraveled() {
        return driveMotor.getVelocity().getValueAsDouble()*(2 * WHEEL_RADIUS * Math.PI);    }

    @Override
    public double getSteerMotorTemperature() {
        return steerMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public double getDriveMotorTemperature() {
        return driveMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public double getSteerMotorCurrent() {
        return steerMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getDriveMotorCurrent() {
        return driveMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getSteerMotorVelocity() {
        return steerMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getDriveMotorVelocity() {
       return driveMotor.getVelocity().getValueAsDouble();
    }
}
