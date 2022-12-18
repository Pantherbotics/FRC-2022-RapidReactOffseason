package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    //Drive objects
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    //Turning objects
    private final TalonSRX turningMotor;
    private final CANCoder turningEncoder;
    private final PIDController turningPIDController;
    private final AnalogInput absoluteEncoder;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorID, int turningMotorID, int turningEncoderID, int absoluteEncoderID, double absoluteEncoderOffset) {
        //Create the SparkMax for the drive motor, and configure the units for its encoder
        driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        //Create the CANCoder and configure it to work as the RemoteSensor0 for the turning motor
        turningEncoder = new CANCoder(turningEncoderID);
        turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        //Create the turning TalonSRX
        turningMotor = new TalonSRX(turningMotorID);
        turningMotor.configRemoteFeedbackFilter(turningEncoder, 0);
        turningMotor.config_kP(0, ModuleConstants.kPTurning);
        turningMotor.config_kI(0, ModuleConstants.kITurning);
        turningMotor.config_kD(0, ModuleConstants.kDTurning);
        turningMotor.config_kF(0, ModuleConstants.kFTurning);
        turningMotor.setSelectedSensorPosition(turningEncoder.getAbsolutePosition());
        //Create PID Controller for turning motor
        turningPIDController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteEncoder = new AnalogInput(absoluteEncoderID);
        absoluteEncoderOffsetRad = absoluteEncoderOffset;

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }


    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        double position = turningPIDController.calculate(getTurningPosition(), state.angle.getRadians());
        turningMotor.set(ControlMode.Position, position);
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }
}
