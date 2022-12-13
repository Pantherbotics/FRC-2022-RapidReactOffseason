package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.*;
import frc.robot.Constants;

public class SwerveModule {
    //Drive objects
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;

    //Turning objects
    private final TalonSRX turningMotor;
    private final CANCoder turningEncoder;
    private final AnalogInput absoluteEncoder;
    private final double turningEncoderOffsetRadians;

    public SwerveModule(int driveMotorID, int turningMotorID, int turningEncoderID, int absoluteEncoderID, double absoluteEncoderOffset) {
        //Create the SparkMax for the drive motor, and configure the units for its encoder
        driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //Get the drive PID controller and configure it for velocity PID
        drivePID = driveMotor.getPIDController();
        drivePID.setP(.0001);
        drivePID.setI(0);
        drivePID.setD(.0001);
        drivePID.setIZone(0);
        drivePID.setFF(.000175);
        drivePID.setOutputRange(-1, 1);

        //Create the CANCoder and configure it to work as the RemoteSensor0 for the turning motor
        turningEncoder = new CANCoder(turningEncoderID);
        turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        //Create the turning TalonSRX
        turningMotor = new TalonSRX(turningMotorID);
        turningMotor.configRemoteFeedbackFilter(turningEncoder, 0);
        turningMotor.config_kP(0, Constants.ModuleConstants.kPTurning);
        turningMotor.config_kI(0, Constants.ModuleConstants.kITurning);
        turningMotor.config_kD(0, Constants.ModuleConstants.kDTurning);
        turningMotor.config_kF(0, Constants.ModuleConstants.kFTurning);
        turningMotor.setSelectedSensorPosition(turningEncoder.getAbsolutePosition());


    }
}
