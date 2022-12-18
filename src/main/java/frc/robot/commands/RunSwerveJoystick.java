package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class RunSwerveJoystick extends CommandBase {
    private final Drivetrain drivetrain;
    private final Joystick joystick;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public RunSwerveJoystick(Drivetrain drivetrain, Joystick joystick) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;
        //These limiters help to smooth out the joystick input by limiting the acceleration during sudden changes
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        //Background on the speed values:
        // - YL is positive when pulled BACKWARDS, and negative when pushed FORWARDS (not intuitive)
        // - XL is positive when pushed to the right, and negative when pushed to the left (normal)
        // - The Y axis on the joystick should control X (forward/backward) movement of the robot, and vice versa
        // - In order for the Y axis (negative when forward) to drive X forward (positive), it needs to be negated
        // - In order for the X axis to control the expected Y axis movement of positive to the left, it is also negated
        // - XR is positive when pushed to the right, and negative when pushed to the left (normal)
        // - In order for XR to follow the positive CCW of the gyro, it needs to be negated

        // 1. Get real-time joystick inputs, converted to work with Swerve and WPI
        double xSpeed = -powAxis(getYL(), OIConstants.driverEXP);
        double ySpeed = -powAxis(getXL(), OIConstants.driverEXP);
        double turningSpeed = -getXR();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        ;

        // 5. Output each module states to wheels
        drivetrain.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * @return The X axis value of the left joystick. It should be positive to the right.
     */
    private double getXL() {
        return joystick.getRawAxis(OIConstants.kDriverXL);
    }

    /**
     * @return The Y axis value of the left joystick. It should be NEGATIVE when forwards.
     */
    private double getYL() {
        return joystick.getRawAxis(OIConstants.kDriverYL);
    }

    /**
     * @return The X axis value of the right joystick. It should be positive to the right.
     */
    private double getXR() {
        return joystick.getRawAxis(OIConstants.kDriverXR);
    }

    /**
     * @return The Y axis value of the right joystick. It should be NEGATIVE when forwards.
     */
    private double getYR() {
        return joystick.getRawAxis(OIConstants.kDriverYR);
    }

    private double powAxis(double a, double b) {
        return (a >= 0) ? Math.pow(a, b) : -Math.pow(-a, b);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
