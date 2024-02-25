package frc.robot.commands.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class MoveTo extends Command{
    
    private final Swerve s_Swerve;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    private double translationVal;
    private double strafeVal;
    private double rotationVal;

    private final Pose2d targetPos;

    public MoveTo(Swerve s_Swerve, Pose2d targetPose2d) {
        this.s_Swerve = s_Swerve;
        this.targetPos = targetPose2d;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        s_Swerve.zeroGyro();
        double xVal = targetPos.getX() / targetPos.getTranslation().getNorm();
        double yVal = targetPos.getY() / targetPos.getTranslation().getNorm();
        double zVal = targetPos.getRotation().getRotations() / (targetPos.getTranslation().getNorm() / Constants.Swerve.maxSpeed);
        translationVal =
            translationLimiter.calculate(
                MathUtil.applyDeadband(yVal, 0.03));
        strafeVal =
            strafeLimiter.calculate(
                MathUtil.applyDeadband(xVal, 0.03));
        rotationVal =
            rotationLimiter.calculate(
                MathUtil.applyDeadband(zVal, 0.03));
    }

    @Override
    public void execute() {
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
            rotationVal * Constants.Swerve.maxAngularVelocity,
            false, // idk
            true
        );
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0, 0), 0, false, false);
    }


    @Override
    public boolean isFinished() {
        if(targetPos.minus(s_Swerve.getPose()).getTranslation().getNorm() < 0.1) {return true;}
        else {return false;}
    }
}
