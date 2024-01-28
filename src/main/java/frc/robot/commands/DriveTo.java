package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class DriveTo extends Command {
    private final SwerveDrive drive;
    private final Pose2d targetPose;
    private double error;

    public DriveTo(SwerveDrive drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        var currentPose = drive.getPose();
        var forwardError = targetPose.getX() - currentPose.getX();
        var sidewaysError = targetPose.getY() - currentPose.getY();
        error = Math.sqrt(Math.pow(forwardError, 2) + Math.pow(sidewaysError, 2));
        var response = MathUtil.clamp(error * 0.0001d, 0.1d, 0.1d);
        var angle = Math.atan(forwardError / sidewaysError);
        if (sidewaysError < 0) {
            angle += Math.PI;
        }
        var forwardResponse = response * Math.sin(angle);
        var sidewaysResponse = response * Math.cos(angle);
        var speeds = new ChassisSpeeds(
            MathUtil.clamp(forwardResponse, -0.1d, 0.1d),
            MathUtil.clamp(sidewaysResponse, -0.1d, 0.1d),
            0d
        );
        drive.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        return error < 3;
    }
}
