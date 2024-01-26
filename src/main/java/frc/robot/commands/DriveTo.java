package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class DriveTo extends Command {
    private final SwerveDrive drive;
    private final Pose2d targetPose;
    private final PIDController xPid = new PIDController(0.02d, 0d, 0d);
    private final PIDController yPid = new PIDController(0.02d, 0d, 0d);

    public DriveTo(SwerveDrive drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;
        xPid.setSetpoint(targetPose.getX());
        yPid.setSetpoint(targetPose.getY());
        addRequirements(drive);
    }

    @Override
    public void execute() {
        var forwardError = targetPose.getX() - drive.getPose().getX();
        var sidewaysError = targetPose.getY() - drive.getPose().getY();
        var error = Math.sqrt(Math.pow(forwardError, 2) + Math.pow(sidewaysError, 2));
        var response = MathUtil.clamp(error * 0.0001d, 0.1d, 0.1d);
        var angle = Math.atan(forwardError / sidewaysError);
        var forwardResponse = response * Math.sin(angle);
        var sidewaysResponse = response * Math.cos(angle);
        System.out.println(error + " : " + forwardResponse);
        var speeds = new ChassisSpeeds(
            MathUtil.clamp(forwardResponse, 0.1d, 0.1d),
            MathUtil.clamp(sidewaysResponse, 0.1d, 0.1d),
            0d
        );
        drive.drive(speeds);
    }
}
