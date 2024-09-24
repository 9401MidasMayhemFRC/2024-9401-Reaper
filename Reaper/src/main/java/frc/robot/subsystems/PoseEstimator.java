package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.utilities.MathUtils;

public class PoseEstimator extends SubsystemBase {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDrivePoseEstimator m_noteEstimator;
    private final Drivetrain m_drivetrain;
    private final Field2d m_field = new Field2d();
    private boolean m_auto = true;
    private boolean m_trackNote = false;
    private int m_noteID = 0;

    private InterpolatingDoubleTreeMap m_noteDistance = new InterpolatingDoubleTreeMap();

    public PoseEstimator(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
                drivetrain.getGyro(),
                drivetrain.getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.229, 0.229, 0.229),
                VecBuilder.fill(10, 10, 10));
        m_noteDistance = MathUtils.pointsToTreeMap(VisionConstants.kNoteDistance);
        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putBoolean("Reset Pose", false);

        m_noteEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
                drivetrain.getGyro(),
                drivetrain.getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.229, 0.229, 0.229),
                VecBuilder.fill(10, 10, 10));
    }

    @Override
    public void periodic() {
        updatePoseEstimator(false);
        updateShuffleboard();
    }

    private void updateWithNote(int noteID) {

        double tx = LimelightHelpers.getTY("limelight-note");
        double ty = LimelightHelpers.getTX("limelight-note");
        double yAdj = (ty - 0.005936 * tx * tx) / (0.000203 * tx * tx + 1);
        double distance = m_noteDistance.get(yAdj) * 0.0254;

        if (distance >= 0) {
            SmartDashboard.putNumber("DistanceToNote", distance);
            Rotation2d robotAngle = m_drivetrain.getGyro();
            double visionAngle = (robotAngle.getRadians() - Math.toRadians(tx));
            SmartDashboard.putNumber("Vision Angle", visionAngle);
            Translation2d noteToRobot = new Translation2d(distance, new Rotation2d(visionAngle));
            Pose2d calculatedPose = new Pose2d(VisionConstants.kNoteIDs[noteID].plus(noteToRobot), robotAngle);

            SmartDashboard.putNumber("NoteCalcX", calculatedPose.getX());
            SmartDashboard.putNumber("NoteCalcY", calculatedPose.getY());
                        SmartDashboard.putNumber("NoteVecX", noteToRobot.getX()*39.37);
            SmartDashboard.putNumber("NoteVecY", noteToRobot.getY()*39.37);

            double latency = LimelightHelpers.getLatency_Capture("limelight-note")
                    + LimelightHelpers.getLatency_Pipeline("limelight-note");

            latency /= 1000.0;
            
            double timestamp = Timer.getFPGATimestamp() - latency;

            SmartDashboard.putNumber("NoteLatency", latency);

            if (calculatedPose.getTranslation().getDistance(getPose().getTranslation()) <= 2.0) {
                m_noteEstimator.addVisionMeasurement(calculatedPose, timestamp, VecBuilder.fill(2.0, 2.0, 999999));
            }
        }

    }

    public void trackNote(int noteID) {
        m_noteID = noteID;
        m_noteEstimator.resetPosition(m_drivetrain.getGyro(), m_drivetrain.getModulePositions(), getPose());

        m_trackNote = true;
    }

    public void stopNoteTracking() {
        m_trackNote = false;
    }

    private void updatePoseEstimator(boolean force) {
        double limelightLatency = LimelightHelpers.getLatency_Pipeline("")
                + LimelightHelpers.getLatency_Capture("");
        limelightLatency = limelightLatency / 1000.0;
        double timestamp = Timer.getFPGATimestamp() - limelightLatency;
        double velocity = MathUtils.pythagorean(m_drivetrain.getChassisSpeed().vxMetersPerSecond,
                m_drivetrain.getChassisSpeed().vyMetersPerSecond);
        double angularVelocity = m_drivetrain.getChassisSpeed().omegaRadiansPerSecond;
        Pose2d limelightBotPose = LimelightHelpers.getBotPose2d_wpiBlue("");

        // double yAdj = 0.945*(limelightBotPose.getY())+.305;

        // limelightBotPose = new Pose2d(limelightBotPose.getX(), yAdj,
        // limelightBotPose.getRotation());

        Pose2d currentPose = getPose();
        boolean validSolution = LimelightHelpers.getTV("");
        double ta = LimelightHelpers.getTA("");

        // LimelightResults results = LimelightHelpers.getLatestResults("");
        int validTagCount = LimelightHelpers.getTargetCount("");

        SmartDashboard.putBoolean("Auto Pose", m_auto);
        SmartDashboard.putNumber("Limelight TA", ta);

        boolean updatePose = ((currentPose.getTranslation()
                .getDistance(limelightBotPose.getTranslation()) <= 3.0)
                && validSolution && limelightBotPose.getTranslation().getDistance(currentPose.getTranslation()) >= 0.05
                && velocity <= 4.0 && angularVelocity <= 0.5 * Math.PI);

        boolean slowRotate = m_drivetrain.getChassisSpeed().omegaRadiansPerSecond <= 4 * Math.PI;

        // boolean m_overide = SmartDashboard.getBoolean("Set Pose Est", false);

        m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_drivetrain.getGyro(),
                m_drivetrain.getModulePositions());
        m_noteEstimator.updateWithTime(Timer.getFPGATimestamp(), m_drivetrain.getGyro(),
                m_drivetrain.getModulePositions());
        if ((validTagCount >= 2.0 && ta >= 0.035) && slowRotate) {
            if (m_auto) {
                m_poseEstimator.addVisionMeasurement(limelightBotPose, timestamp, VecBuilder.fill(10.0, 10.0, 10.0));
            } else {
                double antiTrust = -150.0 * ta + 10.0;
                if (antiTrust <= 2.0) {
                    antiTrust = 2.0;
                }
                m_poseEstimator.addVisionMeasurement(limelightBotPose, timestamp,
                        VecBuilder.fill(antiTrust, antiTrust, antiTrust));

            }
        } else if ((updatePose && validTagCount == 1 && ta >= 0.055) && slowRotate) {
            double antiTrust = -69.0 * ta + 14.83;
            if (antiTrust <= 5.0) {
                antiTrust = 5.0;
            }
            if (m_auto) {
                m_poseEstimator.addVisionMeasurement(limelightBotPose, timestamp, VecBuilder.fill(20.0, 20.0, 20.0));
            } else {
                m_poseEstimator.addVisionMeasurement(limelightBotPose, timestamp,
                        VecBuilder.fill(antiTrust, antiTrust, antiTrust));
            }
        }
        if (m_trackNote && LimelightHelpers.getTV("limelight-note")) {
            updateWithNote(m_noteID);
        }

    }

    private void updateShuffleboard() {
        Pose2d pose = getPose();
        SmartDashboard.putNumber("PoseEstX", pose.getX());
        SmartDashboard.putNumber("PoseEstY", pose.getY());
        SmartDashboard.putNumber("PoseEstRot", pose.getRotation().getRadians());

        if (SmartDashboard.getBoolean("Reset Pose", false)) {
            updatePoseEstimator(true);
        }

        m_field.setRobotPose(pose);

    }

    public Pose2d getPose() {
        if (m_trackNote) {
            return m_noteEstimator.getEstimatedPosition();
        } else {
            return m_poseEstimator.getEstimatedPosition();
        }

    }

    public void setAuto(boolean auto) {
        m_auto = auto;
    }

    public boolean inside(Translation2d[] bounds, boolean onEdge) {
        Pose2d currentPose = getPose();
        double xMin = Math.min(bounds[0].getX(), bounds[1].getX());
        double xMax = Math.max(bounds[0].getX(), bounds[1].getX());
        double yMin = Math.min(bounds[0].getY(), bounds[1].getY());
        double yMax = Math.max(bounds[0].getY(), bounds[1].getY());
        return ((currentPose.getX() > xMin && currentPose.getX() < xMax)
                || (onEdge && (currentPose.getX() >= xMin && currentPose.getX() <= xMax))
                        &&
                        (currentPose.getY() > yMin && currentPose.getY() < yMax)
                || (onEdge && (currentPose.getY() >= yMin && currentPose.getY() <= yMax)));
    }

    public void resetOdometry(Pose2d pose) {
        m_drivetrain.resetOdometry(pose.getRotation().times(-1.0));
        m_poseEstimator.resetPosition(m_drivetrain.getGyro().times(1.0), m_drivetrain.getModulePositions(), pose);
    }

}
