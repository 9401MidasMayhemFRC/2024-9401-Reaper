package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.RackPinion;
import frc.robot.subsystems.Shooter;

public class AimShooterAuto extends Command {

    private final Shooter m_shooter;
    private final RackPinion m_rackPinion;
    private final Feeder m_feeder;
    private final Drivetrain m_drivetrain;
    private boolean m_finshed = false;
    private Timer m_timer = new Timer();
    private final InterpolatingDoubleTreeMap m_distanceTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_angleTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_veloTable = new InterpolatingDoubleTreeMap();
    private final double m_angleAdjustment = 0.25;
    private final PIDController m_rotPID = new PIDController(0.09,0.02,0);
    
    public AimShooterAuto(Shooter shooter, RackPinion rackPinion, Feeder feeder, Drivetrain drivetrain){

        m_shooter = shooter;
        m_rackPinion = rackPinion;
        m_feeder = feeder;
        m_drivetrain = drivetrain;

        m_distanceTable.put(18.875,38.0);
        m_distanceTable.put(14.61,50.0);
        m_distanceTable.put(11.38,62.0);
        m_distanceTable.put(8.51,74.0);
        m_distanceTable.put(6.615,86.0);
        m_distanceTable.put(5.425,98.0);
        m_distanceTable.put(4.195,110.0);
        m_distanceTable.put(2.965,122.0);
        m_distanceTable.put(1.905,134.0);
        m_distanceTable.put(1.05,146.0);
        m_distanceTable.put(0.34,158.0);
        m_distanceTable.put(-0.22,170.0);
        m_distanceTable.put(-1.06,182.0);
        m_distanceTable.put(-1.62,194.0);
        m_distanceTable.put(-2.03,206.0);
        m_distanceTable.put(-2.34,218.0);
        m_distanceTable.put(-2.69,230.0);
        m_distanceTable.put(-2.83,242.0);
        m_distanceTable.put(-3.14,254.0);


        m_angleTable.put(38.0,(48.07 + m_angleAdjustment));
        m_angleTable.put(50.0,(42.06 + m_angleAdjustment));
        m_angleTable.put(62.0,(36.05 + m_angleAdjustment));
        m_angleTable.put(80.0,(31.76 + m_angleAdjustment));
        m_angleTable.put(98.0,(26.21 + m_angleAdjustment));
        m_angleTable.put(116.0,(21.67 + m_angleAdjustment));
        m_angleTable.put( 134.0, (16.95 + m_angleAdjustment));
        m_angleTable.put(152.0,(13.95 + m_angleAdjustment));
        m_angleTable.put(178.0,(12.02 + m_angleAdjustment));
        m_angleTable.put(188.0,(8.58 + m_angleAdjustment));
        
        m_veloTable.put(38.0,37.23);
        m_veloTable.put(50.0,40.24);
        m_veloTable.put(62.0,42.81);
        m_veloTable.put(80.0,45.39);
        m_veloTable.put(98.0,47.96);
        m_veloTable.put(116.0,54.83);
        m_veloTable.put( 134.0, 66.42);
        m_veloTable.put(152.0,70.28);
        m_veloTable.put(178.0,78.00);
        m_veloTable.put(188.0,90.45);

        addRequirements(m_shooter,m_rackPinion,m_feeder,m_drivetrain);

    }

    @Override
    public void initialize() {
        m_finshed = false;
        m_timer.reset();
        m_timer.start();
        
    }

    @Override
    public void execute() {
    double ty = LimelightHelpers.getTY("")-1.31;
    double tx = LimelightHelpers.getTX("");
    boolean tv = LimelightHelpers.getTV("");
    double dist = m_distanceTable.get(ty);
    double desiredRot;

    
    if(tv){
        m_rackPinion.setPose(m_angleTable.get(dist));
        m_shooter.setShooterVelo(m_veloTable.get(dist));
        desiredRot = m_rotPID.calculate(tx);
    } else {
        m_rackPinion.setPose(m_angleTable.get(38.0));
        m_shooter.setShooterVelo(m_veloTable.get(38.0));
        desiredRot = 0.0;
    }

    if ((m_timer.get() >= .60)){
         m_feeder.setFeedVelo(.40);
        if (m_timer.get() >= 1.25){
            m_finshed = true;
        }
    }

    m_drivetrain.drive(0,0,desiredRot,false,false);

    }

    @Override
    public void end(boolean interrupted) {
                m_shooter.setShooterVelo(0.0);
                m_feeder.setFeedVelo(0.0);
                m_rackPinion.setPose(5.0);

    }

    @Override
    public boolean isFinished() {
        return m_finshed;
    }

}
