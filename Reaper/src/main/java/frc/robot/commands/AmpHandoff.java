package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RackPinion;
import frc.robot.subsystems.ThriftyIntake;
import frc.robot.utilities.LEDs;

public class AmpHandoff extends Command{
    
    private Amp m_amp;
    private Intake m_intake;
    private Elevator m_elevator;
    private RackPinion m_rackPinion;
    private Timer m_timer = new Timer();
    private boolean m_finished = false;
    private boolean m_noteDetected = false;
    private LEDs m_leds;

    public AmpHandoff(Amp amp, Intake intake, Elevator elevator, LEDs leds, RackPinion rackPinion){

        m_leds = leds;
        m_amp = amp;
        m_intake = intake;
        m_elevator = elevator;
        m_rackPinion = rackPinion;
        addRequirements(m_amp, m_intake, m_elevator, m_rackPinion);

    }

    @Override
    public void initialize() {
        m_finished = false;
        m_noteDetected = false;
        m_elevator.setPosition(0.5);
        m_amp.setSpeed(0.35);
        m_rackPinion.setPose(5.0);
        m_timer.reset();
        m_timer.start();
        LimelightHelpers.setPipelineIndex("limelight-note", 1);
    }

    @Override
    public void execute() {

        if (m_elevator.getPosition() <= 1.0){
            m_intake.run(0.80);
        }

        if((m_timer.get() > 0.5) && (m_amp.getCurrent() > 20.0 || LimelightHelpers.getTV("limelight-note")) && !m_noteDetected){ 
            m_noteDetected = true;
            m_timer.reset();
            m_amp.stop();
        }
            
        if( m_timer.get() >= 0.045 && m_noteDetected){
                m_finished = true;
        }
        
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }

    @Override
    public void end(boolean interrupted) {

        if(m_noteDetected){
            m_leds.setWingo();
        }

        m_amp.stop();
        m_intake.stop();
        m_elevator.setPosition(ElevatorConstants.kRestPose);
        LimelightHelpers.setPipelineIndex("limelight-note", 0);

    }

}
