package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;


public class ZeroClimber extends Command{
    
    private final Climber m_climber;
    private boolean m_leftDone = false;
    private boolean m_rightDone = false;
    private final Timer m_timer = new Timer();
    public ZeroClimber(Climber climber){
        m_climber = climber;
        addRequirements(m_climber);

    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_leftDone = false;
        m_rightDone = false;
        m_climber.zeroLeft();
        m_climber.zeroRight();

    }

    @Override
    public void execute() {
        if(m_timer.get()> 0.100){
            if(m_climber.getLeftCurrent()>20.0){
                m_climber.setLeftZero();
                m_climber.stopLeft();
                m_leftDone = true;
            }
            if(m_climber.getRightCurrent()>20.0){
                m_climber.setRightZero();
                m_climber.stopRight();
                m_rightDone = true;
            }
        }


        
    }

    @Override
    public boolean isFinished() {
        return (m_leftDone && m_rightDone);
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.run();
        m_climber.setPose(0.5);
        m_timer.stop();
    }

}
