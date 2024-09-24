package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private TalonFX m_leftKraken = new TalonFX(12,"rio");
    private TalonFXConfigurator m_leftTalonFXConfigurator = m_leftKraken.getConfigurator();
    private MotorOutputConfigs m_leftMotorConfigs = new MotorOutputConfigs();
    private StatusSignal<Double> m_leftCurrentSignal = m_leftKraken.getTorqueCurrent();
    private StatusSignal<Double> m_leftVeloSignal = m_leftKraken.getVelocity();

    private TalonFX m_rightKraken = new TalonFX(11,"rio");
    private TalonFXConfigurator m_rightTalonFXConfigurator = m_rightKraken.getConfigurator();
    private MotorOutputConfigs m_rightMotorConfigs = new MotorOutputConfigs();
    private StatusSignal<Double> m_rightCurrentSignal = m_rightKraken.getTorqueCurrent();
    private StatusSignal<Double> m_rightVeloSignal = m_rightKraken.getVelocity();

    private VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private Slot0Configs m_PIDSlot0Configs = new Slot0Configs();
    private CurrentLimitsConfigs m_currentLimitsConfigs = new CurrentLimitsConfigs();
    
    private double m_shooterVelo = 0.0;
    private boolean m_shooterPIDEnabled = false;

    

    public Shooter(){

        m_currentLimitsConfigs.StatorCurrentLimit = 100.0;
        m_currentLimitsConfigs.StatorCurrentLimitEnable = true;
        m_currentLimitsConfigs.SupplyCurrentLimit = 50.0;
        m_currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        m_PIDSlot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        m_PIDSlot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        m_PIDSlot0Configs.kA = 0.0;
        m_PIDSlot0Configs.kP = 0.01; // An error of 1 rps results in 0.01 V output
        m_PIDSlot0Configs.kI = 0; // no output for integrated error
        m_PIDSlot0Configs.kD = 0; // no output for error derivative

        m_leftTalonFXConfigurator.apply(new TalonFXConfiguration());
        //m_leftMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_leftMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        m_leftTalonFXConfigurator.apply(m_PIDSlot0Configs);
        m_leftTalonFXConfigurator.apply(m_currentLimitsConfigs);
        m_leftTalonFXConfigurator.apply(m_leftMotorConfigs);

        m_rightTalonFXConfigurator.apply(new TalonFXConfiguration());
        //m_rightMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_rightMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        m_rightTalonFXConfigurator.apply(m_PIDSlot0Configs);
        m_rightTalonFXConfigurator.apply(m_currentLimitsConfigs);
        m_rightTalonFXConfigurator.apply(m_rightMotorConfigs);

    }

    /**
     * Sets the Velocity of the Shooter's Krakens taking in RPM and giving the Krakens RPS
     * as well as enables the PID
     * @param shooterVelo Velocity in RPM
     * 
     */
    public void setShooterVelo(double shooterVelo){
        m_shooterPIDEnabled = true;
        m_shooterVelo = shooterVelo;
    }

    public void RunShooter(){
        m_shooterPIDEnabled = true;
    }

    public void setVelo(double RPS){
        m_shooterVelo = RPS;
    }

    public double getLeftMotorCurrent(){
        m_leftCurrentSignal.refresh();
        return m_leftCurrentSignal.getValue();
    }

    public double getRightMotorCurrent(){
        m_rightCurrentSignal.refresh();
        return m_rightCurrentSignal.getValue();
    }

    public void stopShooter(){
        m_leftKraken.stopMotor();
        m_rightKraken.stopMotor();
        m_shooterPIDEnabled = false;
    }

    public double getVelocity(){
        m_rightVeloSignal.refresh();
        m_leftVeloSignal.refresh();

        return (m_leftVeloSignal.getValue() + m_rightVeloSignal.getValue())/2.0;
    }

    public Command incrementVelo(){
    return new InstantCommand(()-> setVelo( m_shooterVelo + 3.0));
  }

  public Command decrementVelo(){
    return new InstantCommand(()-> setVelo(m_shooterVelo - 3.0));
  }
        

    @Override
    public void periodic() {
        
        if(m_shooterPIDEnabled){
            m_leftKraken.setControl(m_request.withVelocity(m_shooterVelo));
            m_rightKraken.setControl(m_request.withVelocity(-1.0*m_shooterVelo));
        } else {
            m_leftKraken.stopMotor();
            m_rightKraken.stopMotor();
        }

        SmartDashboard.putNumber("Target Shooter Velo.",m_shooterVelo);
        SmartDashboard.putNumber("Actual Left Motor Velo.",m_leftKraken.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Actual Right Motor Velo.",m_rightKraken.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Left Motor Current",m_leftKraken.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Current",m_rightKraken.getTorqueCurrent().getValueAsDouble());
        

        

    }

}
