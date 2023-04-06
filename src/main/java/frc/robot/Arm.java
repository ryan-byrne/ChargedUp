package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Arm implements Sendable {
	// Initializing Variables
	private final CANSparkMax liftMotor;
	private final CANSparkMax extendMotor;
	private final WPI_VictorSPX leftClaw;
	private final WPI_VictorSPX rightClaw;

	ShuffleboardTab m_armTab;

	public Arm(
		int liftCanId, 
		int extendCanId, 
		int leftClawCanId,
		int rightClawCanId
	) {
		liftMotor = new CANSparkMax(liftCanId, CANSparkMaxLowLevel.MotorType.kBrushless); // controlled by CAN SparkMax
		extendMotor = new CANSparkMax(extendCanId,CANSparkMaxLowLevel.MotorType.kBrushless);// controlled by CAN SparkMax
		leftClaw = new WPI_VictorSPX(leftClawCanId);
		rightClaw = new WPI_VictorSPX(rightClawCanId);
		m_armTab = Shuffleboard.getTab("Arm");
		m_armTab.add("Telemetry", this);

	}

	public double getExtensionSpeed(){
		return extendMotor.getEncoder().getVelocity();
	}	

	public double getLiftSpeed(){
		return liftMotor.getEncoder().getVelocity();
	}

	public double getExtensionLength(){
		return extendMotor.getEncoder().getPosition();
	}

	public double getLiftAngle(){
		return liftMotor.getEncoder().getPosition();
	}

	public void setExtensionSpeed(double speed){
		if ( this.getExtendDisabled() ){
			extendMotor.set(0);
		} else {
			extendMotor.set(speed);
		}
	}

	public void setExtension(double extension){
		if (extension > this.getExtensionLength()){
			while (this.getExtensionLength()<extension){
				this.setExtensionSpeed(100);
			}
		}
		else {
			while (this.getExtensionLength()>extension){
				this.setExtensionSpeed(-100);
			}
		}
		
	}

	public void setTarget(String type, int level){
		if (type == "cone" && level == 2){
			this.setLiftAngle(44);
			this.setExtension(-31);
		}
		else if (type == "cone" && level == 1){
			this.setLiftAngle(41);
			this.setExtension(-15);
		}
		else if (type == "cube" && level == 2){
			this.setLiftAngle(39);
			this.setExtension(-19);
		}
		else if (type == "cube" && level == 1){
			this.setLiftAngle(36);
		}
	}

	public void setLiftAngle(double angle){
		if (angle > this.getLiftAngle()){
			while (this.getLiftAngle()<angle){
				this.setLiftSpeed(100);
			}
		}
		else {
			while (this.getLiftAngle()>angle){
				this.setLiftSpeed(100);
			}
		}
	}

	public void setLiftSpeed(double speed){
		liftMotor.set(speed);
	}

	public void setIntake(double speed){
		leftClaw.set(speed);
		rightClaw.set(-speed);
	}

	public boolean getExtendDisabled(){
		return (this.getLiftAngle() > 50) || (this.getLiftAngle() < 20);
	}

	public boolean getLiftDisabled(){
		return false;
	}

	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Arm");
		builder.addBooleanProperty("extend-disabled", this::getExtendDisabled, null);
		builder.addDoubleProperty("lift-angle", this::getLiftAngle, null);
		builder.addDoubleProperty("extension", this::getExtensionLength, null);
	}
}