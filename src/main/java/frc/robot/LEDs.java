
package frc.robot;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import java.util.timer;


public class LEDs {
	boolean bit1;
	boolean bit2;
	boolean bit3;
		public LEDs (){
			bit1 = this.bit1;
			bit2 = this.bit2;
			bit3 = this.bit3
		}
		// to go with robotinit
		public void LEDsetup(){
			DigitalOutput bit1 = new DigitalOutput(4);
			 DigitalOutput bit2 = new DigitalOutput(6);
				DigitalOutput bit3 = new DigitalOutput(7);
				bit1.set(false);
				bit2.set(false);
				bit3.set(false);
				//set all pins to low for startup
			
		}

		public void autofade(){
			bit1.set(true);
			bit2.set(false);
			bit3.set(true);
		}

		public void LEDmain(){
			 operatorController.getPOV(int pov);
			 if (pov == 0){
				bit1.set(true);
				bit2.set(false);
				bit3.set(false);
			 }else if(pov == 90){
				bit1.set(false);
				bit2.set(true);
				bit3.set(false);
			 }else if(pov == 270){
				bit1.set(true);
				bit2.set(true);
				bit3.set(false);
			 }else if(pov == 180){
				bit1.set(false);
				bit2.set(false);
				bit3.set(true);
			 }else if (DriverStation.getinstance().getBatteryVoltage() < 7.2){
				bit1.set(true);
				bit2.set(true);
				bit3.set(true);
			 }else{
				bit1.set(false);
				bit2.set(false);
				bit3.set(false);
			 }
		}






	}
	
