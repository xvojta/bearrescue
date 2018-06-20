package stormbots;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.text.NumberFormat;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;

public class AutoMedved {

	public static void main(String[] args) throws IOException, InterruptedException {
		StopButton stopButton = new StopButton(SensorPort.S1);
		stopButton.start();

		EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S4);


		// UART sensor pro LIDAR
		Port p2 = LocalEV3.get().getPort("S2");
		Lidar lidar = new Lidar(p2);
		lidar.start();

		// Vypisovani hodnoty lidaru na display, konci stiskem enter nebo escape
		float[] distances = new float[irSensor.sampleSize()];
		LCD.clear();
        while(Button.ESCAPE.isUp() && Button.ENTER.isUp()) {
        	irSensor.fetchSample(distances,0);
        	int lidarDist = lidar.getActualDistance();
			LCD.drawString("Lidar dist: " + (lidarDist<0?"ERROR":lidarDist) + "    ", 2, 3);
			LCD.drawString("IR dist: " + distances[0] + "    ", 2, 5);
			Thread.sleep(5);
        }
        Sound.playTone(440, 200);
        Thread.sleep(300);

		bludiste(irSensor);
		
		LCD.clear();
		int SCAN_ANGLE = 160;
		int bearOnAngle = bearDetection(scan(SCAN_ANGLE, lidar));
		LCD.drawString("Bear angle:" + bearOnAngle + "   ", 2, 5);
	
		if (bearOnAngle >= 0) {
	        Sound.playTone(440, 100);
	        Sound.playTone(660, 100);
	        Sound.playTone(880, 100);
			rotateRobotAngle(-SCAN_ANGLE + bearOnAngle);
			jizdaKMedvedovi(lidar);
			
			// navrat domu
			rotateRobotAngle(140); // otocit se zpet, ale ne uplne, abychom se nesekli o roh u vjezdu
	        if (bearOnAngle < 30){
	        	// pokud sebere medveda hned u leve zdi, je potreba zatocit vic, aby zustal spravne
				moveSteeringBC(13, 50, 0, true);  // vracime se mirnou zatackou
	        } else if (bearOnAngle < 80){
	        	moveSteeringBC(4, 50, 0, true);  // vracime se mirnou zatackou
	        } else {
	        	moveSteeringBC(-3, 50, 0, true);  // je to proti vychodu, tocime se spis doleva
	        }
	        
			// dojet k nejblizsi zdi
	        int maxWayToWall = Motor.B.getTachoCount()+5*360;
			do {
				irSensor.fetchSample(distances,0);
			} while (distances[0]>30 && Motor.B.getTachoCount() < maxWayToWall);
			moveSteeringBC(0, 0, 0, false);

			// test, zda jsme se nesekli u zdi, pokud ano udelame vpravo vbok
			if (Motor.B.getTachoCount() >= maxWayToWall){
				rotateRobotAngle(90);
			}
			
			
			// podel zdi domu
			jedPodelZdi(20, irSensor, true);
			
		} else {
			// nenasli jsme medveda, co dal? Vydat se treba podel zdi, a zkusit ho nabrat cestou, nekde tam asi bude?
	        Sound.playTone(660, 100);
	        Sound.playTone(440, 100);
	        Sound.playTone(220, 100);
	        
			// otocit se zpet ke zdi
			rotateRobotAngle(-SCAN_ANGLE + 10);
			
			// otevrit prave chapadlo, at toho sebereme co nejvic
			zavritPraveChapadlo(false);
			
			// jet podel leve zdi a lehce se k ni tlacit - az do rohu (odhadneme podle otacek)
			moveSteeringBC(-3, 30, 5, false);
			
			// otocit se doprava 
			rotateRobotAngle(90);
			if (bearIsCatched(lidar)){
				// pokud uz medveda mame, jedeme rovnou domu
				zavritPraveChapadlo(true);
				rotateRobotAngle(90);
				moveSteeringBC(3, 30, 6, false);
				rotateRobotAngle(90);
			} else {
				// medveda nemame, pokracujeme podel dalsi zdi
				moveSteeringBC(-3, 30, 6, false);

				// otocit se doprava a jet podel dalsi zdi
				rotateRobotAngle(90);
				if (bearIsCatched(lidar)){
					// pokud uz medveda mame, jedeme rovnou domu
					zavritPraveChapadlo(true);
					rotateRobotAngle(40);
					moveSteeringBC(0, 30, 7, false);
					rotateRobotAngle(45);
				} else {
					// medveda nemame, pokracujeme podel dalsi zdi
					moveSteeringBC(-3, 30, 6, false);

					// otocit se doprava a jet podel dalsi zdi
					rotateRobotAngle(90);
					moveSteeringBC(-3, 30, 7, false);

					zavritPraveChapadlo(true);
				}
					
			}

			
			// podel zdi domu
			jedPodelZdi(20, irSensor, true);
		}

		
		
        // Opakovane skenovani / dalsi sken enter / konec skenovani escape
//        File file = new File("measurements.txt");
//        if(!file.exists()) {
//        	file.createNewFile();
//        }
        //FileWriter w = new FileWriter("measurements.txt");
//		PrintWriter writer = new PrintWriter(new FileOutputStream(
//        	    new File("measurements.txt"), 
//        	    true /* append = true */));

//        while(Button.ESCAPE.isUp()) {
//        	int[] s = scan(150, lidar);
//        	for(int i = 0; i < s.length; i++) {
//        		writer.print(s[i] + ",");
//        	}
//        	writer.println("");
//        	
//        	while(Button.ENTER.isUp() && Button.ESCAPE.isUp());	
//        }

//        writer.close();
/*		while (Button.ESCAPE.isUp()){			
				int distance = lidar.getActualDistance();
				
				Thread.sleep(20);
		}
*/
		lidar.close(); 
				
	}
	
	private static void zavritPraveChapadlo(boolean zavrit) throws InterruptedException{
		// zavrit prave chapadlo, at toho sebereme co nejvic
		Motor.A.setSpeed(750);
		if (zavrit) {
			Motor.A.backward();
		} else {
			Motor.A.forward();
		}
		Thread.sleep(250);
		Motor.A.flt();
	}
	
	private static void bludiste(EV3IRSensor irSensor) {
		moveSteeringBC(0, 100, 2.8, false); // prvni rovinka
		moveSteeringBC(30, 100, 3.6, false); // otocit do protismeru
		moveSteeringBC(0, 100, 1.0, false);  // druha rovinka
		moveSteeringBC(-30, 100, 3.2, false); // otocit na cil
		moveSteeringBC(0, 100, 2.1, false); // treti rovinka
		jedPodelZdi(104.85, irSensor, false);
		
		moveSteeringBC(-100, 20, 0.3, false); // otocit pred skenem
	}

	public static void moveSteeringBC(double steering, int power, double rotation, boolean nepretrzite) {
		if (nepretrzite) rotation = 1; // aby fungovala spravne nepretrzita jizda
		steering = Math.max(-100 ,Math.min(steering, 100));
		power = Math.max(-80 ,Math.min(power, 80));
		double speedB = 0;
		double speedC = 0;
		double rotationB = 0;
		double rotationC = 0;
		if(steering >= 0) {
			speedB = power*10*Math.signum(rotation);
			speedC = ((50 - steering) * power/50)*10*Math.signum(rotation);
			rotationB = rotation*360 + Motor.B.getTachoCount();
		} else {
			speedB = ((50 + steering) * power/50)*10*Math.signum(rotation);
			speedC = power*10*Math.signum(rotation);
			rotationC = rotation*360 + Motor.C.getTachoCount();
		}
		Motor.B.setSpeed((int)speedB);
		Motor.C.setSpeed((int)speedC);

		if (speedB >= 0) {
			Motor.B.forward();
		} else {
			Motor.B.backward();
		}
		if (speedC >= 0) {
			Motor.C.forward();
		} else {
			Motor.C.backward();
		}
		
		if (!nepretrzite) {
			if (steering >= 0) {
				while ((rotation >=0 && Motor.B.getTachoCount() < rotationB) ||
					   (rotation < 0 && Motor.B.getTachoCount() > rotationB))
				{
					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
				Motor.B.flt(true);
				Motor.C.flt(false);
			} else {
				while ((rotation >=0 && Motor.C.getTachoCount() < rotationC) ||
					   (rotation < 0 && Motor.C.getTachoCount() > rotationC))
				{
					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
				Motor.C.flt(true);
				Motor.B.flt(false);
			}
		}
	}
	
	public static boolean bearIsCatched(Lidar lidar){
		int lidarDist = lidar.getActualDistance();
		return lidarDist <= 15 && lidarDist >= 0;
	}
	
	public static void jedPodelZdi(double rotations, EV3IRSensor sensor, boolean doAutoFinish) {
		double startingTachoCount = Motor.C.getTachoCount();
		double rotationC = startingTachoCount + rotations*360;
		double steering;
		double WALL_DISTANCE = 45.0;
		int finishCounter = 0;
		int maxFinishCounter = 0;
		
		//PID pid = new PID(-2.5, -0.0001, -10);
		PID pid = new PID(-1.8, -0.005, -10);
        float[] distances = new float[sensor.sampleSize()];
        int pocitadlo = 0;
        NumberFormat formatter = new DecimalFormat("#0.00"); 
        sensor.fetchSample(distances,0);
        pid.nextValue(distances[0]-WALL_DISTANCE);
        
		while(Motor.C.getTachoCount() < rotationC) {
			sensor.fetchSample(distances, 0);
			if (Double.isInfinite(distances[0])) {
				distances[0] = 65;
				finishCounter++;
				if (doAutoFinish && finishCounter > maxFinishCounter){
					maxFinishCounter = finishCounter;
					LCD.drawString("Max finish:" + maxFinishCounter + "   ", 2, 1);
				}
				if (doAutoFinish && finishCounter > 20 && (Motor.C.getTachoCount() - startingTachoCount)>3*360){
					// detekovali jsme finish - posledni otacky do cile
					rotationC = Motor.C.getTachoCount() + 5.5*360;
					doAutoFinish = false; // aby se porad neprodluzovala dojezdna trasa
					LCD.drawString("Max fin:" + maxFinishCounter + " FINISH!!!  ", 2, 1);
					moveSteeringBC(-30, 40, 2.5, false);
				}
			} else {
				finishCounter = 0;
			}
			steering = -pid.nextValue(distances[0]-WALL_DISTANCE);
			moveSteeringBC(steering, 40, 0, true);
			LCD.drawString("steering: " + formatter.format(steering) + "     ", 2, 2);
			LCD.drawString("dist: "+ formatter.format(distances[0]) + "     ", 2, 4);
			LCD.drawInt(pocitadlo, 2, 6);
			pocitadlo ++;
		}
		Motor.C.stop();
		Motor.B.stop();
	}
	
	public static double robotDegToMotorDeg(double robotDegrees) {
		return robotDegrees*17./6.5;//*111*8/360;
	}
	
	public static int[] scan(int scanDegrees, Lidar lidar) throws IOException {
		// provadi scan zadanym lidarem v zadanem rozsahu po kroku 0.5 stupnu
		int[] scanValues = new int[2*scanDegrees+1];
//		int[] tachoValues = new int[2*scanDegrees+1];
//		double tachoRange = robotDegToMotorDeg(scanDegrees);
//		int stopTacho = (int)tachoRange + Motor.B.getTachoCount();
		//prvni scan
		scanValues[0] = lidar.getActualDistance();
		
		Motor.B.setSpeed(80);
		Motor.C.setSpeed(80);
		Motor.B.setAcceleration(3000);
		Motor.C.setAcceleration(3000);
		Motor.B.forward(); 
		Motor.C.backward();
		int i = 1;
		double nextScanTacho = Motor.B.getTachoCount() + robotDegToMotorDeg(0.5);
		while (/*Motor.B.getTachoCount() < stopTacho*/i < scanValues.length ) {
			int tc = Motor.B.getTachoCount();
			if(tc >= nextScanTacho) {
				scanValues[i] = lidar.getActualDistance();
//				tachoValues[i] = tc;
				nextScanTacho = nextScanTacho + robotDegToMotorDeg(0.5);
				i++;
			} else {
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
		Motor.B.stop(true);
		Motor.C.stop(false);
		
		return scanValues;
		
	}
	
	static void rozevritSevritChapadla(boolean rozevrit) throws InterruptedException {
		Motor.A.setSpeed(750);
		Motor.D.setSpeed(750);
		if(rozevrit) {
			Motor.A.forward();
			Thread.sleep(250);
			Motor.A.stop();
			Thread.sleep(30);
			Motor.D.backward();
			Thread.sleep(250);
			Motor.D.flt();	
			Motor.A.flt();
		} else {
			Motor.D.forward();
			Thread.sleep(250);
			Motor.D.stop();
			Thread.sleep(30);
			Motor.A.backward();
			Thread.sleep(250);
			//Motor.A.stop();	
			Motor.A.flt();
			Motor.D.flt();
		}
	}
	
	static void jizdaKMedvedovi(Lidar lidar) throws InterruptedException {
		int actualRange;
		int minRange = 100;
		float bTachocount = Motor.B.getTachoCount();
		//moveSteeringBC(0, 30, -0.277, false); nebudeme couvat, je to zbytecne a navic nam to zmeni natoceni robota, kdyz se opre o mantinel
		rozevritSevritChapadla(true);
		moveSteeringBC(0, 25, 0, true);
		while (lidar.getActualDistance() > 15 && (Motor.B.getTachoCount()-bTachocount)/360 <= 7) {
			actualRange = lidar.getActualDistance();
			if (!(actualRange > minRange + 7)) {
				minRange = actualRange;				
			}
			LCD.drawInt(actualRange, 8, 5);
			LCD.drawInt(minRange, 8, 3);
		}
		moveSteeringBC(0, 0, 0, false);
		
		if ((Motor.B.getTachoCount()-bTachocount)/360 > 7) {
			// dojeli jsme ke zdi, je potreba trosku couvnout, aby sly zavrit chapadla
			moveSteeringBC(0, 20, -0.2, false);
		}
		
		rozevritSevritChapadla(false);
	}
	
	static int bearDetection(int[] scan) {
		int[] diference = new int[scan.length - 1];
		for (int i = 1; i < scan.length; i++) {
			diference[i-1] = scan[i] - scan[i-1];
		}
		
		int[] okynko1Dif = new int[scan.length - 1];
		int sirkaOkynka = 5;
		for (int i = 1; i < okynko1Dif.length; i++) {
			okynko1Dif[i-1] = 0;
		}
		for (int i = 1; i < scan.length - sirkaOkynka; i++) {
			for (int j = 0; j < sirkaOkynka; j++) {
				okynko1Dif[i-1] = okynko1Dif[i-1] + diference[i-1+j];
			}
		}		
		
		int[] diference2 = new int[okynko1Dif.length];
		for (int i = 1; i < okynko1Dif.length; i++) {
			diference2[i-1] = okynko1Dif[i] - okynko1Dif[i-1];
		}

		int[] okynko2Dif = new int[diference2.length - 1];
		for (int i = 1; i < okynko2Dif.length; i++) {
			okynko2Dif[i-1] = 0;
		}
		for (int i = 1; i < diference2.length - sirkaOkynka; i++) {
			for (int j = 0; j < sirkaOkynka; j++) {
				okynko2Dif[i-1] = okynko2Dif[i-1] + diference2[i-1+j];
			}
		}		
		
		int maxValueIndex=0;
		int maxValue=okynko2Dif[0];
		for (int i = 1; i < okynko2Dif.length; i++){
			if (maxValue < okynko2Dif[i]) {
				maxValue = okynko2Dif[i];
				maxValueIndex = i;
			}
		}
		
		LCD.drawString("Max val:" + maxValue + " / " + maxValueIndex / 2 + sirkaOkynka/2 + "   ", 2, 4);
		
		// overeno empiricky na prazdnem hristi, ze pres 20 nejde - je treba nastavit podle hriste na soutezi
		int DETECTION_TRESHOLD = 24;
		if (maxValue > DETECTION_TRESHOLD){ 
			// hodnoty jsou po pul stupni, takze delime dvema, abychom ziskali stupne
			// koriguji jeste uhel sirku okynka
			return maxValueIndex / 2 + sirkaOkynka; 
		} else {
			return -1;
		}
	}
	
	static void rotateRobotAngle(double angle) {
		double nextScanTacho;
		double testAngle;
		Motor.B.setSpeed(80);
		Motor.C.setSpeed(80);
		Motor.B.setAcceleration(3000);
		Motor.C.setAcceleration(3000);
		
		nextScanTacho = Motor.B.getTachoCount() + robotDegToMotorDeg(angle);
		testAngle = nextScanTacho - Motor.B.getTachoCount();
		if(robotDegToMotorDeg(angle) > 0) {
			Motor.B.forward(); 
			Motor.C.backward();
		} else {
			Motor.C.forward(); 
			Motor.B.backward();
		}
		
		if(robotDegToMotorDeg(angle) > 0) {
			while(nextScanTacho > Motor.B.getTachoCount());
		} else {
			while(nextScanTacho < Motor.B.getTachoCount());			
		}
		Motor.B.flt();
		Motor.C.flt();
	}
}


