package actr.tasks.driving;

import java.awt.BorderLayout;
import java.io.*;
import java.io.File; 

// import java.util.Arrays;
// import java.io.BufferedWriter;
// import java.io.FileWriter;
// import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.Vector;
import javax.swing.JLabel;

import actr.model.Model;
import actr.task.Result;
import actr.task.Task;

/**
 * The main Driving task class that sets up the simulation and starts periodic updates.
 *  
 * @author Dario Salvucci
 */

/* TODO
Speed sign location : make it appear earlier/bigger 	moves super fast now 
parallel to road end
Conflicts instruction/speedsigns/speedo
java internal width of lanes smaller
	different lane widths even within condition
smaller lanes
block switching + additional nback levels in array 4	2	1	3	2	0	3	4	0	1
*/

public class Driving extends actr.task.Task
{
	static Simulator simulator = null;

	Simulation simulation;
	JLabel nearLabel, farLabel, speedsign, instructions, warning, speedometer;

	final double scale = .6; // .85
	final double steerFactor_dfa = (16 * scale);
	final double steerFactor_dna = (4 * scale);
	final double steerFactor_na  = (3 * scale);
	final double steerFactor_fa  = (0 * scale);
	final double accelFactor_thw  = (1 * .40);
	final double accelFactor_dthw = (3 * .40);
	final double steerNaMax = .07;
	final double thwFollow = 1.0;
	final double thwMax = 4.0;

	double startTime=0, endTime=3630;					// STOPS EVERYTHING
	double accelBrake=0, speed=0;

	static int minX=174, maxX=(238+24), minY=94, maxY=(262+32);
	static int centerX=(minX+maxX)/2, centerY=(minY+maxY)/2; 

	//make it bigger if the experiment is longer than 60s or signs appear more often
	static String currentLimit = "60"; //starting speed
	String previousLimit = "60";
	int nback_count = 0;
	double signOnset = 0; 
	double instructionsOnset = -5;		
	double warningOnset = 0;
	boolean signSeen = false; 
	static boolean instructionsSeen = false;
	static boolean warningSeen = false;
	static int speedI = 0;
	static Coordinate signPos;
	static String currentNBack = "";
	String[] nBack_list = {"2back", "3back", "0back", "1back", "4back", "0back", "3back", "4back", "1back", "2back"};
	String[] nBack_list_constr = {"2back", "3back", "0back", "1back", "4back", "0back", "3back", "4back", "1back", "2back"};
	double sign_count = 0;
	int rehearsal_count = 0;
	static String imaginedSpeedlimit = "60";
	List<String> output = new ArrayList<String>();	
	double acceptableSpeedDiff = 2; 
	double mentalSpeed = 16.66666667; 
	double roughSpeed; 
	double prevTime = 0; 
	double curTime; 
	double prevDist = 0; 
	double curDist; 	
	boolean newCSV = true; 
	String[] blocks = {"normal", "construction", "normal", "construction", "normal", "construction", "normal", "construction"}; 
	String curBlock = "construction"; 
	
	public Driving ()
	{
		super ();
		nearLabel = new JLabel (".");
		farLabel = new JLabel ("X");
		speedsign = new JLabel ("*");
		instructions = new JLabel (",");
		warning = new JLabel (";");
		speedometer = new JLabel("~");
	}

	public void start ()
	{
		simulation = new Simulation (getModel());
		
		if (getModel().getRealTime())
		{
			curBlock = blocks[nback_count]; 			
			
			setLayout (new BorderLayout());
			if (simulator == null) simulator = new Simulator ();
			add (simulator, BorderLayout.CENTER);
			simulator.useSimulation (simulation);
			signPos = null; 
			signOnset = 0;
			signSeen = false; 
			instructionsOnset = -5;
			instructionsSeen = false; 
			mentalSpeed = 16.666667; 		// 60 kmh
		}
		else
		{
			curBlock = blocks[nback_count]; 
			
			add (nearLabel);
			nearLabel.setSize (20, 20);
			nearLabel.setLocation (250, 250);
			add (farLabel);
			farLabel.setSize (20, 20);
			farLabel.setLocation (250, 250);
			add (speedsign);
			speedsign.setSize (20, 20);
			speedsign.setLocation (250, 250);
			add (instructions);
			instructions.setSize(20,20);
			instructions.setLocation(200, 100);
			add (warning);
			warning.setSize(20, 20);
			warning.setLocation(200, 50);
			add(speedometer);
			speedometer.setSize(20,20);
			speedometer.setLocation(300, 350);
		}

		getModel().runCommand ("(set-visual-frequency near .1)");
		getModel().runCommand ("(set-visual-frequency far .1)");

		//they don't reset otherwise if you run the model multiple times in a row
		accelBrake = 0;
		speed = 0;
		previousLimit = "60";
		currentLimit = "60";

		getModel().getVision().addVisual ("near", "near", "near", nearLabel.getX(), nearLabel.getY(), 1, 1, 10);
		getModel().getVision().addVisual ("far", "far", "far", farLabel.getX(), farLabel.getY(), 1, 1, 100);
		getModel().getVision().addVisual ("speedometer", "speedometer", "speedometer", 150, 315, 1, 1, 1);
		addPeriodicUpdate (Env.sampleTime);
		
		
	}

	public void update (double time)
	{
		if (time <= endTime)
		{
			simulation.env.time = time - startTime; 
			updateVisuals();
			if(simulation.env.time>10) //only start after 10s
			{
				updateSign(simulation.env.time);
			}
			updateInstructions(simulation.env.time);			// $$$$$$$$$$$$$$$$$$$$$$
			simulation.update();	
			updateSpeedometer();
		}
		else 
		{
			getModel().stop();
			output("myData", simulation.samples);
		}

	}

	//	save to file
	void output(String filename, Vector<Sample> samples)
	{
		for (int i = 1 ; i < samples.size(); i++)
		{
			Sample s = samples.elementAt(i);
			if(i == 1) {
				output.add(s.listVars() + System.lineSeparator());
				output.add(s.toString() + System.lineSeparator());
			}else
				output.add(s.toString() + System.lineSeparator());
			}
		Model.print(output, "_driving_");
	}

	void updateInstructions(double time)
	{

		//reverse order randomly for participants
		if (nback_count == 0 && Math.round(Math.random()) == 1)
		{
			
			//nBack_list_constr = new String[] {"2back", "3back", "0back",  "1back", "4back",  "0back", "3back",  "4back",  "1back",   "2back"}; 
			//keep the old order
		} else if(nback_count == 0)
		{
			//NBACKLIST ARRAY #2
			nBack_list = new String[] {"2back", "1back", "4back", "3back", "0back", "4back", "1back", "0back", "3back", "2back"};
			//nBack_list_constr = new String[] {"2back", "1back", "4back", "3back", "0back", "4back", "1back", "0back", "3back", "2back"};
		}

		//add instructions
		// now at 3 seconds and every 60 sec		x5 seconds works as long as signs are there every 20 sec
		
		if(!instructionsSeen && ((time > 5 && time < 7) || (time > 1 && time%60 < 1)))		// instr at 10s?
		{
			instructionsSeen = true;
			currentNBack = nBack_list[nback_count];
			getModel().getVision().addVisual("instructions", "instructions", currentNBack.substring(0,1), 300, 50, 50, 50, 0);
			instructions.setLocation(300, 50);
			System.out.println("instr at " + time + "\n");
			instructionsOnset = time;
			nback_count++;
		} else if((time - instructionsOnset >= 3) && instructionsOnset >= 0 && instructionsSeen)	// 2
		{
			getModel().getVision().removeVisual("instructions");
			instructionsSeen = false;
		}
	}

	void updateSign(double time)
	{
		String[] speedlimits = {"60", "70", "80", "90", "100", "110", "120", "130", "140"};		
		Env env = simulation.env;
		Simcar simcar = simulation.env.simcar;
		
		// time%20 before!			
		if((int)time%10 == 0 && !signSeen && (speedI < speedlimits.length) )
		{
			if (env.simcar.nearPoint != null)
			{	
				//pick a random speed with delta<30
				previousLimit = currentLimit;
				while (Math.abs(Integer.parseInt(currentLimit) - Integer.parseInt(previousLimit)) > 30 || (currentLimit == previousLimit))
				{
					int rnd = new Random().nextInt(speedlimits.length);
					currentLimit = speedlimits[rnd];
				} 

				Position newLoc = Road.location(env.simcar.fracIndex + 20 , 4);		// 20 fracIndex, laneposition
				newLoc.y = 0.0;
				signPos = env.world2image(newLoc);
				signSeen = true;
				getModel().getVision().addVisual ("speedsign", "speedsign", currentLimit, (int)signPos.x, (int)signPos.y, 1, 1, 100);
				speedsign.setLocation((int)signPos.x, (int)signPos.y);
				signOnset = time;
			} else
				System.out.println("nearpoint = null");
		// move the speed sign
		} else if (signSeen && (time - signOnset < 2) && signOnset > 0&& (int)Utilities.mps2mph(Utilities.mph2kph(simcar.speed))	>= 0) 
		{ 
			signPos.x += 15;
			signPos.y += 1;
			getModel().getVision().moveVisual("speedsign", (int)signPos.x, (int)signPos.y);
			speedsign.setLocation((int)signPos.x , (int)signPos.y);
			/*		 
			km/h	m/s				second before sign gone (100 m)
			60		16.66666667		6
			70		19.44444444		5.142857143
			80		22.22222222		4.5
			90		25				4
			100		27.77777778		3.6
			110		30.55555556		3.272727273
			120		33.33333333		3
			130		36.11111111		2.769230769
			  */
		} else if(signSeen && (time - signOnset >= 2) && signOnset > 0)
		{
			getModel().getVision().removeVisual("speedsign");
			signPos = null; 
			signSeen = false;
			
		// moving the speed sign while it's being shown
		}   
	}

	void updateSpeedometer()
	{
		Simcar simcar = simulation.env.simcar;
		String speed = Integer.toString((int)Utilities.mps2mph(Utilities.mph2kph(simcar.speed)));
		getModel().getVision().changeValue("speedometer", speed);
	}

	void updateVisuals ()
	{
		Env env = simulation.env;
		if (env.simcar.nearPoint != null)
		{
			Coordinate cn = env.world2image (env.simcar.nearPoint);
			Coordinate cc = env.world2image (env.simcar.carPoint);
			//Coordinate cc = env.world2image (env.simcar.farPoint);

			if (cn == null || cc == null) env.done = true;
			else
			{
				nearLabel.setLocation (cn.x, cn.y);
				farLabel.setLocation (cc.x, cc.y);
				getModel().getVision().moveVisual ("near", cn.x, cn.y);
				getModel().getVision().moveVisual ("far", cc.x, cc.y);
				//				speed = env.simcar.speed;

			}
		}
	}

	double minSigned (double x, double y)
	{
		return (x>=0) ? Math.min (x,y) : Math.max (x,-y);
	}

	void doSteer (double na, double dna, double dfa, double dt)
	{
		Simcar simcar = simulation.env.simcar;
		if (simcar.speed >= 10.0)
		{
			double dsteer = (dna * steerFactor_dna)
					+ (dfa * steerFactor_dfa)
					+ (minSigned (na, steerNaMax) * steerFactor_na * dt);
			dsteer *= simulation.driver.steeringFactor;
			simcar.steerAngle += dsteer;
		}
		else simcar.steerAngle = 0;
	}

	void doAccelerate (double fthw, double dthw, double dt, double tlimit)
	{
		Simcar simcar = simulation.env.simcar;
		if (simcar.speed >= 10.0)
		{
			double dacc = (dthw * accelFactor_dthw)
					+ (dt * (fthw - thwFollow) * accelFactor_thw);
			accelBrake += dacc;
			accelBrake = minSigned (accelBrake, 1.0);
		}
		else
		{
			accelBrake = .65 * (simulation.env.time / 3.0);
			accelBrake = minSigned (accelBrake, .65);
		}
		simcar.accelerator = (accelBrake >= 0) ? accelBrake : 0;
		simcar.brake = (accelBrake < 0) ? -accelBrake : 0;
	}

	/* the speedometer check is only used to update the mental representation of what speed we're driving at. 
	 * the original process far production is still the one responsible for maintaining the speed, 
	 * the only difference is that is used the mental representation of the speed to accelerate/brake
	 * this mental representation is mainly based on the distance travelled between far point checks, 
	 * and gets real accuracy when you check the speedometer. 
	 * the distance between far points is accurate, so add some normal distribution noise around it.  */
	
	void keepLimit(double slimit)
	{	
		imaginedSpeedlimit = Integer.toString((int)slimit); //for sampling
		Simcar simcar = simulation.env.simcar;
		double speed =  mentalSpeed; 		// simcar.speed
		slimit = Utilities.mph2mps(Utilities.kph2mph(slimit));
		//System.out.println("simcar speed: " + simcar.speed + "\tmental: " + mentalSpeed + "\tlimit: " + slimit); 
	
		double diff = (slimit - speed);
		double time = simulation.env.time - startTime;

		//display warning if the speed limit is not kept (method says 6 seconds (3 before and after))
		
		if( Math.abs(diff)>1.39 && !warningSeen && signOnset != 0 && (time - signOnset) > 3) //1.39 m/s is roughly 5 km/h
		{	
			warningOnset = time;
			getModel().getVision().addVisual("warning", "warning", "warning", 200, 100, 50, 50);
			warning.setLocation(200, 50);
			warningSeen = true;
		} else if (warningSeen && (time - warningOnset) > 3)
		{
			getModel().getVision().removeVisual("warning");
			//warningOnset = time;
			warningSeen = false;
		}
			
		if (Math.abs(diff) > 0)
		{
			//			double dacc = (dthw * accelFactor_dthw 1.2)
			//			+ (dt * (fthw - thwFollow 1.0) * accelFactor_thw 0.4);
			
			if(diff > 2) {	// 1
				double dacc = (diff * 1.2) + (0.25 * diff * 0.4); // (diff*10 * 1.2) + (0.25 * diff * 0.4) braking too hard
				accelBrake += dacc;
				accelBrake = minSigned (accelBrake, 1.0); // 1.5
			} else
			{
				double dacc = (diff* 1.2) + (0.25 * diff * 0.4);
				accelBrake += dacc;
				accelBrake = minSigned (accelBrake, 0.65);	// 0.65
			}	
		}
		simcar.accelerator = (accelBrake >= 0) ? accelBrake : 0;
		simcar.brake = (accelBrake < 0) ? -accelBrake : 0;
	}

	void makeCSVwithSpeeds(double real, double rough, double mental)
	{
		try {
	        FileWriter fw = new FileWriter("speeds.csv", true);
	        BufferedWriter bw = new BufferedWriter(fw);
	        PrintWriter pw = new PrintWriter(bw);	
	        if (newCSV)
	        {
	        	pw.println("Real, Rough, Mental, Limit, Real-Rough, Real-Mental, Correct-Real");
	        	newCSV = false; 
	        }
	        int corrSpeed = Integer.parseInt(imaginedSpeedlimit);
	        pw.println(real + "," + rough + "," + mental + "," + corrSpeed + "," + (real-rough) + "," + (real-mental) + "," + (corrSpeed-real));
	        pw.flush();
	        pw.close();

	    } catch (FileNotFoundException e) {
	        System.out.println(e.getMessage());
	    } catch (IOException e) {
	        e.printStackTrace();
	    }
	}
	
		
	boolean isCarStable (double na, double nva, double fva)
	{
		double f = 2.5;		// is this lane width? 
		return (Math.abs(na) < .025*f) && (Math.abs(nva) < .0125*f) && (Math.abs(fva) < .0125*f);
	}
	
	// na = near point, fva = far?? nva = ??		velocity??

	double image2angle (double x, double d)		// x = near/far angle, d = distance
	{
		Env env = simulation.env;
		double px = env.simcar.p.x + (env.simcar.h.x * d);
		double pz = env.simcar.p.z + (env.simcar.h.z * d);
		Coordinate im = env.world2image (new Position (px, pz));
		try { return Math.atan2 (.5*(x-im.x), 450); }		// google
		catch (Exception e) { return 0; }
	}

	public void eval (Iterator<String> it)
	{
		it.next();
		String cmd = it.next();
		if (cmd.equals ("do-steer"))
		{
			double na = Double.valueOf (it.next());
			double dna = Double.valueOf (it.next()); // crash
			double dfa = Double.valueOf (it.next());
			double dt = Double.valueOf (it.next());
			doSteer (na, dna, dfa, dt);
		}
		else if (cmd.equals ("do-accelerate"))
		{
			double fthw = Double.valueOf (it.next());
			double dthw = Double.valueOf (it.next());
			double dt = Double.valueOf (it.next());
			double tlimit = Double.valueOf(it.next());
			doAccelerate (fthw, dthw, dt, tlimit);
		}
		else if (cmd.equals ("keep-limit"))
		{
			// get the (imaginal) speed of the car
			double tlimit = Double.valueOf (it.next());
			keepLimit (tlimit);
		}
		else if (cmd.equals ("placeholder"))
		{
			// estimate the current speed based on how much distance was crossed since the last check
		} else if (cmd.equals("set-rough-speed"))
		{
			Simcar simcar = simulation.env.simcar;
			curTime = simulation.env.time; 						
			curDist = simcar.getDistance();
			
			double diffDist = curDist - prevDist; 
			double dTime = curTime - prevTime;
			if (dTime != 0)
				roughSpeed = diffDist / dTime;  			
			//System.out.println("dist: " + diffDist + " time: " + dTime); 
			//System.out.println("roughSpeed: " + Utilities.mph2kph(Utilities.mps2mph(roughSpeed)) + " vs actual speed: " + simcar.speed);
			
			Random r = new Random(); 
			double noise = r.nextGaussian() * 0.1;		// 1 mps is 3.6 km/h
			mentalSpeed = roughSpeed + noise; 
			
			prevTime = curTime; 
			prevDist = curDist; 
			makeCSVwithSpeeds(simcar.speed, roughSpeed, mentalSpeed);
			
			//System.out.println("mentalSpeed: " + mentalSpeed + " vs actual speed: " + simcar.speed);			
		} else if (cmd.equals("set-speedo-speed"))
		{
			double speedoSpeed = Double.valueOf (it.next());
			//System.out.println("speedospeed: " + Utilities.mph2mps(Utilities.kph2mph(speedoSpeed))); 
			mentalSpeed = Utilities.mph2mps(Utilities.kph2mph(speedoSpeed)); 
		}
	}

	public boolean evalCondition (Iterator<String> it)
	{
		it.next();
		String cmd = it.next();
		if (cmd.equals ("is-car-stable") || cmd.equals ("is-car-not-stable"))
		{
			double na = Double.valueOf (it.next());	
			double nva = Double.valueOf (it.next());
			double fva = Double.valueOf (it.next());
			boolean b = isCarStable(na,nva,fva);
			return cmd.equals("is-car-stable") ? b : !b;
		}
		else return false;
	}

	public double bind (Iterator<String> it)
	{
		try
		{
			it.next(); 
			String cmd = it.next();
			if (cmd.equals ("image->angle"))
			{
				double x = Double.valueOf (it.next());		// near/far point
				double d = Double.valueOf (it.next());		// distance
				return image2angle (x, d);
			}
			else if (cmd.equals ("mp-time")) return simulation.env.time;
			else if (cmd.equals ("get-thw"))
			{
				double fd = Double.valueOf (it.next());
				double v = Double.valueOf (it.next());
				double thw = (v==0) ? 4.0 : fd/v;
				System.out.println("fthw: " + thw); 
				return Math.min (thw, 4.0);
			}
			else if (cmd.equals ("get-velocity")) return speed;
			else if (cmd.equals ("get-chunk-id"))
			{
				//every chunk needs a unique id. Time is unique so I just used that
				double cid = (int)simulation.env.time;
				return cid;
			}
			else if (cmd.equals ("get-num-sign"))
			{
				//number of signs passed
				sign_count += 1;
				return sign_count;
			}
			else if (cmd.equals ("reset-num-sign"))
			{
				sign_count = 0;
				return sign_count;
			}
			else if (cmd.equals ("get-num-rehearsal"))
			{
				//number of rehearsals per iteration
				rehearsal_count += 1;
				return rehearsal_count;
			}
			else if (cmd.equals ("reset-rehearsal"))
			{
				//number of rehearsals in a loop
				rehearsal_count = 0;
				return rehearsal_count;
			} 
			// ADDED for timing 
			else if (cmd.equals("eval-speed"))
			{
				// 0 = early, 1 = safe, 2 = late
				double speedometerSpeed = Double.valueOf (it.next());
				double imaginedSpeed = Double.valueOf (it.next());
				double diff = Math.abs(speedometerSpeed - imaginedSpeed); 
				System.out.println("\tspeedo: " + speedometerSpeed + "(" + String.format("%.2f", Utilities.mph2mps(Utilities.kph2mph(speedometerSpeed))) + ") "+ " imagined: " + imaginedSpeed + "(" + String.format("%.2f", Utilities.mph2mps(Utilities.kph2mph(imaginedSpeed))) + ")"); 
				
				if (diff <= acceptableSpeedDiff)	// 2 atm
					return 0;	// early
				else if (diff > acceptableSpeedDiff && diff <= 2*acceptableSpeedDiff) //4 kmh
					return 1; 	// safe
				else if (diff > 2*acceptableSpeedDiff)
					return 2;	// late 
				else 
					return 0; 
				
				// in paper stable 0.07 rad (1/4 lane) = 4.01 degrees
				// https://www.calculator.net/right-triangle-calculator.html?av=&alphav=0.07&alphaunit=r&bv=10&betav=&betaunit=d&cv=&hv=&areav=&perimeterv=&x=0&y=0 
	
			// ADDED for range of similar experiences
			} else if (cmd.equals("get-mintick"))
			{
				double minT = Double.valueOf (it.next());
				double deduction = Double.valueOf (it.next());
				minT -= deduction; 
				return (minT < 0) ? 0: minT; 
			} 
			else return 0;
			
		}
		catch (Exception e)
		{
			System.out.println("exception!\n"); 
			e.printStackTrace();
			System.exit(1);
			return 0;
		}
	}

	void incNum(Integer rehearsal_count, boolean startover)
	{
		if (startover)
			rehearsal_count = 0;
		else
		{
			rehearsal_count += 1;
		}
	}


	public int numberOfSimulations () { return 1; }

	public Result analyze (Task[] tasks, boolean output)
	{
		return null;
	}
	
	String[] removeElement(String[] arr, int removeAt)
	{
		String[] newArray = new String[arr.length-1]; 
		String condi; 
		for (int i = 0; i < removeAt; i++)
		{
			condi = arr[i]; 
			newArray[i]= condi; 
		}
		for (int i = removeAt+1; i < arr.length; i++)
		{
			condi = arr[i];
			newArray[i-1] = condi; 
		}
		return newArray; 
	}

}
