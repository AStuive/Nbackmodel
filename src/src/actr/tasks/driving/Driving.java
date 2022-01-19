package actr.tasks.driving;

import java.awt.BorderLayout;
import java.io.*;
import java.nio.file.Path;
import java.nio.file.Paths;
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

import actr.env.Core;
import actr.env.Main;
import actr.model.Chunk;
import actr.model.Declarative;
import actr.model.Imaginal;
import actr.model.Model;
import actr.model.Symbol;
import actr.task.Result;
import actr.task.Task;

/**
 * The main Driving task class that sets up the simulation and starts periodic updates.
 *  
 * @author Dario Salvucci
 */

/* TODO

* Can fire when isa drive	
attend near
eval safety safe
eval safety unsafe*
eval safety unsafe

Low control, bare minimum	BUT ALSO SOME (INACCURATE) SPEEDOMTER KNOWING

Can fire when isa control
Near attend far
Process far
Process without old
Loop stable
Loop unstable
High control	speedometer

							
Productions that don’t specify an isa in RHS 
Process far

change lines/centre position
driving stuff in the remaining 2 mdoels + different rehearsal numbers
	
	accuracy in scheunemann: 
			0		1		2		3		4
	norm	0.99	0.98	0.87	0.95	0.92
	constr	0.99	0.95	0.93	0.79	0.83
	
	remove the pre-nback speed blocks from fequency and SRR analysis
	last week just stop fitting, no matter what it looks like
	
	total differences: 
	rt			acc diff mp 11.5	correlation scheunemann
	0.075		0.559259259			0.433028953
	0.08		0.435 				0.582920244
	0.0825		0.493333333			0.394785963
	0.085		0.409876543			0.879471985		+
	0.0875		0.476666667			0.479955095
	0.09		0.443333333			0.218094163
	0.1			0.773535353			0.517722819

	mp  rt 0.85	acc diff			correlation
	1.5			3.300185186			0.379163117
	3			0.557777778			0.503243973
	8			0.576666666			0.439360898
	10.5		0.448194444			0.74344017	 	-
	11			0.579444444			0.384241394
	11.5		0.409876543			0.879471985		+
	12			0.443597884			0.453360541
	12.5		0.514545454			0.394233445
	15			0.508484848			0.534283038


add driving stuff to other models
100x per model draaien



it now goes back to the next step in driving loop right after looking at instructions/signs/speedo
same when it decided it's too unsafe to look at speedo
added the safe production but stays in control, or long pauses would happen
can't (re)start visual speedo production after instructions/sign until at least 1 driving loop production was fired
	nor start a new check
can't start rehearsal if it's not safe	
added percentage of time control/nback in goal

	
	run op hele lage threshold om te kijken of het construction verschil er nog is
	save hoe vaak verkeerd retrieved
	save afstand tot middelpunt weg
	latency factor 1.2!
	
	
	
check the naming of saw-speedsign/ didnt-see-speedsign!
there are 3 locations where number of rehearsals needs to be changed!
retrieval failed can't fire immediately after seeing something non driving
copy nback too! (nback*... instead of nnback-)

the warnings come from env.draw world2image (road.location (simcar.fracIndex + 1000, 2.5)); +1000 is too big 

why is the very first 2back always wrong? 
why is construction better? 


0.86 + 1 = 0.93 avg

20	40	60	80	100	120	140	160	180
[70 	60 	70 	60 	80 	60 	80 	60 	80]

at 179... 
nbackstate~4733 	remember 1 sign back go further		(nback-state~4733 isa nback-state id 160 slimit 60 1back 140 nbackspeed 80)
nbackstate~3385		remember 2 sign back stop		(nback-state~3385 isa nback-state id 120 slimit 60 1back 100 nbackspeed 70)


should've found nbackstate~.... id 140 1back 120 slimit 80
retrieval 3 rt -100 etc 

*/ 


// kortere blokken, check trace
// retrieved verkeerde?
// stuur naar moritz
// safe toevoeging?
// geen meeting voglende week 

// volgende week veel runs 100, planning schrijven voor na vakantie. 




// add more similatiies
// for some reason it is more often safe in construction. WHY. check it. 



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

	int blockDuration = 180; 					//			!!!!!!!!!!!!!!!!!!!!!!
	
	double startTime=0, endTime=20*blockDuration+20; 	// STOPS EVERYTHING
	double accelBrake=0, speed=0;

	static int minX=174, maxX=(238+24), minY=94, maxY=(262+32);
	static int centerX=(minX+maxX)/2, centerY=(minY+maxY)/2; 

	//make it bigger if the experiment is longer than 60s or signs appear more often
	static String currentLimit = "60"; //starting speed
	String previousLimit = "60";
	int nback_count_norm = 0;
	int nback_count_constr = 0; 
	int signCount = 0; 
	double signOnset = 0; 
	double instructionsOnset = -5;		
	double warningOnset = 0;
	boolean signShowing = false; 
	boolean signSeen = false; 
	int signReadded = 0; 
	static boolean instructionsShowing = false;
	static boolean warningSeen = false;
	boolean instructionsSeen = false; 
	int instrReadded = 0; 
	static int speedI = 0;
	static Coordinate signPos;
	static String currentNBack = "";	//2back, constr 3back		
	String[] nBack_list = 			{"2back", "0back", "3back", "1back", "3back", "1back", "4back", "4back", "0back", "2back", "9back"}; // 9 added for saving data at the end
	String[] nBack_list_constr = 	{"3back", "1back", "0back", "4back", "2back", "2back", "0back", "1back", "4back", "3back"};
	double sign_count = 0;								
	int rehearsal_count = 0;
	static String imaginedSpeedlimit = "60";
	List<String> output = new ArrayList<String>();	
	double mentalSpeed = 0; 
	double roughSpeed; 
	double prevTime = 0; 
	double curTime; 
	double prevDist = 0; 
	double curDist; 	
	String[] blocks = {"normal", "construction", "normal", "construction", "normal", "construction", "normal", "construction", "normal", "construction", "normal", "construction", "normal", "construction", "normal", "construction", "normal", "construction", "normal", "construction"}; 
	String curBlock = "normal"; 
	int[] speedSigns = {0, 0, 0, 0, 0}; 	// adds most recent speed at [4]
	int speedSignsAdded = 0; 
	int correctNbackSpeed = 0;
	int numSpeedoChecks = 0; 
	double freqSpeedoChecks = 0; 
	String startTimeBlock = "0"; 
	String startTimeNbackTask = "0"; 		// when enough signs have passed to do the nback
	double envTime[] = {0};
	double blockTime[] = new double[envTime.length]; 
	String prevCorSpeed = "0"; 
	double lastSafe = 0.0;
	boolean newBlock = false; 
	boolean newText = true; 
	boolean newCSV = true; 
	boolean newCSV1 = true; 
	double totSegments = 0;
	double errSegments = 0;
	double steerReversals = 0;
	double lastSteerAngle = -5; ; 
	String lastSave = "0"; 
	double[] accAllNbacksNorm = new double[]{9, 9, 9, 9, 9};  
	double[] accAllNbacksConstr = new double[]{9, 9, 9, 9, 9};   
	double[] corNbackBlockNorm = new double[]{0, 0, 0, 0, 0};  
	double[] corNbackBlockConstr = new double[]{0, 0, 0, 0, 0};  
	double[] SRRAllNbacksNorm = new double[]{9, 9, 9, 9, 9}; 
	double[] SRRAllNbacksConstr = new double[]{9, 9, 9, 9, 9}; 
	double[] freqAllNbacksNorm = new double[]{9, 9, 9, 9, 9}; 
	double[] freqAllNbacksConstr = new double[]{9, 9, 9, 9, 9}; 
	double modelNbackSpeed;  
	boolean removedSpeedSign = false;
	boolean removedInstr = false; 
	double[] deciAccAllNbacksNorm = new double[]{9, 9, 9, 9, 9};  
	double[] deciAccAllNbacksConstr = new double[]{9, 9, 9, 9, 9}; 
	double[] deciCorNbackBlockNorm = new double[]{0, 0, 0, 0, 0};
	double[] deciCorNbackBlockConstr = new double[]{0, 0, 0, 0, 0};
	String currentGoal = ""; 
	double numDrive = 0; 
	double numControl = 0; 
	double numNback = 0; 
	double percDrive = 0; 
	double percControl = 0; 
	double percNback = 0; 
	double[] percentsCDrive = new double[]{9, 9, 9, 9, 9}; 
	double[] percentsCControl = new double[]{9, 9, 9, 9, 9}; 
	double[] percentsNDrive = new double[]{9, 9, 9, 9, 9}; 
	double[] percentsNControl = new double[]{9, 9, 9, 9, 9}; 
	double[] percentsNNback= new double[]{9, 9, 9, 9, 9}; 
	double[] percentsCNback= new double[]{9, 9, 9, 9, 9}; 
	double allPercentsDrive= 0; 
	double allPercentsControl = 0; 
	double allPercentsNback = 0; 
	double allFreqs = 0; 
	double safeish = 0; 
	double unsafeish = 0; 
	double percentSafeish = 0; 
	double percentUnsafeish = 0; 
	double[] allSafeishN = new double[] {9, 9, 9, 9, 9}; 
	double[] allSafeishC = new double[] {9, 9, 9, 9, 9}; 
	int numRehearsal = 0; 
	
	double othw = 0; 
	double st = 0;
	int modelThinksCorrectSpeed = 0; 

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
			setLayout (new BorderLayout());
			if (simulator == null) simulator = new Simulator ();
			add (simulator, BorderLayout.CENTER);
			simulator.useSimulation (simulation);

		} else
		{		
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
		currentNBack = ""; 
		signPos = null; 
		signOnset = 0;
		signShowing = false; 
		instructionsOnset = -5;
		instructionsShowing = false; 
		warningSeen = false;
		mentalSpeed = 8; 		
		speedSignsAdded = 0; 
		correctNbackSpeed = 0; 
		Arrays.fill(speedSigns, 0);  	// make all 0 
		
		getModel().getVision().addVisual ("near", "near", "near", nearLabel.getX(), nearLabel.getY(), 1, 1, 10);
		getModel().getVision().addVisual ("far", "far", "far", farLabel.getX(), farLabel.getY(), 1, 1, 100);
		getModel().getVision().addVisual ("speedometer", "speedometer", "speedometer", 150, 315, 1, 1, 1);
		addPeriodicUpdate (Env.sampleTime);
		
	}

	public void update (double time)
	{
		if (time <= endTime) // 7200
		{				
			
			simulation.env.time = time - startTime; 
			updateVisuals();
			if(simulation.env.time>10) //only start after 10s
			{
				updateSign(simulation.env.time);
			}
			updateInstructions(simulation.env.time);			// $$$$$$$$$$$$$$$$$$$$$$
			
			Chunk goal = getModel().getBuffers().get(Symbol.goal); 
			currentGoal = goal.getISA(); 
//			System.out.println("goalx: " + goal.getISA()); 
			
				
			
			String curSave = Double.toString(simulation.env.time); 
			if (instructionsOnset >= 0 && !newBlock && !signShowing && Double.valueOf(curSave) - Double.valueOf(lastSave) >= 0.05) 
			{
				saveDVs(); 
				lastSave = Double.toString(simulation.env.time); 
			} 
			
			makeCSVwithData(simulation.env.simcar.speed, mentalSpeed);	
//			Env env = simulation.env; 
//			Simcar simcar = env.simcar;
//			double angle = simcar.steerAngle; 
//			double carPos = simcar.p.z;
//
//			System.out.println("carPosition: " + carPos + " steer angle: " + String.format("%.3f", env.simcar.steerAngle)); 
//			
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

	// Save all the data (20 Hz) during the experiment
	void saveDVs() 
	{
		// Currently only compares the imaginal speed to what it should be driving, not driving speed itself
//		Env env = simulation.env;
//		Simcar simcar = simulation.env.simcar;
		double deltaSpeed = 0; 
				
		try { 
			Chunk imag = getModel().getBuffers().get(Symbol.imaginal); 
			Iterator<Symbol> it = imag.getSlotNames(); 
			Iterator<Symbol> it2 = imag.getSlotValues();
			while (it.hasNext()) 
			{ 
				Symbol symbolname = it.next();
				Symbol symbolvalue = it2.next();
				String name = symbolname.toString(); 
				String value = symbolvalue.toString();
				if (name.equals("nbackspeed"))
				{
					int speed = Integer.valueOf(value); 
					if (modelThinksCorrectSpeed != speed)
						modelThinksCorrectSpeed = speed;
//					System.out.println("value2 " + modelThinksCorrectSpeed);	
				} 
			}
		} catch (Exception e) {}
				
		// Can do the nback already
		if (Double.parseDouble(startTimeNbackTask) != 0)
		{					
			if (modelThinksCorrectSpeed != 0 )
				deltaSpeed = Math.abs(modelThinksCorrectSpeed - correctNbackSpeed); 
			else
				deltaSpeed = Math.abs(modelNbackSpeed - correctNbackSpeed); 
//			deltaSpeed = Math.abs(3.6*simcar.speed - correctNbackSpeed);
			totSegments++;
			if (deltaSpeed > 5)
				errSegments++; 
			
			switch(currentGoal) 
			{
			case("drive"): numDrive++; 
			break;
			case("control"): numControl++; 
			break;
			case("nback"): numNback++; 
			default: 
			} 
		}
//		totSegments++;
//		System.out.println("ERRsegments: " + errSegments + " total: " + totSegments); 
	}
		
		// Get the accuracy for last block
		void checkCorrectLastBlock()
		{ 
			double acc = 1 - (errSegments / totSegments);
//			System.out.println("lastBlock errorseg: " + errSegments + " totseg: " + totSegments + " acc: " + acc);
			
			int curNback = Character.getNumericValue(currentNBack.charAt(0)); 
			// Only get correct when nback task has been started
		
			percDrive = numDrive / totSegments; 
			allPercentsDrive+= percDrive; 
			percControl = numControl / totSegments; 
			allPercentsControl+= percControl; 
			percNback = numNback / totSegments; 
			allPercentsNback += percNback; 
			allFreqs += freqSpeedoChecks; 
//			System.out.println("freq checks : " + freqSpeedoChecks + " all: " + allFreqs); 
						
			if (curBlock == "normal")
			{
				deciCorNbackBlockNorm[curNback] += acc; 				
				if (acc >= 0.9)
					corNbackBlockNorm[curNback]++; 
//				System.out.println("corNbackBLockNorm: " + corNbackBlockNorm[curNback]); 
			} else
			{
				deciCorNbackBlockConstr[curNback] += acc; 
				if (acc >= 0.9)
					corNbackBlockConstr[curNback]++; 
//				System.out.println("corNbackBLockConstr: " + corNbackBlockConstr[curNback]); 
			}
			
			totSegments = 0; 
			errSegments = 0;
			numDrive = 0;
			numControl = 0;		
			numNback = 0; 
			numSpeedoChecks = 0; 
			freqSpeedoChecks = 0;
		}
			
	// Save data at the end of every block and the blocks together
	void saveTotalOutput()
	{
		int blocksPassed = nback_count_norm + nback_count_constr;
		int curNback = Character.getNumericValue(currentNBack.charAt(0));
		System.out.println("==== " + String.format("%.2f", simulation.env.time) + "s Saving data block " + (blocksPassed) + ": " + curNback + "back " + curBlock + " ====");
		double acc = 0; 
		double SRS = 0; 
		double decimalAcc = 0;
		double percD = 0; 
		double percC = 0; 
		double percN = 0; 
		double freq = 0; 
		
		String filename = Main.core.getFilename();
		double time = simulation.env.time; 
		
		
		 	 
		// At the end of a (speed)block
		Declarative x = getModel().getDeclarative();
		double rt = x.getRetrievalThreshold();
		double mp = x.getMismatchPenalty(); 			
		double lf = x.getLatencyFactor(); 
		
		if (safeish != 0)
			percentSafeish = safeish / (safeish+unsafeish);
		else 
			percentSafeish = 0; 
		
		if (unsafeish != 0)
			percentUnsafeish = unsafeish / (safeish+unsafeish);
		else
			percentUnsafeish = 0; 
		
		try {
			
			File text = new File("Blocks-" + filename + "_rt" + rt + "_mp" + mp + "_lf" + lf + ".csv"); 
	        FileWriter fw = new FileWriter(text, true);
	        BufferedWriter bw = new BufferedWriter(fw);
	        PrintWriter pw = new PrintWriter(bw);	

	        String block = curBlock; 
	        if (block == "normal")
	        	block = block + "\t";

	        if (curBlock == "normal")  
	        {	
// the xx == 9 is the first time an accuracy is added, the else is to get the average of 2 blocks next time
// speedSignsAdded is the number of speed signs that were passed, ie blocks that could've been correct
	        	decimalAcc = deciCorNbackBlockNorm[curNback] / (speedSignsAdded-curNback);  
	        	if (deciAccAllNbacksNorm[curNback] == 9)
	        		deciAccAllNbacksNorm[curNback] = decimalAcc; 
	        	else 
	        		deciAccAllNbacksNorm[curNback] = (deciAccAllNbacksNorm[curNback] + decimalAcc) / 2;

	        	acc = corNbackBlockNorm[curNback] / (speedSignsAdded-curNback); //full 3 min block = 9 signs 	1 - (errSegments / totSegments);
	        	if (accAllNbacksNorm[curNback] == 9)
	        		accAllNbacksNorm[curNback] = acc;
	        	else 
	        		accAllNbacksNorm[curNback] = (accAllNbacksNorm[curNback] + acc) / 2; 

//	        	System.out.println("acc: " + acc + "  out of " + speedSignsAdded + " nback only " + (speedSignsAdded-curNback));
	        	
	        	SRS = steerReversals / (time - Double.valueOf(startTimeNbackTask));
	        	if (SRRAllNbacksNorm[curNback] == 9)
	        		SRRAllNbacksNorm[curNback] = SRS; 
	        	else
	        		SRRAllNbacksNorm[curNback] = (SRRAllNbacksNorm[curNback] + SRS) / 2; 
	        	
	        	freq = allFreqs / (speedSignsAdded-curNback);
	        	if (freqAllNbacksNorm[curNback] == 9) 
	        		freqAllNbacksNorm[curNback] = freq; 
	        	else 
	        		freqAllNbacksNorm[curNback] = (freqAllNbacksNorm[curNback] + freq) / 2; 
//	        	System.out.println("freq " + freq); 
	        	
	        	percD = allPercentsDrive / (speedSignsAdded-curNback); // percDrive; 
	        	if (percentsNDrive[curNback] == 9)
	        		percentsNDrive[curNback] = percD;
	        	else
	        		percentsNDrive[curNback] = (percentsNDrive[curNback] + percD) / 2; 
	        	
	        	percC = allPercentsControl / (speedSignsAdded-curNback); // percControl; 
	        	if (percentsNControl[curNback] == 9)
	        		percentsNControl[curNback] = percC;
	        	else
	        		percentsNControl[curNback] = (percentsNControl[curNback] + percC) / 2; 	
	        
	        	percN = allPercentsNback / (speedSignsAdded-curNback); // percNback; 
//	        	System.out.println("all%: " + allPercentsNback + " /blocks = " + percN); 
	        	if (percentsNNback[curNback] == 9)
	        		percentsNNback[curNback] = percN;
	        	else
	        		percentsNNback[curNback] = (percentsNNback[curNback] + percN) / 2;
	        	
	           	if (allSafeishN[curNback] == 9)
	        		allSafeishN[curNback] = percentSafeish; 
	        	else
	        		allSafeishN[curNback] = (allSafeishN[curNback] + percentSafeish) / 2; 
	        	
	        // Construction
	        } else 
	        {
	        	decimalAcc = deciCorNbackBlockConstr[curNback] / (speedSignsAdded-curNback);  
	        	if (deciAccAllNbacksConstr[curNback] == 9)
	        		deciAccAllNbacksConstr[curNback] = decimalAcc; 
	        	else 
	        		deciAccAllNbacksConstr[curNback] = (deciAccAllNbacksConstr[curNback] + decimalAcc) / 2;
	        	
	        	acc = corNbackBlockConstr[curNback] / (speedSignsAdded-curNback);
	        	if (accAllNbacksConstr[curNback] == 9)
	        		accAllNbacksConstr[curNback] = acc; 
	        	else
	        		accAllNbacksConstr[curNback] = (accAllNbacksConstr[curNback] + acc) / 2;
	        	
//	        	System.out.println("accAllNbacks " + accAllNbacksConstr[curNback]); 
//	        	System.out.println("acc: " + acc + "  out of " + speedSignsAdded + " nback only " + (speedSignsAdded-curNback));

	        	SRS = steerReversals / (time - Double.valueOf(startTimeNbackTask));
	        	if (SRRAllNbacksConstr[curNback] == 9)
	        		SRRAllNbacksConstr[curNback] = SRS;
	        	else 
	        		SRRAllNbacksConstr[curNback] = (SRRAllNbacksConstr[curNback] + SRS) / 2; 
	        	
	        	freq = allFreqs / (speedSignsAdded-curNback);
	        	if (freqAllNbacksConstr[curNback] == 9)
	        		freqAllNbacksConstr[curNback] = freq;
	        	else
	        		freqAllNbacksConstr[curNback] = (freqAllNbacksConstr[curNback] + freq) / 2; 
	        
	        	percD = allPercentsDrive / (speedSignsAdded-curNback); // percDrive; 
	        	if (percentsCDrive[curNback] == 9)
	        		percentsCDrive[curNback] = percD;
	        	else
	        		percentsCDrive[curNback] = (percentsCDrive[curNback] + percD) / 2; 
	        	
	        	percC = allPercentsControl / (speedSignsAdded-curNback); // percControl; 
	        	if (percentsCControl[curNback] == 9)
	        		percentsCControl[curNback] = percD;
	        	else
	        		percentsCControl[curNback] = (percentsCControl[curNback] + percD) / 2; 	
	        
	        	percN = allPercentsNback / (speedSignsAdded-curNback); // percNback; 
//	        	System.out.println("all%: " + allPercentsNback + " /blocks = " + percN); 
	        	if (percentsCNback[curNback] == 9)
	        		percentsCNback[curNback] = percN;
	        	else
	        		percentsCNback[curNback] = (percentsCNback[curNback] + percN) / 2;
	        	
	        	if (allSafeishC[curNback] == 9)
	        		allSafeishC[curNback] = percentSafeish; 
	        	else
	        		allSafeishC[curNback] = (allSafeishC[curNback] + percentSafeish) / 2; 
	        }
	        
	        if (newText) {
	        	pw.println("Time,Condition,Nback,FreqSpeedo,Accuracy,Steering Reversal Rates,PercDrive,PercControl,PercNback,PercSafe"); 
	        	newText = false; 
	        } 
	        
//	        System.out.println("err: " + errSegments + " tot " + totSegments + " acc " + acc); 
//	        System.out.println("Sr: " + steerReversals + " / " + time + " SRS = " + SRS); 
	        pw.println(String.format("%.2f", time-blockDuration) + "," + block + "," + currentNBack + "," + String.format("%.2f", freq) + "," + String.format("%.2f", acc) + "," + String.format("%.2f", SRS) + ", " + percD + "," + percC + "," + percN + "," + percentSafeish);
	        
	        corNbackBlockNorm = new double[5]; 
	        corNbackBlockConstr = new double[5]; 
	        deciCorNbackBlockNorm = new double[5];
	        deciCorNbackBlockConstr = new double[5]; 
			percentsCDrive[curNback] = percDrive; 
			percentsCControl[curNback] = percControl; 
			percentsCNback[curNback] = percNback; 
			allPercentsDrive = 0; 
			allPercentsControl = 0; 
			allPercentsNback = 0; 
			allFreqs = 0; 
	        percDrive = 0;
	        percControl = 0;
	        percNback = 0; 
	        pw.flush();
	        pw.close();
			
	    } catch (FileNotFoundException e) {
	        System.out.println(e.getMessage());
	    } catch (IOException e) {
	        e.printStackTrace();
	    }
		

//		System.out.println("safeish: " + safeish + " unsafeish " + unsafeish + "%safe: " + (safeish/(safeish+unsafeish))); 
//		System.out.println("out of all checks, %safe: " + String.format("%.2f", percentSafeish) + " %unsafe: " + String.format("%.2f", percentUnsafeish)); 
		percentSafeish = 0; 
		safeish = 0; 
		percentUnsafeish = 0;
		unsafeish = 0; 
		
		errSegments = 0;
		totSegments = 0; 
		steerReversals = 0; 

//		blocksPassed = nback_count_norm + nback_count_constr;
		// At the end of the whole experiment
		if (blocksPassed >= 20)
		{
			try {
				File data = new File("Output-" + filename + "_rt" + rt + "_mp" + mp + "_lf" + lf + ".csv");	
				FileWriter fw = new FileWriter(data, true);		
				BufferedWriter bw = new BufferedWriter(fw);
		        PrintWriter pw = new PrintWriter(bw);
		       if (newCSV) {
					pw.println("0backNAcc,0backCAcc,1backNAcc,1backCAcc,2backNAcc,2backCAcc,3backNAcc,3backCAcc,4backNAcc,4backCAcc,"
							+ "0backNSRR,0backCSRR,1backNSRR,1backCSRR,2backNSRR,2backCSRR,3backNSRR,3backCSRR,4backNSRR,4backCSRR,"
							+ "0backNFreq,0backCFreq,1backNFreq,1backCFreq,2backNFreq,2backCFreq,3backNFreq,3backCFreq,4backNFreq,4backCFreq,"
							+ "0percNControl,0percCControl,1percNControle,1percCControl,2percNControl,2percCControl,3percNControl,3percCControl,4percNControl,4percCControl,"
							+ "0percNNback,0percCNback,1percNNback,1percCNback,2percNNback,2percCNback,3percNNback,3percCNback,4percNNback,4percCNback,"
							+ "0SafeishN,0SafeishC,1SafeishN,1SafeishC,2SafeishN,2SafeishC,3SafeishN,3SafeishC,4SafeishN,4SafeishC");
					newCSV = false; 
				}
				pw.println(accAllNbacksNorm[0] + "," + accAllNbacksConstr[0] + "," + accAllNbacksNorm[1] + "," + accAllNbacksConstr[1] + "," + accAllNbacksNorm[2] + "," + accAllNbacksConstr[2] + "," + accAllNbacksNorm[3] + "," + accAllNbacksConstr[3] + "," + accAllNbacksNorm[4] + "," + accAllNbacksConstr[4] + "," + 
				SRRAllNbacksNorm[0] + "," + SRRAllNbacksConstr[1] + "," + SRRAllNbacksNorm[1] + "," + SRRAllNbacksConstr[1] + "," + SRRAllNbacksNorm[2] + "," + SRRAllNbacksConstr[2] + "," + SRRAllNbacksNorm[3] + "," + SRRAllNbacksConstr[3] + "," + SRRAllNbacksNorm[4] + "," + SRRAllNbacksConstr[4] + "," + 
				freqAllNbacksNorm[0] + "," + freqAllNbacksConstr[0] + "," + freqAllNbacksNorm[1] + "," + freqAllNbacksConstr[1] + "," + freqAllNbacksNorm[2] + "," + freqAllNbacksConstr[2] + "," + freqAllNbacksNorm[3] + "," + freqAllNbacksConstr[3] + "," + freqAllNbacksNorm[4] + "," + freqAllNbacksConstr[4] + "," +
				percentsNControl[0] + "," + percentsCControl[0] + "," + percentsNControl[1] + "," + percentsCControl[1] + "," + percentsNControl[2] + "," + percentsCControl[2] + "," + percentsNControl[3] + "," + percentsCControl[3] + "," + percentsNControl[4] + "," + percentsCControl[4] + "," +
				percentsNNback[0] + "," + percentsCNback[0] + "," + percentsNNback[1] + "," + percentsCNback[1] + "," + percentsNNback[2] + "," + percentsCNback[2] + "," + percentsNNback[3] + "," + percentsCNback[3] + "," + percentsNNback[4] + "," + percentsCNback[4] + "," +
				allSafeishN[0] + "," + allSafeishC[0] + "," + allSafeishN[1] + "," + allSafeishC[1] + "," + allSafeishN[2] + "," + allSafeishC[2] + "," + allSafeishN[3] + "," + allSafeishC[3] + "," + allSafeishN[4] + "," + allSafeishC[4]);

//		        for (int i = 0; i < 5; i++) {
//		        	pw.println("normal" + "\t\t" + i + "\t" + String.format("%.2f", freqAllNbacksNorm[i]) + "\t\t" + String.format("%.2f",accAllNbacksNorm[i]) + " (" + String.format("%.2f", deciAccAllNbacksNorm[i]) + ")\t\t" + String.format("%.2f",SRRAllNbacksNorm[i]));
//		        	pw.println("construction" + "\t" + i + "\t" + String.format("%.2f", freqAllNbacksConstr[i]) + "\t\t" + String.format("%.2f",accAllNbacksConstr[i]) + " (" + String.format("%.2f", deciAccAllNbacksConstr[i]) + ")\t\t" + String.format("%.2f",SRRAllNbacksConstr[i]));
//		        }
	        
		        pw.flush();
		        pw.close();
				
		    } catch (FileNotFoundException e) {
		        System.out.println(e.getMessage());
		    } catch (IOException e) {
		        e.printStackTrace();
		    }
		}

	}
	
	void updateInstructions(double time)
	{	
		// Display the instructions and switch blocks etc
		if(!instructionsShowing && ((time > 9 && time < 11) || (time > 11 && ((time-10)%blockDuration <= 1))))
		{
//			System.out.println("showing instructions"); 
			if (time >= 15) 
			{	
				checkCorrectLastBlock(); 
				saveTotalOutput(); 
			}
			startTimeBlock = String.valueOf(simulation.env.time); 
			curBlock = blocks[nback_count_norm + nback_count_constr];
			Env env = simulation.env;
			env.setCurBlockRoadEnv(curBlock);	
			Simcar simcar = simulation.env.simcar;
			simulation.env.road.startup(); 
			
			if (curBlock == "normal")
				currentNBack = nBack_list[nback_count_norm];	
			 else if(curBlock == "construction")
				currentNBack = nBack_list_constr[nback_count_constr];
						
			// reset some variables
			speedSignsAdded = 0; 
			// make a deep copy of env time
			
//			double envTime[] = {simulation.env.time};
//			double blockTime[] = new double[envTime.length]; 
//			blockTime[0] = envTime[0]; 
//			timeStartBlock = blockTime[0];
			newBlock = true; 
			prevCorSpeed = Double.toString(Utilities.mph2kph(Utilities.mps2mph(simcar.speed))); 
			
			instructionsShowing = true;
			// Clear speeds array and correct speed
			correctNbackSpeed = 0; 
			startTimeNbackTask = "0";
			startTimeBlock = "0"; 
			
			for (int i = 0; i < speedSigns.length; i++)
				speedSigns[i] = 0;
						
			getModel().getVision().addVisual("instructions", "instructions", currentNBack.substring(0,1), 320, 45, 50, 50, 0);
			instructionsOnset = time;
			
		} 
		else if (time - instructionsOnset < 3 && instructionsOnset >= 0 && instructionsShowing)
		{	
			// artificially giving the model more time to notice instructions by re-adding them
			if (!instructionsSeen && instrReadded < 10  && !removedInstr)
			{	//System.out.println("instructions: " + currentNBack + " now"); 
				//System.out.println(instrReadded + " Readded instructions at " + time); 
				getModel().getVision().removeVisual("instructions");
				getModel().getVision().addVisual("instructions", "instructions", currentNBack.substring(0,1), 320, 45, 50, 50, 0);
				//instructionsSeen = true; 
				instrReadded++;  
			}
		} else if((time - instructionsOnset >= 3) && instructionsOnset >= 0 && instructionsShowing)	// 2
		{	
			if (!removedInstr)
				getModel().getVision().removeVisual("instructions");
			else
				removedInstr = false; 
			instructionsShowing = false;
			instructionsSeen = false;
			instrReadded = 0; 	
			if (curBlock == "normal")
				nback_count_norm++;	
			 else if(curBlock == "construction")
				 nback_count_constr++;
			// addVisual(String id, String type, String value, int x, int y, int w, int h, double d)
		}			
	}
	
	void showRoadCondition(double time)
	{
		//addVisual(String id, String type, String value, int x, int y, int w, int h, double d)
		String start = String.valueOf(time); 
		getModel().getVision().addVisual("road-condition", "road-condition", curBlock, 150, 250, 1, 1, 0); 
		System.out.println("reach here?");
		if (time - Double.parseDouble(start) > 3)
		{
			System.out.println("if");
			getModel().getVision().removeVisual("road-condition");
		} else
		{
			System.out.println("else");
			getModel().getVision().removeVisual("road-condition");
			getModel().getVision().addVisual("road-condition", "road-condition", curBlock, 150, 250, 1, 1, 0); 
		}
	}
	
	void updateSign(double time)
	{
		String[] speedlimits = {"60", "70", "80", "90", "100", "110", "120", "130", "140"};		
		Env env = simulation.env;
		//Simcar simcar = simulation.env.simcar;
		
		Random r = new Random(); 
		double noise = r.nextGaussian() * 0.25;	
		
		// time%20 before!	added some temporal jitter			
		if((int)(time + noise)%20 == 0 && !signShowing && (speedI < speedlimits.length) )
		{	
			if (env.simcar.nearPoint != null)
			{	//System.out.println("jitter: " + noise); 
				//pick a random speed with delta<30
				previousLimit = currentLimit;
				while (Math.abs(Integer.parseInt(currentLimit) - Integer.parseInt(previousLimit)) > 30 || (currentLimit == previousLimit))
				{
					int rnd = new Random().nextInt(speedlimits.length);
					currentLimit = speedlimits[rnd];
				} 
				// keeps an array with the recent speeds to get the right one for the current nback
				int newSpeed = Integer.parseInt(currentLimit); 				
				if (speedSignsAdded > 0) 
					shiftLeft(); 
				speedSignsAdded++;
				speedSigns[4] = newSpeed;
				
				// determine what speed it should be driving at
				int nback = Character.getNumericValue(currentNBack.charAt(0)); 	
				if (speedSignsAdded > nback)
				{
					correctNbackSpeed = speedSigns[4 - nback];	
					if (startTimeNbackTask == "0") {			// only overwrite once
						startTimeNbackTask = Double.toString(simulation.env.time); 	
					}
				} else 
					correctNbackSpeed = Integer.parseInt(currentLimit); 
				
				// No checking immediately after nback starts as there's nothing to judge yet
				if (signCount > 0 && instructionsOnset > 0 && !newBlock && speedSignsAdded > nback+1)  
					checkCorrectLastBlock(); // Only after nback was started
				signCount++; 
				
				if (newBlock)
					newBlock = false;  
							
				// display speed sign
				Position newLoc = Road.location(env.simcar.fracIndex + 20 , 4);		// env.simcar.fracIndex + 20 , 4
				newLoc.y = 0.0;
				signPos = env.world2image(newLoc);
				signShowing = true;
				//getModel().getVision().addVisual ("speedsign", "speedsign", currentLimit, (int)signPos.x, (int)signPos.y, 1, 1, 100);
				getModel().getVision().addVisual ("speedsign", "speedsign", currentLimit, 605, 235, 1, 1, 100);
				//speedsign.setLocation(605, 235); //speedsign.setLocation((int)signPos.x, (int)signPos.y);
				signOnset = time;
				

			} 
				
		// move the speed sign
		} else if (signShowing && (time - signOnset < 3) && signOnset > 0) 
		{ 	//System.out.println("sign moving removedSpeedSign? " + removedSpeedSign); 
			// artificially giving the model more time to notice sign by re-adding it
			if (!signSeen && signReadded < 10 && !removedSpeedSign)
			{
				//System.out.println(signReadded + " Readded speed sign at " + time); 
				getModel().getVision().removeVisual("speedsign");
				getModel().getVision().addVisual ("speedsign", "speedsign", currentLimit, 605, 235, 1, 1, 100);
				signReadded++;
			}
//			signPos.x += 5;
//			signPos.y += 1;

		} else if(signShowing && (time - signOnset >= 3) && signOnset > 0)
		{
			if (!removedSpeedSign)
				getModel().getVision().removeVisual("speedsign");
			else 
				removedSpeedSign = false; 
			signPos = null; 
			signShowing = false;
			signSeen = false; 
			signReadded = 0; 
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
			Coordinate cc = env.world2image (env.simcar.farPoint);
			//Coordinate cc = env.world2image (env.simcar.carPoint);

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
		
		double curSteerAngle = simcar.steerAngle; 
		
		if (startTimeNbackTask != "0")	// to exclude the period before nback can be performed
		{
			if (lastSteerAngle == 0 && curSteerAngle > 0)
				steerReversals++; 	
			 else if (lastSteerAngle == 1 && curSteerAngle < 0)
				 steerReversals++;
			lastSteerAngle = (simcar.steerAngle > 0 ? 1 : 0); 	//  positive angles = 1, negative 0
			}
		}

	void doAccelerate (double fthw, double dthw, double dt, double tlimit)
	{
//		Simcar simcar = simulation.env.simcar;
//		if (simcar.speed >= 10.0)
//		{
//			double dacc = (dthw * accelFactor_dthw)
//					+ (dt * (fthw - thwFollow) * accelFactor_thw);
//			accelBrake += dacc;
//			accelBrake = minSigned (accelBrake, 1.0);
//		}
//		else
//		{
//			accelBrake = .65 * (simulation.env.time / 3.0);
//			accelBrake = minSigned (accelBrake, .65);
//		}
//		simcar.accelerator = (accelBrake >= 0) ? accelBrake : 0;
//		simcar.brake = (accelBrake < 0) ? -accelBrake : 0;
	}
	
	void keepLimit(double slimit)	// slimit is nbackspeed from imaginal in kph
	{		
//		Env env = simulation.env;
		modelNbackSpeed = slimit; 
		imaginedSpeedlimit = Integer.toString((int)slimit); 
		
		Simcar simcar = simulation.env.simcar;	
		double realSpeed = simcar.speed; 
		slimit = Utilities.mph2mps(Utilities.kph2mph(slimit)); // in mps

		// calculate the difference between what the model thinks it should be driving and what it should be driving
		double deltaSpeed; 
		if (correctNbackSpeed != 0)
			deltaSpeed = Utilities.mph2mps(Utilities.kph2mph(correctNbackSpeed)) - realSpeed; 	
		 else
		 {	// So after new instructions it keeps following the speed it was driving at
			 deltaSpeed = Utilities.mph2mps(Utilities.kph2mph(Double.parseDouble(currentLimit))) - realSpeed;
		 }
		double time = simulation.env.time - startTime;

		if (!newBlock) // newBlock is set to false at the first new speed sign
		{
			//warning if the speed limit is not kept (method says 6 seconds (3 before and after))
			if( Math.abs(deltaSpeed)>1.39 && !warningSeen && signOnset != 0 && (time - signOnset) > 3) //1.39 m/s is roughly 5 km/h
			{	
				warningOnset = time;
				//getModel().getVision().addVisual("warning", "warning", "warning", 200, 100, 50, 50);
				warning.setLocation(200, 50);
				warningSeen = true;
			} else if (warningSeen && Math.abs(deltaSpeed)<=1.39) // && (time - warningOnset) > 3) 
			{
				//getModel().getVision().removeVisual("warning");
				//warningOnset = time;
				warningSeen = false;
			}
		}
		
//		System.out.println("slimit: " + slimit + " mental: " + mentalSpeed); 
		
		
//	if (Math.abs(slimit - mentalSpeed) > 0) {
//		System.out.println("old accelbrake: " + accelBrake + " accelerator: " + simcar.accelerator + " brake: " + simcar.brake); 
//		
//		double fp = env.simcar.p.x; // far point
//		double dt = env.time - st; // delta time since last check?
//		double tr = slimit * dt; // speed you should be driving * delta time
//		double ftr = mentalSpeed * dt; // simcar.speed before
////		System.out.println("fp: " + fp +  " dt: " + dt + " tr: " + tr + " ftr: " + ftr);
//		
//		fp = fp - ftr + tr;
////		System.out.println("new fp: " + fp); 
//		double dist = fp - env.simcar.p.x;
//		double thw = Math.min(dist / mentalSpeed, 4.0);
//		double dhw = thw - othw;
////		System.out.println("dist: " + dist + " thw: " + thw + " dhw: " + dhw);
//		double dacc = (dhw * accelFactor_dthw * 5) + (dt * (thw) * accelFactor_thw * 5);
//		accelBrake += dacc;
//		accelBrake = minSigned(accelBrake, 1.0);
////		System.out.println("dacc: " + dacc + " accelBrake: " + accelBrake); 
//		othw = thw;
//		st = env.time; // time current check
//	}
	

//	System.out.println("old/new accelbrake: " + accelBrake + " accelerator: " + simcar.accelerator + " brake: " + simcar.brake); 
	
//	simcar.accelerator = (accelBrake >= 0) ? accelBrake : 0;
//	simcar.brake = (accelBrake < 0) ? -accelBrake : 0;
		
	// Original
	// acceleration is based on the MENTAL speed, not (necessarily) what the car is actually driving at
		
		double diff = slimit - mentalSpeed;
		if (Math.abs(diff) > 0)
		{				
			double dacc = (diff * 1.2) + (0.25 * diff * 0.4); 
			accelBrake += dacc;
			accelBrake = minSigned (accelBrake, 1.0); // 
		}
		
		if (mentalSpeed > 100/3.6 && Math.abs(diff) > 2)	{	//2 = 7.2 km/h
			accelBrake = accelBrake*1.5; 
//			System.out.println("multiplied accelbrake"); 
		}
		simcar.accelerator = (accelBrake >= 0) ? accelBrake : 0; 	
		simcar.brake = (accelBrake < 0) ? -(accelBrake/500) : 0;		// braking is way too strong
//		System.out.println("new accelbrake: " + accelBrake + " accelerator: " + simcar.accelerator + " brake: " + simcar.brake); 
		
}

	void makeCSVwithData(double real, double mental)
	{
		try {
			String filename = Main.core.getFilename();
			Declarative x = getModel().getDeclarative();
			double rt = x.getRetrievalThreshold();
			double mp = x.getMismatchPenalty(); 
			double lf = x.getLatencyFactor(); 
			File csv = new File("AllData-" + filename + "_rt" + rt + "_mp" + mp + "_lf" + lf + ".csv");  
	        FileWriter fw = new FileWriter(csv, true);
	        BufferedWriter bw = new BufferedWriter(fw);
	        PrintWriter pw = new PrintWriter(bw);	
	        if (newCSV1) {
	        	pw.println("\nTime,Real,Mental,ImaginalSpeed,CorrectSpeed,CurrentLimit,FreqSpeedo,Nback,Road,Goal,StartTimeNback");
	        	newCSV1 = false; 
	        }
	        String start; 
	        String corrSpeed = "0"; 
	        if (Double.parseDouble(startTimeNbackTask) != 0) {
	        	corrSpeed = Double.toString(correctNbackSpeed); 
	        	start = startTimeNbackTask; 
	        } else {
	        // After new instructions no line until the next sign
	        	corrSpeed = currentLimit; 
	        	start = ""; 
	        }
	        
	        String timing  = String.valueOf(simulation.env.time);
	        pw.println(timing + "," + 3.6*real + "," + 3.6*mental + "," + (modelThinksCorrectSpeed != 0? modelThinksCorrectSpeed : "") + "," + corrSpeed + "," + currentLimit + "," + freqSpeedoChecks + "," + currentNBack + "," + curBlock + "," + currentGoal + "," + start);
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
		double f = 2.5;		
		return (Math.abs(na) < .025*f) && (Math.abs(nva) < .0125*f) && (Math.abs(fva) < .0125*f);
	}
	
	boolean isCarSafe() {
		
		Env env = simulation.env;
		if (env.time > 2) {			

			double safeDistance = 0.65; 						
			double carWidth = 2.0;
			double carPosition = env.simcar.p.z;
			double rightLane = env.road.left(env.simcar.fracIndex, env.simcar.lane).z;
			double leftLane = env.road.right(env.simcar.fracIndex, env.simcar.lane).z;
//			System.out.println("carpos: " + carPosition + " left: " + String.format("%.3f", leftLane) + " right: " + String.format("%.3f", rightLane));
			//System.out.println("distance to safe: " + (safeDistance+carWidth/2));
			//System.out.println("Accepted safe: " + String.format("%.3f", (leftLane + (safeDistance + carWidth / 2))) + " to " + String.format("%.3f",(rightLane - (safeDistance + carWidth / 2))));
			
			if (curBlock == "construction")
				return false;  
			else 
			{
				boolean b = carPosition > leftLane + (safeDistance + carWidth / 2)
					&& carPosition < rightLane - (safeDistance + carWidth / 2);
				lastSafe = b ? env.time : lastSafe;
			//System.out.println("isCarSafe returned \t\t" + b); 
				return b;
			}
		} else 
			return false;
	}

	
	double image2angle (double x, double d)		// x = near/far angle, d = distance
	{
		Env env = simulation.env;
		double px = env.simcar.p.x + (env.simcar.h.x * d);
		double pz = env.simcar.p.z + (env.simcar.h.z * d);
		Coordinate im = env.world2image (new Position (px, pz)); 
		try { return Math.atan2 (.5*(x-im.x), 450); }		
		//  counterclockwise angle to the POSITIVE (right side) X axis. Y value first
		//  math.atan2(y,x) * 180 / Math.PI = normal degrees ie 2.2 = 45
//				|	/<_
//				|  /   \^
//				| /	    \  
//				|/	     |^
//		-------------------x	math.atan2(y, x)
//				|
//				|
//				|
//				y
		
//		right triangle
//		hoek beta
//		  |\\\\\
//		  |   	    \\ 	c
//		a |                 \\
//		  |______________________\	hoek alpha
//		hoek 90	      b
//
//		if b = 100, a = 
//
//		Ctrl+O - Select cos h in Scientific mode.
//		Ctrl+S - Select sin h in Scientific mode.
//		Ctrl+T - Select tan h in Scientific mode.
//		Shift+S - Select sin -1 in Scientific mode.
//		Shift+O - Select cos -1 in Scientific mode.
//		Shift+T - Select tan -1 in Scientific mode.
//		1 graden = 0.0175 radialen

		catch (Exception e) { return 0; }
	}

	public void eval (Iterator<String> it)
	{
		it.next();
		String cmd = it.next();
		if (cmd.equals ("do-steer"))
		{
			double na = Double.valueOf (it.next());
			double dna = Double.valueOf (it.next()); 
			double dfa = Double.valueOf (it.next());
			double dt = Double.valueOf (it.next());
			doSteer (na, dna, dfa, dt);
			Env env = simulation.env;					
			Simcar simcar = simulation.env.simcar;				
			//System.out.println("steer angle: " + String.format("%.3f", env.simcar.steerAngle)); 
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
		} 
		else if (cmd.equals("set-rough-speed"))
		{	
			// only check rough speed every once in a while but do keep adding noise every time
			Simcar simcar = simulation.env.simcar;
			
			Random r = new Random(); 
			double noise = r.nextGaussian() * 0.1;		// 1 mps is 3.6 km/h		0.3 = 1.2 km/h
			curTime = simulation.env.time; 						
			curDist = simcar.getDistance();
			
			// difference mental - (rough + noise)			
			double diffDist = curDist - prevDist; 
			double dTime = curTime - prevTime;
			if (diffDist != 0 && dTime != 0)
				roughSpeed = diffDist / dTime + noise; 
			else 
				roughSpeed = noise; 
			
			//System.out.println("roughspeed: " + roughSpeed*3.6); 
			
			mentalSpeed = (mentalSpeed + roughSpeed)/2; 
			
			prevTime = curTime; 
			prevDist = curDist; 
						
		} 
		else if (cmd.equals("set-speedo-speed"))
		{					
			// calculate frequency of speedo checks, only after nbacks started
			if (currentNBack != "" && startTimeNbackTask != "0") { 
				numSpeedoChecks++; 
				double deltaTime = simulation.env.time - Double.valueOf(startTimeNbackTask);
				freqSpeedoChecks = numSpeedoChecks / deltaTime; 
//				System.out.println("num: " + numSpeedoChecks + " delta " + deltaTime + " freq " + freqSpeedoChecks); 
			}
			
			// updates mentalspeed to speedo value
			double speedoSpeed = Double.valueOf (it.next()); // in kmh
			double imagined = Double.valueOf(it.next());  // in kmh
			mentalSpeed = Utilities.mph2mps(Utilities.kph2mph(speedoSpeed)); // in mps			
		} 
		else if (cmd.equals("saw-speedsign"))
		{	
			signSeen = true;  
			 //System.out.println("saw speed sign");
		} 
		else if (cmd.equals("attended-speedsign"))
		{
			getModel().getVision().removeVisual("speedsign");
			removedSpeedSign = true;
//			System.out.println("attended-speed-sign removedSpeedSign? " + removedSpeedSign );
		} 
		else if (cmd.equals("saw-instructions")) 
			instructionsSeen = true; 
		else if (cmd.equals("attended-instructions"))
		{
			getModel().getVision().removeVisual("instructions");
			removedInstr = true;
//			System.out.println("attended-instructions removedinstr? " + removedInstr);
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
		} else if (cmd.equals("didnt-see-speedsign"))
			return !signSeen;
		
		else if (cmd.equals("didnt-see-instructions"))
			return !instructionsSeen; 
		else if (cmd.equals("safe-zone") || cmd.equals("not-safe-zone")) 
		{	
			boolean b = isCarSafe();
			return cmd.equals("safe-zone") ? b : !b;
		} else if (cmd.equals("do-reset") || cmd.equals("do-not-reset")) 
		{
			
			boolean safe = isCarSafe();
			boolean b = !safe  && (simulation.env.time - lastSafe - Env.sampleTime*2 < 0.05) ? true : false;
			// sample time = 0.050
			// if car is not safe, and time since last time safe is less than 0.05 seconds -> true
			// otherwise false
			
			// 1) car is not safe but was safe very recently > reset
			// 2) car is not safe but wasn't safe recently -> do not reset
			// 3) car is safe -> do not reset
			
			return (cmd.equals("do-reset") ? b : !b);
			
		// so it doesn't check speedometer when far away from the centre
		} else if (cmd.equals("is-safeish-and-stable") || cmd.equals("is-unsafeish-and-unstable"))
		{	
			Env env = simulation.env; 
			double carPosition = env.simcar.p.z;
			double ll = env.road.right(env.simcar.fracIndex, env.simcar.lane).z;
			double rl = env.road.left(env.simcar.fracIndex, env.simcar.lane).z;	
			// dead center is 2.5. 
//			System.out.println("ll: " + ll +" rl " + rl + " carPosition " + carPosition); 
//			System.out.println("angle: " + env.simcar.steerAngle); 
			Simcar simcar = env.simcar;
			double angle = simcar.steerAngle; 
			double carPos = simcar.p.z;
			boolean stable; 
			boolean safe; 
//			System.out.println("carPosition: " + carPos + " steer angle: " + String.format("%.3f", env.simcar.steerAngle)); 
			
			// Safe as long as the steering wheel isn't too far from 0
			if (curBlock == "normal") 
				stable = (angle > -0.08 && angle < 0.08 ? true : false); 	// 0.1 before
			else 
				stable = (angle > -0.06 && angle < 0.06 ? true : false); 	// 0.075 before
			
			// Safe as long as the edge of the car is within the lane
			if (carPosition > (ll+1) && carPosition < (rl-1))  // (-)0.75 N / (-)0.625 C
				safe =  true; 
			else 
				safe = false; 

			// Another option is to implement an 'unsafe but steering toward safety' method
			
//			System.out.println("steering stable? : " + stable + "\t pos safe: " + safe + " " + "\t " + (cmd.equals("is-safeish-and-stable") ? " stable&safe: " + (safe && stable) : "")); 
//			System.out.println("stable&safe? " + (safe && stable));
			if (safe && stable)
				safeish++; 
			else 
				unsafeish++; 
//			System.out.println("safeish: " + safeish + " unsafeish " + unsafeish); 
			
//			safe = true; 
//			stable = true; 
			
			return (cmd.equals("is-safeish-and-stable") ? (safe && stable) : !(safe && stable)); 
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
				double asd = image2angle (x, d);
				//System.out.println("image to angle : " + asd); 
				return asd;
			}
			else if (cmd.equals ("mp-time")) return simulation.env.time;
			else if (cmd.equals ("get-thw"))
			{
				double fd = Double.valueOf (it.next());
				double v = Double.valueOf (it.next());
				double thw = (v==0) ? 4.0 : fd/v;
				return Math.min (thw, 4.0);
			}
			else if (cmd.equals ("get-velocity")) return speed;
			else if (cmd.equals ("get-chunk-id"))
			{
				//every chunk needs a unique id. Time is unique so I just used that
				double cid = (int)simulation.env.time;
				int modlo = (int)cid%20; 
			    if (modlo < 10)  // it was above 20 
			        cid -= modlo;  
			    else      	// under 20
			    	cid += (20-modlo);				
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
				//System.out.println("\tspeedo: " + speedometerSpeed + "(" + String.format("%.2f", Utilities.mph2mps(Utilities.kph2mph(speedometerSpeed))) + ") "+ " imagined: " + imaginedSpeed + "(" + String.format("%.2f", Utilities.mph2mps(Utilities.kph2mph(imaginedSpeed))) + ")"); 
				
				if (diff <= 2.5)	
					return 0;	// early
				else if (diff > 2.5 && diff <= 5) 
					return 1; 	// safe
				else if (diff > 5)
					return 2;	// late 
				else 
					return 0; 
				
			// ADDED for range of similar experiences
			} else if (cmd.equals("get-mintick"))
			{
				double minT = Double.valueOf (it.next());
				double deduction = Double.valueOf (it.next());
				minT -= deduction; 
				return (minT < 0) ? 0: minT; 	
			} 	// Computes the average time in between signs
			else if (cmd.equals("get-average-time"))
			{
				double totTime = Double.valueOf (it.next());
				int numSigns = Integer.valueOf (it.next());
				double avg = totTime/numSigns;
				return avg; 
			// Computes at which time the correct nback sign was displayed	
			} else if (cmd.equals("get-correct-time"))
			{
				double totTime = Double.valueOf (it.next());
				double avgTime = Double.valueOf (it.next());	
				double signs = Double.valueOf (it.next());	
//				System.out.println("cormom: " + (totTime - (signs * avgTime)));
				return totTime - (signs * avgTime); 
			} else if (cmd.equals("get-rehearse-number"))
			{
				String numReh = it.next();
				int number = Character.getNumericValue(numReh.charAt(numReh.length()-1)); 
				return number; 
			} else if (cmd.equals("get-maxtick"))
			{
				double ticks = Double.valueOf (it.next());
				//System.out.println("maxtick: " + ticks); 
				return (ticks > 65) ? 65 : ticks; 	
			}
				
				/* (p nback*briefly-stop-later-rehearsal
					    =goal>
		        isa nback
		        - status notice
		        - status read-nback
		        - status remember0
		        - status remember1
		        - status remember2
		        - status remember3
		        - status remember4
		        < nr 3	;; number of rehearsals
		        > num 0
		        rehearsal t
		    =retrieval>
		        isa nback-state
		==>
		    =goal>
		        status notice
		    -retrieval>
		) */
				
			else return 0;
			
		}
		catch (Exception e)
		{
			System.out.println("exception!"); 
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
	
	void shiftLeft()
	{
		for (int i = 0; i<4; i++)
		{
			speedSigns[i] = speedSigns[i+1]; 
		}
		speedSigns[4] = 0; 	
		
	}
}
