package mdpredux;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import ros.RosBridge;
import burlap.behavior.singleagent.Policy;
import burlap.oomdp.core.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.pomdp.BeliefAgent;
import burlap.oomdp.singleagent.pomdp.BeliefMDPGenerator;
import burlap.oomdp.singleagent.pomdp.BeliefState;
import burlap.oomdp.singleagent.pomdp.BeliefState.StateBelief;
import burlap.oomdp.singleagent.pomdp.PODomain;

public class MeldonAgent extends BeliefAgent {

	protected PODomain	 	domain;
	protected Policy		policy;
	protected SADomain		beliefDomain;
	protected Graphy graphy;
	private double threshold = 0.7;
	private RosBridge bridge;
	private boolean write_output = true;
	int face_val=0;
	double decay = 0.85;
	long ping_delay = 10000;
	long last_ping = 0;
	//bridge should be connected before calling this
	public MeldonAgent(PODomain domain, Policy policy, RosBridge bridge){
		this.domain = domain;
		this.policy = policy;
		String[] derp = {"metal_bowl", "silver_spoon", "plastic_spoon", "color_bowl"};
		this.graphy = new Graphy(derp);
		BeliefMDPGenerator bdgen = new BeliefMDPGenerator(this.domain);
		this.beliefDomain = (SADomain)bdgen.generateDomain();
		this.bridge = bridge;
		//this.bridge.advertise("/fetch_commands", "std_msgs/String");
	}
	
	int step = 0;
	@Override
	public GroundedAction getAction(BeliefState curBelief) {
		
		//convert to belief state
		double[] beliefs = new double[curBelief.getStatesAndBeliefsWithNonZeroProbability().size()];
		String[] objects = new String[beliefs.length];
		int i=0;
		for(StateBelief o : curBelief.getStatesAndBeliefsWithNonZeroProbability()){
			beliefs[i] = o.belief;
			objects[i] = o.s.getFirstObjectOfClass("desired_object").getStringValForAttribute("object");
			i++;
		}
		this.graphy.paint(objects, beliefs);
		
		State s = BeliefMDPGenerator.getBeliefMDPState(this.beliefDomain, curBelief);
		GroundedAction ga = (GroundedAction)this.policy.getAction(s);
		//should we pick?
		double max_likelihood = 0.0;
		for(i =0; i<objects.length; i++){
			max_likelihood = Math.max(max_likelihood, beliefs[i]);
			if(beliefs[i] >= threshold && System.currentTimeMillis() - ping_delay >= last_ping && objects.length > 1){
				last_ping = System.currentTimeMillis();
				HashMap<String, String> hack = new HashMap<String, String>();
				System.out.println("PICK " + objects[i]);
				hack.put("data", objects[i]);
				this.bridge.publish("/fetch_commands", "std_msgs/String", hack);
				break;
			}
		}
		HashMap<String, Integer> hack2 = new HashMap<String, Integer>();
		int real_val = 100 - (int) (Math.pow(Math.min(0.8, max_likelihood), 2) * 120.0);
		face_val = (int) ((1-decay) * real_val + (decay) * face_val);
		hack2.put("data", face_val);
		/*if(max_likelihood >= 0.7) hack2.put("data", 0 + (10 - (33*(int)(max_likelihood - 0.7))));
		else if(max_likelihood >= 0.6) hack2.put("data", 10 - (100*(int)(max_likelihood - 0.6)));
		else if(max_likelihood >= 0.5) hack2.put("data", 20 - (100*(int)(max_likelihood - 0.5)));
		else if(max_likelihood >= 0.4) hack2.put("data", 30 - (100*(int)(max_likelihood - 0.4)));
		else if(max_likelihood >= 0.3) hack2.put("data", 50 - (100*(int)(max_likelihood - 0.4)));
		else hack2.put("data", 70 - (33*(int)(max_likelihood)));*/
		bridge.publish("/confusion/value/command", "std_msgs/Int32", hack2);
		publishDistribution(objects, beliefs);
		if(write_output){
				try(BufferedWriter output = new BufferedWriter(new FileWriter("/home/meldon/derp.txt", true))){
					output.write(Long.toString(System.currentTimeMillis()));
					for(i=0; i<objects.length;i++){
						output.write(",");
						output.write(objects[i] + ":" + Double.toString(beliefs[i]));
					}
					output.write("\n");
				} catch (IOException e) {
					e.printStackTrace();
				}
		}
		//
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		return ga;
	}
	
	public void publishDistribution(String[] objects, double[] beliefs) { 
		HashMap<String, Object> msg = new HashMap<>();
		msg.put("names", objects);
		msg.put("probs", beliefs);
		bridge.publish("mm_bf_state", "gesture_rec/BayesFilterStateDist", msg);
		
	}
	
	

}
