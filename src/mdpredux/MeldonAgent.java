package mdpredux;

import gesturesubscribers.ObjectArraySubscriber;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import org.apache.commons.math3.linear.RealVector;

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
	private boolean write_output = false;
	int face_val=0;
	double decay = 0.85;
	long ping_delay = 5000;//10000;
	long last_ping = 0;
	
	private ObjectArraySubscriber object_sub_left;
	private ObjectArraySubscriber object_sub_right;
	
	//bridge should be connected before calling this
	public MeldonAgent(PODomain domain, Policy policy, RosBridge bridge, ObjectArraySubscriber osl, ObjectArraySubscriber osr){
		this.domain = domain;
		this.policy = policy;
		String[] derp = {"metal_bowl", "silver_spoon", "plastic_spoon", "color_bowl"};
		this.graphy = new Graphy(derp);
		BeliefMDPGenerator bdgen = new BeliefMDPGenerator(this.domain);
		this.beliefDomain = (SADomain)bdgen.generateDomain();
		this.bridge = bridge;
		object_sub_left= osl;
		object_sub_right = osr;
	}
	public void resetBelief(){
		double[] reset_belief = new double[this.curBelief.numStates()];
		Arrays.fill(reset_belief, 1.0/this.curBelief.numStates());
		this.curBelief.setBeliefCollection(reset_belief);
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
		publishDistribution(objects, beliefs);
		for(i =0; i<objects.length; i++){
			max_likelihood = Math.max(max_likelihood, beliefs[i]);
			if(beliefs[i] >= threshold && System.currentTimeMillis() - ping_delay >= last_ping && objects.length > 1){
				last_ping = System.currentTimeMillis();
				HashMap<String, String> hack = new HashMap<String, String>();
				System.out.println("PICK " + objects[i]);
				hack.put("data", objects[i]);
				//this.bridge.publish("/fetch_commands", "std_msgs/String", hack);
				max_likelihood = 1.0; //set happiness to full
				break;
			}
		}
		HashMap<String, Integer> hack2 = new HashMap<String, Integer>();
		int real_val = 100 - (int) (Math.pow(Math.min(0.8, max_likelihood), 2) * 120.0);
		face_val = (int) ((1-decay) * real_val + (decay) * face_val);
		hack2.put("data", face_val);
		bridge.publish("/confusion/value/command", "std_msgs/Int32", hack2);
		
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
	public void publish_look(String o){
		HashMap<String, Object> pose= new HashMap<String, Object>();
		HashMap<String, Object> position = new HashMap<String, Object>();
		HashMap<String, Object> quaternion = new HashMap<String, Object>();
		HashMap<String, RealVector> derp = new HashMap<String, RealVector>();
		object_sub_left.retrieveMostRecentNewMessage(derp);
		object_sub_right.retrieveMostRecentNewMessage(derp);
		pose.put("position", position);
		pose.put("orientation", quaternion);
		quaternion.put("x", 0);quaternion.put("y", 0);quaternion.put("z", 0);quaternion.put("w", 0);
		double[] xyz = derp.get(o).toArray();
		position.put("x", xyz[0]);
		position.put("y", xyz[1]);
		position.put("z", xyz[2]);
		System.out.println("whyyyyyyyy");
		this.bridge.publish("/eyeGaze/Point/command2", "geometry_msgs/Pose", pose);
	}
	
	

}
