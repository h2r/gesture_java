package mdpredux;

import gesturesubscribers.ObjectArraySubscriber;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

import org.apache.commons.lang3.StringUtils;
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

import com.sun.speech.freetts.*;



public class MeldonAgent extends BeliefAgent {

	protected PODomain	 	domain;
	protected Policy		policy;
	protected SADomain		beliefDomain;
	//protected Graphy graphy;
    protected BarChart barry;
	//private double threshold = 0.7;
	private double threshold = 1.0;
	private RosBridge bridge;
	private boolean write_output = false;
	int face_val=0;
	double decay = 0.85;
	long ping_delay = 5000;//10000;
	long last_ping = 0;
	
	private ObjectArraySubscriber object_sub_left;
	private ObjectArraySubscriber object_sub_right;

	public String lastSaid;// = "";
    public enum Response {
        INITIAL, CLOSE, MAYBE, PROBABLY
    }
    public Response oldResp = Response.INITIAL;
	
	//bridge should be connected before calling this
	public MeldonAgent(PODomain domain, Policy policy, RosBridge bridge, ObjectArraySubscriber osl, ObjectArraySubscriber osr){
		this.domain = domain;
		this.policy = policy;
		//String[] derp = {"metal_bowl", "silver_spoon", "plastic_spoon", "color_bowl"};
		//this.graphy = new Graphy(derp);
        this.barry = new BarChart("Robot Internal Estimate", "Object Probability Distribution");
		BeliefMDPGenerator bdgen = new BeliefMDPGenerator(this.domain);
		this.beliefDomain = (SADomain)bdgen.generateDomain();
		this.bridge = bridge;
		object_sub_left= osl;
		object_sub_right = osr;
        lastSaid = "";
	}
	public void resetBelief(){
		double[] reset_belief = new double[this.curBelief.numStates()];
		Arrays.fill(reset_belief, 1.0/this.curBelief.numStates());
		this.curBelief.setBeliefCollection(reset_belief);
	}

	public void speak(String text) {
		  Voice voice;
		  VoiceManager voiceManager = VoiceManager.getInstance();
		  voice = voiceManager.getVoice("kevin16");
		  voice.setStyle("casual");
		  voice.allocate();
		  voice.setVolume(100);
		  voice.speak(text);
		  voice.deallocate();
		 }
	public String whatsOnTable(String[] objects) {
		String iSee = " I see the ";
		if(objects.length == 1){
			iSee += addSpace(objects[0]);
		}
		else {
			for(int i = 0; i < objects.length - 1; i++){

				iSee += addSpace(objects[i]) + " and the ";
			}
			iSee += addSpace(objects[objects.length-1]);
		}
		return iSee;
	}
	public String addSpace(String object) {
		String[] obj = object.split("(?=\\p{Upper})");
		String withSpace = StringUtils.join(obj," ");
		return withSpace;
	}

    public double entropy(double[] beliefs) {
        double ent = 0;
        for(int i = 0; i < beliefs.length; i++) {
            ent -= beliefs[i]*Math.log(beliefs[i]);
        }
        return ent;
    }

	public void topObjects(TreeMap<String,Double> objs) {
        if(objs.entrySet().size() == 10) {
            return;
        }
        String[] orderedObjects = new String[objs.size()];
        double[] orderedBeliefs = new double[objs.size()];
        Iterator<Map.Entry<String,Double>> it = objs.entrySet().iterator();
        for (int i = objs.size() - 1; i >= 0; i--) {
            Map.Entry<String, Double> cur = it.next();
            orderedObjects[i] = cur.getKey();
            orderedBeliefs[i] = cur.getValue();
        }
        //System.out.println("OrderedObjects" + Arrays.toString(orderedObjects));
        //System.out.println("OrderedBeliefs" + Arrays.toString(orderedBeliefs));
        String topObject = orderedObjects[0];
        String secondObject = orderedObjects[1];
        double topBelief = orderedBeliefs[0];
        double secondBelief = orderedBeliefs[1];
		String topTwo;
        Response resp;
		if(topBelief - secondBelief < 0.05 && !secondObject.equals("")) {
            if(topObject.compareTo(secondObject) < 0)
                topTwo = "Is it the " + addSpace(topObject) + " or the " + addSpace(secondObject) + "?";
            else
                topTwo = "Is it the " + addSpace(secondObject) + " or the " + addSpace(topObject) + "?";
            resp = Response.CLOSE;
        }
		else if(topBelief - secondBelief < 0.2 && !secondObject.equals("")) {
            topTwo = "I think it's the " + addSpace(topObject) + " but it might be the " + addSpace(secondObject) + ". Which one is it?";
            resp = Response.MAYBE;
            //publish_look(topObject);
        }
		else if (topBelief < threshold && !secondObject.equals("")){
            topTwo = "It's probably the " + addSpace(topObject);
            resp = Response.PROBABLY;
            //publish_look(topObject);
        }
        else {
            topTwo = "";
            resp = Response.INITIAL;
        }
        //double ent = entropy(beliefs);
		//System.out.println("entropy = " + ent);
		if(resp != oldResp || !lastSaid.equals(topTwo)) {
			oldResp = resp;
            lastSaid = topTwo;
			System.out.println(topTwo);
            //System.out.println(topObject + ": " + topBelief + "; " + secondObject + ": " + secondBelief);
			this.speak(topTwo);
            publish_look(topObject);
		}
	}
	public void publishFace(double max_likelihood) {
        HashMap<String, Integer> hack2 = new HashMap<String, Integer>();
        int real_val = 100 - (int) (Math.pow(Math.min(0.8, max_likelihood), 2) * 120.0);
        face_val = (int) ((1-decay) * real_val + (decay) * face_val);
        //System.out.println("face val: " + face_val);
        hack2.put("data", face_val);
        bridge.publish("/confusion/value/command", "std_msgs/Int32", hack2);
    }
	int step = 0;
	@Override
	public GroundedAction getAction(BeliefState curBelief) {
		
		//convert to belief state
		double[] beliefs = new double[curBelief.getStatesAndBeliefsWithNonZeroProbability().size()];
		String[] objects = new String[beliefs.length];
		int i=0;
        final HashMap<String,Double> map = new HashMap<String,Double>();
        TreeMap<String,Double> objs = new TreeMap<String,Double>(new Comparator<String>() {
			@Override
			public int compare(String a, String b) {
				if (map.get(a) < map.get(b)) {
					return -1;
				} else if(map.get(a) > map.get(b)) {
					return 1; // returning 0 would merge keys
				} else if(a.compareTo(b) == 0) {
                    return -1;
                    //throw new RuntimeException("Tried to compare two identical items" + a + " " + b);
                } else return a.compareTo(b);
			}
		});
		for(StateBelief o : curBelief.getStatesAndBeliefsWithNonZeroProbability()){
			beliefs[i] = o.belief;
			objects[i] = o.s.getFirstObjectOfClass("desired_object").getStringValForAttribute("object");
			i++;
            map.put(o.s.getFirstObjectOfClass("desired_object").getStringValForAttribute("object"), o.belief);
		}
        objs.putAll(map);
//		String iSee = whatsOnTable(objects);
//		if(!iSee.equals(this.lastSeen)) {
//			this.lastSeen = iSee;
//			System.out.println(iSee);
//			this.speak(iSee);
//		}

        //this.graphy.paint(objects, beliefs);
        this.barry.createDataset(objects,beliefs);
        this.barry.draw();
		if(objects.length > 1) topObjects(objs);

		
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
				//System.out.println("PICK " + objects[i]);
				hack.put("data", objects[i]);
                publish_look(objects[i]);
				this.bridge.publish("/fetch_commands", "std_msgs/String", hack);
				max_likelihood = 1.0; //set happiness to full
				String handing = "Handing over the " + addSpace(objects[i]);
                if(!lastSaid.equals(handing)) {
                    System.out.println("lastSaid = " + lastSaid);
                    lastSaid = handing;
                    System.out.println(handing);
                    this.speak(handing);
                }
				break;
			}
		}
        //publishFace(max_likelihood); //replaced with head movement animation
		
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
		HashMap<String, RealVector> msg = new HashMap<String, RealVector>();
		object_sub_left.retrieveMostRecentNewMessage(msg);
		object_sub_right.retrieveMostRecentNewMessage(msg);
		pose.put("position", position);
		pose.put("orientation", quaternion);
		quaternion.put("x", 0);quaternion.put("y", 0);quaternion.put("z", 0);quaternion.put("w", 0);
		try {
            double[] xyz = msg.get(o).toArray();

            position.put("x", xyz[0]);
            position.put("y", xyz[1]);
            position.put("z", xyz[2]);

            this.bridge.publish("/eyeGaze/Point/command2", "geometry_msgs/Pose", pose);
        } catch (NullPointerException e) {
            System.err.println("Tried to look at non-existent object: " + addSpace(o));
        }
    }
}
