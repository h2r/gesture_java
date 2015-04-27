package mdpredux;

import gesturesubscribers.BoolSubscriber;
import gesturesubscribers.ObjectArraySubscriber;
import gesturesubscribers.PointArraySubscriber;
import gesturesubscribers.PointSubscriber;
import gesturesubscribers.StringSubscriber;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.apache.commons.math3.distribution.MultivariateNormalDistribution;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import ros.RosBridge;

import burlap.behavior.singleagent.auxiliary.StateEnumerator;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.debugtools.RandomFactory;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.Attribute;
import burlap.oomdp.core.Attribute.AttributeType;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.ObjectClass;
import burlap.oomdp.core.ObjectInstance;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TransitionProbability;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.pomdp.BeliefState;
import burlap.oomdp.singleagent.pomdp.ObservationFunction;
import burlap.oomdp.singleagent.pomdp.PODomain;

public class GestureDomain implements DomainGenerator {
	private static String OBSERVATIONCLASS = "observation_class";
	private static String DESIREDOBJECTCLASS = "desired_object";
	private static String OBJECTATTRIBUTE = "object";
	private static String LEFTARMORG = "left_arm_origin";
	private static String RIGHTARMORG = "right_arm_origin";
	private static String RIGHTARMPNT = "right_arm_point";
	private static String LEFTARMPNT = "left_arm_point";
	private static String POINTARRAYTOPIC = "all_positions";
	private static String POINTARRAYMSGTYPE = "coordinate_publisher/PointArray";
	private static String SPEECHOBS = "speech_observation";
	private static String SPEECHTOPIC = "speech_recognition";
	private static String SPEECHMSGTYPE = "std_msgs/String";
	private static String OBJECTARRAYTYPE = "object_recognition_msgs/RecognizedObjectArray";
	private static String OBJECTPOSTOPICLEFT = "/ein_left/blue_memory_objects";
	private static String OBJECTPOSTOPICRIGHT = "/ein_right/blue_memory_objects";
	//private static String OBJECTPOSTOPICRIGHT = "/test_objects";//, "object_recognition_msgs/RecognizedObjectArray", sub);

	private double transition_to_same = 0.9;
	private String[] objects;
	private StringSubscriber string_sub = new StringSubscriber();
	PointArraySubscriber body_pose_sub = new PointArraySubscriber();
	private RosBridge bridge;
	private Map<String, Map<String,Double>> unigram_probs;
	private double epsilon; //for unigram...
	private MultivariateNormalDistribution gesture_dist;
	private ObjectArraySubscriber object_sub_left = new ObjectArraySubscriber();
	private ObjectArraySubscriber object_sub_right = new ObjectArraySubscriber();
	/**
	 * 
	 * @param objects - a list of object names
	 * @param object_pos - a list of realvectors representing the 3d positions of objects (in same order)
	 * @param object_filename - a file of object word relations (file format is line="<object_name> <word1> <word2> ... <wordn>")
	 */
	public GestureDomain(String[] objects, String object_filename, RosBridge bridge){
		this.objects = objects;
		this.unigram_probs = object_unigram_model(object_filename);
		this.bridge = bridge;
		this.bridge.subscribe(SPEECHTOPIC, SPEECHMSGTYPE, string_sub);
		this.bridge.subscribe(OBJECTPOSTOPICLEFT, OBJECTARRAYTYPE, object_sub_left);
		this.bridge.subscribe(OBJECTPOSTOPICRIGHT, OBJECTARRAYTYPE, object_sub_right);
		this.bridge.subscribe(POINTARRAYTOPIC, POINTARRAYMSGTYPE, body_pose_sub, 1, 1);
		double[] mean = {0.0};
		double[][] variance = {{0.35}};
		gesture_dist = new MultivariateNormalDistribution(mean, variance);
	}
	public GestureDomain(String[] objects, String object_filename, RosBridge bridge, double transition_same){
		assert(transition_same >=0.0 && transition_same <= 1.0);
		this.bridge = bridge;
		this.transition_to_same = transition_same;
		this.objects = objects;
		this.unigram_probs = object_unigram_model(object_filename);
		this.bridge.subscribe(SPEECHTOPIC, SPEECHMSGTYPE, string_sub);
		this.bridge.subscribe(OBJECTPOSTOPICLEFT, OBJECTARRAYTYPE, object_sub_left);
		this.bridge.subscribe(OBJECTPOSTOPICRIGHT, OBJECTARRAYTYPE, object_sub_right);
		this.bridge.subscribe(POINTARRAYTOPIC, POINTARRAYMSGTYPE, body_pose_sub, 1, 1);
		double[] mean = {0.0};
		double[][] variance = {{0.35}};
		gesture_dist = new MultivariateNormalDistribution(mean, variance);
	}
	public static String getObjectAttributeFromName(String name){
		return name + "_location";
	}
	public static String getObjectValidAttributeFromName(String name){
		return name + "_valid";
	}
	private Map<String, Map<String,Double>> object_unigram_model(String filename){
		Map<String, Map<String,Double>> word_probs = new HashMap<String, Map<String,Double>>();
		try {
			BufferedReader br = new BufferedReader(new FileReader(filename));
			String line;
			String[] words;
			int total_words=0;
			this.epsilon = 0.1;
			boolean first = true;
			Map<String,Double> curr = new HashMap<String,Double>(); //this is only to supress warnings
			//read and count
			while((line = br.readLine())!= null){
				words = line.split("\\s+");
				first = true;
				for(String w : words){
					if(first){
						first = false;
						if(!word_probs.containsKey(w)){
							word_probs.put(w, new HashMap<String, Double>()); //create a new hashmap if necessary
						}
						curr = word_probs.get(w);
					}else{
						total_words +=1;
						if(curr.containsKey(w)) curr.put(w, curr.get(w)+1.0);
						else curr.put(w, 1.0);
					}
				}
			}
			//normalize
			double total;
			for(String k : word_probs.keySet()){ // for each object
				curr = word_probs.get(k);
				total = 0.0;
				for(String v : curr.keySet()){
					total += curr.get(v);
				}
				for(String v : curr.keySet()){
					curr.put(v, curr.get(v)/total);
				}
			}
			this.epsilon = 1/(double)total_words;
			br.close();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return word_probs; //empty
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return word_probs;
	}
	@Override
	public Domain generateDomain() {
		
		PODomain domain = new PODomain();
		
		
		Attribute which_object = new Attribute(domain, OBJECTATTRIBUTE, AttributeType.DISC);
		which_object.setDiscValues(objects);
		
		Attribute left_arm_origin = new Attribute(domain, LEFTARMORG, AttributeType.DOUBLEARRAY);
		Attribute right_arm_origin = new Attribute(domain ,RIGHTARMORG, AttributeType.DOUBLEARRAY);
		Attribute left_arm_point = new Attribute(domain, LEFTARMPNT, AttributeType.DOUBLEARRAY);
		Attribute right_arm_point = new Attribute(domain ,RIGHTARMPNT, AttributeType.DOUBLEARRAY);
		Attribute speech = new Attribute(domain, SPEECHOBS, AttributeType.STRING);
		
		ObjectClass desired_object = new ObjectClass(domain, DESIREDOBJECTCLASS);
		desired_object.addAttribute(which_object);
		
		ObjectClass obs_class = new ObjectClass(domain, OBSERVATIONCLASS);
		for(String s : this.objects){ //attribute for all object locations
			Attribute object_att = new Attribute(domain, GestureDomain.getObjectAttributeFromName(s), AttributeType.DOUBLEARRAY);
			obs_class.addAttribute(object_att);
			Attribute object_valid_att = new Attribute(domain, GestureDomain.getObjectValidAttributeFromName(s), AttributeType.BOOLEAN);
			obs_class.addAttribute(object_valid_att);
		}
		obs_class.addAttribute(left_arm_origin);
		obs_class.addAttribute(right_arm_origin);
		obs_class.addAttribute(left_arm_point);
		obs_class.addAttribute(right_arm_point);
		obs_class.addAttribute(speech);
		
		new WaitingAction(domain, this.transition_to_same);
		new MultimodalObservations(domain, this.epsilon);
		StateEnumerator senum = new StateEnumerator(domain, new DiscreteStateHashFactory());
		for(String s : this.objects){
			senum.getEnumeratedID(generate_state(domain, s));
		}
		domain.setStateEnumerator(senum);
		return domain;
	}
	
	private static State generate_state(Domain d, String object_name){
		State s = new State();
		ObjectInstance o = new ObjectInstance(d.getObjectClass(DESIREDOBJECTCLASS), DESIREDOBJECTCLASS);
		o.setValue(OBJECTATTRIBUTE, object_name);
		s.addObject(o);
		return s;
	}
	public static BeliefState getInitialBeliefState(PODomain domain){
		BeliefState bs = new BeliefState(domain);
		bs.initializeBeliefsUniformly();
		return bs;
	}
	
	
	public class WaitingAction extends Action{
		double trans_prob = 0.9;
		public WaitingAction(Domain domain, double trans_prob){
			super("waiting", domain, "");
			this.trans_prob = trans_prob;
		}
		
		@Override
		protected State performActionHelper(State s, String[] params) {
			Random random = RandomFactory.getMapped(0);
			double r = random.nextDouble();
			
			if(r <= trans_prob || objects.length == 1){
				return s; //no change
			}
			else{
				double curr_value = trans_prob;
				String curr_obj = s.getFirstObjectOfClass(DESIREDOBJECTCLASS).getValueForAttribute(OBJECTATTRIBUTE).getStringVal();
				for(String item : objects){ //look at all other objects
					if(!curr_obj.equals(item)){ //if it is a different object than the current one
						curr_value += (1-trans_prob)/(objects.length-1); 
						if(curr_value >= r){
							s.getFirstObjectOfClass(DESIREDOBJECTCLASS).setValue(OBJECTATTRIBUTE, item);
							return s;
						}
					}
				}
			}
			
			return s;
		}
		
		@Override
		public List<TransitionProbability> getTransitions(State s, String [] params){
			List<TransitionProbability> tps = new ArrayList<TransitionProbability>(objects.length);
			State next_state = s.copy();
			tps.add(new TransitionProbability(next_state, trans_prob)); //transition to same
			for(String item : objects){
				next_state = s.copy();
				if(!item.equals(s.getFirstObjectOfClass(DESIREDOBJECTCLASS).getValueForAttribute(OBJECTATTRIBUTE).getStringVal())){
					//don't do the same object twice
					next_state.getFirstObjectOfClass(DESIREDOBJECTCLASS).setValue(OBJECTATTRIBUTE, item);
					tps.add(new TransitionProbability(next_state, (1-trans_prob)/(objects.length-1)));
				}
			}	
			return tps;
		}
		
		
	}
	
	
	
	public class MultimodalObservations extends ObservationFunction{
		private double epsilon;
		
		public MultimodalObservations(PODomain domain, double epsilon){
			super(domain);
			this.epsilon = epsilon;
		}
		
		@Override
		public List<State> getAllPossibleObservations() {
			throw new RuntimeException("Infinite number of possible observations");
		}
		
		@Override
		public State sampleObservation(State state, GroundedAction action){
			return sampleRealWorldObservation(state, action);
		}
		@Override
		public State sampleRealWorldObservation(State state, GroundedAction action){
			State obs = new State();
			ObjectInstance obR = new ObjectInstance(this.domain.getObjectClass(OBSERVATIONCLASS), OBSERVATIONCLASS);
			obR.setValue(SPEECHOBS, string_sub.retrieveMostRecentNewMessage());
			double[][] positions = body_pose_sub.retrieveMostRecentNewMessage();
			/*if(positions[0] == null  || positions[1] == null || positions[2] == null || positions[3] == null){
				obR.setValue(LEFTARMPNT, new double[1]); //this avoids BURLAP throwing errors if anything is null
				obR.setValue(RIGHTARMPNT, new double[1]);
				obR.setValue(LEFTARMORG, new double[1]);
				obR.setValue(RIGHTARMORG, new double[1]);
			}else{*/
				obR.setValue(LEFTARMORG, positions[0]);
				obR.setValue(RIGHTARMORG, positions[1]);
				obR.setValue(LEFTARMPNT, positions[2]); //this avoids BURLAP throwing errors if anything is null
				obR.setValue(RIGHTARMPNT,  positions[3]);
			//}
			Map<String, RealVector> object_coords = new HashMap<>();
			while(object_coords.isEmpty()){
				object_sub_left.retrieveMostRecentNewMessage(object_coords);
				object_sub_right.retrieveMostRecentNewMessage(object_coords);
				if(object_coords.isEmpty()) System.out.println("Waiting for object locations");
			}
			for(String s : objects){
				if(object_coords.containsKey(s)){
					obR.setValue(GestureDomain.getObjectAttributeFromName(s),object_coords.get(s).toArray());
					obR.setValue(GestureDomain.getObjectValidAttributeFromName(s), true);
				}else{
					obR.setValue(GestureDomain.getObjectAttributeFromName(s), new double[3]);
					obR.setValue(GestureDomain.getObjectValidAttributeFromName(s), false);
				}
			}
			obs.addObject(obR);
			return obs;
		}
		@Override
		public double getObservationProbability(State observation, State state,
				GroundedAction action) {
			String speech_obs = observation.getFirstObjectOfClass(OBSERVATIONCLASS).getStringValForAttribute(SPEECHOBS);
			String object_val = state.getFirstObjectOfClass(DESIREDOBJECTCLASS).getStringValForAttribute(OBJECTATTRIBUTE);
			if(!observation.getFirstObjectOfClass(OBSERVATIONCLASS).getBooleanValue(GestureDomain.getObjectValidAttributeFromName(object_val))){
				return 0.0;
			}
			double prob = 1.0;
			String[] words = speech_obs.split("\\s+");
			if(!unigram_probs.containsKey(object_val)) unigram_probs.put(object_val, new HashMap<String, Double>()); //new object that you can only point at 
			String temp;
			Map<String, Double> word_prob = unigram_probs.get(object_val);
			for(String w : words){
				if(!w.equals("")){
					temp = w.toLowerCase();
					if(word_prob.containsKey(temp)){
						prob *= word_prob.get(temp);
					}else{
						prob *= this.epsilon;
					}
				}
			}
			double[] lao = observation.getFirstObjectOfClass(OBSERVATIONCLASS).getDoubleArrayValue(LEFTARMORG);
			double[] rao = observation.getFirstObjectOfClass(OBSERVATIONCLASS).getDoubleArrayValue(RIGHTARMORG);
			double[] rap = observation.getFirstObjectOfClass(OBSERVATIONCLASS).getDoubleArrayValue(RIGHTARMPNT);
			double[] lap = observation.getFirstObjectOfClass(OBSERVATIONCLASS).getDoubleArrayValue(LEFTARMPNT);
			double[] objloc;
			double closest_angle_right = Double.MAX_VALUE;
			double closest_angle_left = Double.MAX_VALUE;
			RealVector right_point;
			RealVector left_point;
			RealVector object_point_right;
			RealVector object_point_left;
			double[] left_radian = new double[1];
			double[] right_radian = new double[1];
//			/= observation.getFirstObjectOfClass(OBSERVATIONCLASS).getDoubleArrayValue(GestureDomain.getObjectAttributeFromName(object_val));
			if(lao.length == 3 && lap.length == 3 && rap.length == 3 && rao.length == 3){ //if not all points are there, don't calculate with them
				for(String s : objects){ //make sure gesture is not null by seeing if any object is within a radius
					objloc = observation.getFirstObjectOfClass(OBSERVATIONCLASS).getDoubleArrayValue(GestureDomain.getObjectAttributeFromName(s));
					right_point = new ArrayRealVector(rap).subtract(new ArrayRealVector(rao));
					left_point = new ArrayRealVector(lap).subtract(new ArrayRealVector(lao));
					object_point_right = new ArrayRealVector(objloc).subtract(new ArrayRealVector(rao));
					object_point_left = new ArrayRealVector(objloc).subtract(new ArrayRealVector(lao));
					left_radian[0] = Math.acos(left_point.cosine(object_point_left));
					right_radian[0] = Math.acos(right_point.cosine(object_point_right));
					if(Math.abs(left_radian[0]) < closest_angle_left) closest_angle_left = Math.abs(left_radian[0]);
					if(Math.abs(right_radian[0]) < closest_angle_right) closest_angle_right = Math.abs(right_radian[0]);
				}
				objloc = observation.getFirstObjectOfClass(OBSERVATIONCLASS).getDoubleArrayValue(GestureDomain.getObjectAttributeFromName(object_val));
				//put all coordinates in zero origin frame
				right_point = new ArrayRealVector(rap).subtract(new ArrayRealVector(rao));
				left_point = new ArrayRealVector(lap).subtract(new ArrayRealVector(lao));
				object_point_right = new ArrayRealVector(objloc).subtract(new ArrayRealVector(rao));
				object_point_left = new ArrayRealVector(objloc).subtract(new ArrayRealVector(lao));
				left_radian[0] = Math.acos(left_point.cosine(object_point_left));
				right_radian[0] = Math.acos(right_point.cosine(object_point_right));
				if(closest_angle_left <= 0.7){
					//prob *= Math.max(gesture_dist.density(left_radian), 0.04);
					prob *= gesture_dist.density(left_radian);
				}
				if(closest_angle_right <= 0.7){
					//prob *= Math.max(gesture_dist.density(right_radian), 0.04);
					prob *= gesture_dist.density(right_radian);
				}
			}
			return prob;
		}
		
		@Override
		public boolean isTerminalObservation(State observation) {
			return false;
		}
		
	}
	public static RewardFunction getRF(){
		return new GestureRF();
	}
	
	public static class GestureRF implements RewardFunction{
		
		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			return 0.0;
		}
		
		
		
	}


}
