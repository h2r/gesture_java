package gesturesubscribers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import ros.RosBridge;
import ros.RosListenDelegate;

public class EinStateSubscriber implements RosListenDelegate{
	private Map<String, RealVector> object_pos = new HashMap<String, RealVector>();
	private boolean zero_g = false;
	private String movement_state = "ARMED";
	private String patrol_state = "PATROLLING";
	private String place_mode = "HAND";
	private String patrol_mode = "ONCE";
	private String idle_mode = "EMPTY";
	
	@SuppressWarnings("unchecked")
	@Override
	synchronized public void receive(Map<String, Object> data, String stringRep) {
		//System.out.println(stringRep);
		Map<Object, Object> msg = (Map<Object, Object>) data.get("msg");
		ArrayList<Map<String, Object>> objects = (ArrayList<Map<String, Object>>)(msg.get("objects"));
		String object_name;
		Map<String, Double> super_hack;
		object_pos.clear();
		if(objects != null){
			for(Map<String, Object> o : objects){
				object_name = ((Map<String, String>)o.get("type")).get("key");
				super_hack = (Map<String, Double>)((Map<String, Object>)((Map<String, Object>)((Map<String, Object>)o.get("pose")).get("pose")).get("pose")).get("position");
				RealVector object_coord = new ArrayRealVector(3);
				object_coord.addToEntry(0, super_hack.get("x"));
				object_coord.addToEntry(1, super_hack.get("y"));
				object_coord.addToEntry(2, super_hack.get("z"));
				object_pos.put(object_name, object_coord);
			}
		}
		//System.out.println(object_pos.toString())
		int zg = (int) msg.get("zero_g");
		int ms = (int) msg.get("movement_state");
		int ps = (int) msg.get("patrol_state");
		int plm = (int) msg.get("place_mode");
		int pam = (int) msg.get("patrol_mode");
		int im = (int) msg.get("idle_mode");
		
		this.zero_g = zg != 0;
		
		if(ms == 0) this.movement_state = "ARMED";
		if(ms == 1) this.movement_state = "BLOCKED";
		if(ms == 2) this.movement_state = "STOPPED";
		if(ms == 3) this.movement_state = "HOVERING";
		if(ms == 4) this.movement_state = "MOVING";
		
		if(ps == 0) this.patrol_state = "IDLING";
		if(ps == 1) this.patrol_state = "PATROLLING";
		if(ps == 2) this.patrol_state = "PICKING";
		if(ps == 3) this.patrol_state = "PLACING";
		if(ps == 4) this.patrol_state = "HANDING";
		
		if(plm == 0) this.place_mode =  "HAND";
		else this.place_mode = "WAREHOUSE";
		
		if(pam == 0) this.patrol_mode = "ONCE";
		else this.patrol_mode = "LOOP";
		
		if(im == 0) this.idle_mode = "EMPTY";
		if(im == 1) this.idle_mode = "STOPCLEAR";
		if(im == 2) this.idle_mode = "PATROl";
		if(im == 3) this.idle_mode = "CRANE";
		if(im == 4) this.idle_mode = "SHRUG";
		//System.out.println(patrol_state);
	}
	synchronized public String getMovementState(){
		return this.movement_state;
	}
	synchronized public String getPatrolState(){
		return this.patrol_state;
	}
	synchronized public String getPlaceMode(){
		return this.place_mode;
	}
	synchronized public String getPatrolMode(){
		return this.patrol_mode;
	}
	synchronized public String getIdleMode(){
		return this.idle_mode;
	}
	synchronized public boolean isZeroG(){
		return this.zero_g;
	}
	synchronized public void retrieveMostRecentNewMessage(Map<String, RealVector> derp){
		derp.putAll(object_pos);
	}
	

	public static void main(String[] args){
		//RosBridge bridge = RosBridge.createConnection("ws://localhost:9090");
		RosBridge bridge = RosBridge.createConnection("ws://apollo:9090");
		bridge.waitForConnection();
		EinStateSubscriber sub1 = new EinStateSubscriber();
		EinStateSubscriber sub2 = new EinStateSubscriber();
		bridge.subscribe("ein_right/state", "ein/EinState", sub1);
		bridge.subscribe("ein_left/state", "ein/EinState", sub2);
	}
}
