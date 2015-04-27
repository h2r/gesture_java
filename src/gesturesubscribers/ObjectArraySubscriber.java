package gesturesubscribers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import ros.RosBridge;
import ros.RosListenDelegate;

public class ObjectArraySubscriber implements RosListenDelegate{
	private Map<String, RealVector> object_pos = new HashMap<String, RealVector>();
	@SuppressWarnings("unchecked")
	@Override
	synchronized public void receive(Map<String, Object> data, String stringRep) {
		ArrayList<Map<String, Object>> objects = (ArrayList<Map<String, Object>>)((Map<Object, Object>) data.get("msg")).get("objects");
		String object_name;
		Map<String, Double> super_hack;
		object_pos.clear();
		for(Map<String, Object> o : objects){
			object_name = ((Map<String, String>)o.get("type")).get("key");
			super_hack = (Map<String, Double>)((Map<String, Object>)((Map<String, Object>)((Map<String, Object>)o.get("pose")).get("pose")).get("pose")).get("position");
			RealVector object_coord = new ArrayRealVector(3);
			object_coord.addToEntry(0, super_hack.get("x"));
			object_coord.addToEntry(1, super_hack.get("y"));
			object_coord.addToEntry(2, super_hack.get("z"));
			object_pos.put(object_name, object_coord);
		}
		System.out.println(object_pos.toString());
	}
	synchronized public Map<String, RealVector> retrieveMostRecentNewMessage(){
		return object_pos;
	}
	synchronized public void retrieveMostRecentNewMessage(Map<String, RealVector> derp){
		derp.putAll(object_pos);
	}

	public static void main(String[] args){
		//RosBridge bridge = RosBridge.createConnection("ws://localhost:9090");
		RosBridge bridge = RosBridge.createConnection("ws://apollo:9090");
		bridge.waitForConnection();
		ObjectArraySubscriber sub = new ObjectArraySubscriber();
		bridge.subscribe("/ein_left/blue_memory_objects", "object_recognition_msgs/RecognizedObjectArray", sub);
		bridge.subscribe("/ein_right/blue_memory_objects", "object_recognition_msgs/RecognizedObjectArray", sub);
	}
}
