package gesturesubscribers;

import java.util.Map;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import ros.RosBridge;
import ros.RosListenDelegate;

public class PointSubscriber implements RosListenDelegate{
	private RealVector pos = new ArrayRealVector(3);
	@Override
	public void receive(Map<String, Object> data, String stringRep) {
		@SuppressWarnings("unchecked")
		Map<String, Double> hack = (Map<String, Double>) data.get("msg");
		double x = hack.get("x");
		double y = hack.get("y");
		double z = hack.get("z");
		pos = new ArrayRealVector(3);
		pos.setEntry(0,x); pos.setEntry(1,y); pos.setEntry(2,z);
	}
	public double[] retrieveMostRecentNewMessage(){
		return pos.toArray();
	}

	public static void main(String[] args){
		RosBridge bridge = RosBridge.createConnection("ws://localhost:9090");
		bridge.waitForConnection();
		bridge.subscribe("left_arm_origin", "geometry_msgs/Point", new PointSubscriber());
	}
}
