package gesturesubscribers;

import java.util.List;
import java.util.Map;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import ros.RosBridge;
import ros.RosListenDelegate;

public class PointArraySubscriber implements RosListenDelegate{
	private double[] left_arm_origin = new double[1];
	private double[] right_arm_origin = new double[1];
	private double[] right_arm_point = new double[1];
	private double[] left_arm_point = new double[1];
	@Override
	public synchronized void receive(Map<String, Object> data, String stringRep) {
		@SuppressWarnings("unchecked")
		Map<String, Object> hack1 = (Map<String, Object>) data.get("msg");
		@SuppressWarnings("unchecked")
		List<Map<String, Double>> hack2 = (List<Map<String, Double>>) hack1.get("points");
		if(hack2.size() != 4) return;
		left_arm_origin = new double[3];
		right_arm_origin = new double[3];
		left_arm_point = new double[3];
		right_arm_point = new double[3];
		Map<String, Double> cur;
		String x = "x";
		String y = "y";
		String z = "z";
		cur = hack2.get(0);
		left_arm_origin[0] = cur.get(x);
		left_arm_origin[1] = cur.get(y);
		left_arm_origin[2] = cur.get(z);
		cur = hack2.get(1);
		right_arm_origin[0] = cur.get(x);
		right_arm_origin[1] = cur.get(y);
		right_arm_origin[2] = cur.get(z);
		cur = hack2.get(2);
		left_arm_point[0] = cur.get(x);
		left_arm_point[1] = cur.get(y);
		left_arm_point[2] = cur.get(z);
		cur = hack2.get(3);
		right_arm_point[0] = cur.get(x);
		right_arm_point[1] = cur.get(y);
		right_arm_point[2] = cur.get(z);

	}
	//goes lao, rao, lap, rap
	public synchronized double[][] retrieveMostRecentNewMessage(){
		double[][] arms = {left_arm_origin.clone(), right_arm_origin.clone(), left_arm_point.clone(), right_arm_point.clone()};
		return arms;
	}

	public static void main(String[] args){
		RosBridge bridge = RosBridge.createConnection("ws://localhost:9090");
		bridge.waitForConnection();
		bridge.subscribe("all_positions", "coordinate_publisher/PointArray", new PointArraySubscriber());
	}
}
