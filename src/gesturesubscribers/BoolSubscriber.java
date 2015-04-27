package gesturesubscribers;

import java.util.Map;

import ros.RosBridge;
import ros.RosListenDelegate;

public class BoolSubscriber implements RosListenDelegate{
	private boolean legit_pose = false;
	@Override
	public void receive(Map<String, Object> data, String stringRep) {
		@SuppressWarnings("unchecked")
		Map<String, Boolean> hack = (Map<String, Boolean>) data.get("msg");
		legit_pose = hack.get("data") || legit_pose; //once true, always true
	}
	public boolean retrieveMostRecentNewMessage(){
		return legit_pose;
	}

	public static void main(String[] args){
		RosBridge bridge = RosBridge.createConnection("ws://localhost:9090");
		bridge.waitForConnection();
		bridge.subscribe("updated_position", "std_msgs/Bool", new BoolSubscriber());
	}
}
