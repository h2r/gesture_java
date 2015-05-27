package gesturesubscribers;

import java.util.Map;

import ros.RosBridge;
import ros.RosListenDelegate;

public class StringSubscriber implements RosListenDelegate{
	private String message = ""; //most recent at the front
	@Override
	synchronized public void receive(Map<String, Object> data, String stringRep) {
		@SuppressWarnings("unchecked")
		Map<String, String> map  = ((Map<String, String>)data.get("msg"));
		message += " " + map.get("data");
		System.out.println(message);
	}
	synchronized public void reset(){
		message = "";
	}
	/**
	 * gets the most recent message and empties the message queue
	 * @return the most recent string ("" if list is empty)
	 */
	synchronized public String retrieveMostRecentNewMessage(){
		String toret =  message;
		message = "";
		return toret;
	}

	public static void main(String[] args){
		RosBridge bridge = RosBridge.createConnection("ws://localhost:9090");
		bridge.waitForConnection();
		bridge.subscribe("speech_recognition", "std_msgs/String", new StringSubscriber());
	}
}
