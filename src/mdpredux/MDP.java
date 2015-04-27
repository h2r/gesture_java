package mdpredux;


import ros.RosBridge;
import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.pomdp.wrappedmdpalgs.BeliefSarsa;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.pomdp.BeliefState;
import burlap.oomdp.singleagent.pomdp.PODomain;
import burlap.oomdp.singleagent.pomdp.POEnvironment;


public class MDP{

	public static void main(String [] args){
		//String[] objects =  {"redBowl", "greenBowl", "woodenBowl"};
		//String[] objects =  {"whiteSpoon", "metalCup", "woodenBowl", "blueBowl", "greenSpoon", "greenBowl", "metalSpoon", "brush", "redBowl", "brownCup"};
		String[] objects =  {"whiteSpoon", "woodenBowl", "blueBowl", "greenSpoon","brush", "brownCup"};
		//[.33, .75 ,0]
		//[.33, -0.75, 0]
		RosBridge bridge = RosBridge.createConnection("ws://localhost:9090");
		bridge.waitForConnection();
		GestureDomain derp = new GestureDomain(objects, "/home/meldon/catkin_ws/src/coordinate_publisher/src/object_lists/list5.txt", bridge, 0.99);
		PODomain d = (PODomain) derp.generateDomain();
		RewardFunction rf = GestureDomain.getRF();
		TerminalFunction tf = new NullTermination();
		BeliefState bs = GestureDomain.getInitialBeliefState(d);
		BeliefSarsa sarsa = new BeliefSarsa(d, rf, tf, 0.99, 20, 1, true, 10., 0.1, 0.5, 10000);
		sarsa.planFromBeliefStatistic(bs);
		Policy p = new GreedyQPolicy(sarsa);
		POEnvironment env = new POEnvironment(d, rf, tf);
		env.setCurMPDStateTo(bs.sampleStateFromBelief());
		MeldonAgent agent = new MeldonAgent(d, p, bridge);
		agent.setEnvironment(env);
		agent.setBeliefState(bs);
		agent.actUntilTerminal();
	}

}