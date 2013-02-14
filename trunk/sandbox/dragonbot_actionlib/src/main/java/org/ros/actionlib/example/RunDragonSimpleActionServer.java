package org.ros.actionlib.example;

import org.ros.namespace.GraphName;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.Node;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import prg.content.dragonbot.teleopstandalone.DragonBotComm;

/**
 * A main for running the Dragon simple action server.
 */
public class RunDragonSimpleActionServer {

  public static void main(String[] args) {
    main();
  }

  public static void main() {
    DragonBotComm comm = new DragonBotComm();
    //DragonBotComm comm = null;
    try {
      // user code implementing the SimpleActionServerCallbacks interface
      ExpressionMotionSimpleActionServerCallbacks impl = new ExpressionMotionSimpleActionServerCallbacks(comm);
      VisemeSimpleActionServerCallbacks impl2 = new VisemeSimpleActionServerCallbacks(comm);
      //LookatSimpleActionServerCallbacks impl3 = new LookatSimpleActionServerCallbacks(comm);
      //IKSimpleActionServerCallbacks impl4 = new IKSimpleActionServerCallbacks(comm);

      ExpressionMotionActionSpec emSpec = new ExpressionMotionActionSpec();
      VisemeActionSpec vSpec = new VisemeActionSpec();
      //LookatActionSpec lSpec = new LookatActionSpec();
      //IKActionSpec ikSpec = new IKActionSpec();

      final String nodeName1 = "ExpressionMotion_Server";
      final String nodeName2 = "Viseme_Server";
      final String nodeName3 = "Lookat_Server";
      final String nodeName4 = "IK_Server";
      final ExpressionMotionSimpleActionServer emas =
          emSpec.buildSimpleActionServer(nodeName1, impl, true);

      final VisemeSimpleActionServer vas =
          vSpec.buildSimpleActionServer(nodeName2, impl2, true);
/*
      final LookatSimpleActionServer las =
          lSpec.buildSimpleActionServer(nodeName3, impl3, false);

      final IKSimpleActionServer ikas =
          ikSpec.buildSimpleActionServer(nodeName4, impl4, false);
*/
      NodeConfiguration configuration = NodeConfiguration.newPrivate();

      NodeMainExecutor runner = DefaultNodeMainExecutor.newDefault();

      runner.execute(new NodeMain() {

        @Override
        public void onStart(ConnectedNode node) {
          emas.addClientPubSub(node);
        }
        
        @Override
        public void onShutdown(Node node) {
        }

        @Override
        public void onShutdownComplete(Node node) {
        }

        @Override
        public void onError(Node node, Throwable throwable) {
        }

        @Override
        public GraphName getDefaultNodeName() {
          return GraphName.of(nodeName1);
        }
      }, configuration);
    
      runner.execute(new NodeMain() {

        @Override
        public void onStart(ConnectedNode node) {
	  vas.addClientPubSub(node);
        }
        
        @Override
        public void onShutdown(Node node) {
        }

        @Override
        public void onShutdownComplete(Node node) {
        }

         @Override
        public void onError(Node node, Throwable throwable) {
        }

       @Override
        public GraphName getDefaultNodeName() {
          return GraphName.of(nodeName2);
        }
      }, configuration);
    
      /*runner.execute(new NodeMain() {

        @Override
        public void onStart(ConnectedNode node) {
	  las.addClientPubSub(node);
        }
        
        @Override
        public void onShutdown(Node node) {
        }

        @Override
        public void onShutdownComplete(Node node) {
        }
        @Override
        public void onError(Node node, Throwable throwable) {
        }


        @Override
        public GraphName getDefaultNodeName() {
          return GraphName.of(nodeName3);
        }
      }, configuration);*/

    /*runner.execute(new NodeMain() {

        @Override
        public void onStart(ConnectedNode node) {
	  ikas.addClientPubSub(node);
        }
        
        @Override
        public void onShutdown(Node node) {
        }

        @Override
        public void onShutdownComplete(Node node) {
        }
        @Override
        public void onError(Node node, Throwable throwable) {
        }


        @Override
        public GraphName getDefaultNodeName() {
          return GraphName.of(nodeName4);
        }
      }, configuration); */
    } catch (Exception e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

}
