package org.ros.actionlib.example;

import org.ros.namespace.GraphName;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.Node;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import prg.content.dragonbot.teleopstandalone.DragonBotComm;
import prg.content.dragonbot.teleopstandalone.Constants.*;


/**
 * A main for running the Dragon simple action server.
 */
public class RunR1D1 {
  public static void main(String[] args) {
    main();
  }

  public static void main() {
    DragonBotComm comm = new DragonBotComm();

    System.out.println("before sleep ... " );
    for( int i = 0; i < 1000; i++ )
    {
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {

    }
    System.out.println( comm.getFaceDisplayFPS());
    comm.update();
    comm.sendExpression(EXPRESSION.EXPRESSION_ANGRY);
    comm.update();
    }
    try {
      Thread.sleep(100000);
    } catch (InterruptedException e) {

    }
   System.out.println( "... after sleep" );
  }
}

