/*
 * Copyright (C) 2011 Alexander Perzylo, Technische Universität München
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.actionlib.example;

import org.ros.actionlib.ActionSpec;
import org.ros.actionlib.server.ActionServerCallbacks;
import org.ros.actionlib.server.DefaultActionServer;
import dragon_msgs.LookatActionFeedback;
import dragon_msgs.LookatActionGoal;
import dragon_msgs.LookatActionResult;
import dragon_msgs.LookatFeedback;
import dragon_msgs.LookatGoal;
import dragon_msgs.LookatResult;

/**
 * The LookatActionServer is a specialized DefaultActionServer that offers
 * services related to the Lookat action. The LookatActionServer
 * completely hides the Generics approach of the DefaultActionServer's
 * implementation.
 * 
 * @author Alexander C. Perzylo, perzylo@cs.tum.edu
 * 
 * @see DefaultActionServer
 */
public class LookatActionServer
    extends
    DefaultActionServer<LookatActionFeedback, LookatActionGoal, LookatActionResult, LookatFeedback, LookatGoal, LookatResult> {

  public LookatActionServer(
      String nameSpace,
      ActionSpec<?, LookatActionFeedback, LookatActionGoal, LookatActionResult, LookatFeedback, LookatGoal, LookatResult> spec,
      ActionServerCallbacks<LookatActionFeedback, LookatActionGoal, LookatActionResult, LookatFeedback, LookatGoal, LookatResult> callbacks) {

    super(nameSpace, spec, callbacks);

  }
}
