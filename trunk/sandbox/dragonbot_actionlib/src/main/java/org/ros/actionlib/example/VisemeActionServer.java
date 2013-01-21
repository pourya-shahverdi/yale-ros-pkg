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
import dragon_msgs.VisemeActionFeedback;
import dragon_msgs.VisemeActionGoal;
import dragon_msgs.VisemeActionResult;
import dragon_msgs.VisemeFeedback;
import dragon_msgs.VisemeGoal;
import dragon_msgs.VisemeResult;

/**
 * The VisemeActionServer is a specialized DefaultActionServer that offers
 * services related to the Viseme action. The VisemeActionServer
 * completely hides the Generics approach of the DefaultActionServer's
 * implementation.
 * 
 * @author Alexander C. Perzylo, perzylo@cs.tum.edu
 * 
 * @see DefaultActionServer
 */
public class VisemeActionServer
    extends
    DefaultActionServer<VisemeActionFeedback, VisemeActionGoal, VisemeActionResult, VisemeFeedback, VisemeGoal, VisemeResult> {

  public VisemeActionServer(
      String nameSpace,
      ActionSpec<?, VisemeActionFeedback, VisemeActionGoal, VisemeActionResult, VisemeFeedback, VisemeGoal, VisemeResult> spec,
      ActionServerCallbacks<VisemeActionFeedback, VisemeActionGoal, VisemeActionResult, VisemeFeedback, VisemeGoal, VisemeResult> callbacks) {

    super(nameSpace, spec, callbacks);

  }
}
