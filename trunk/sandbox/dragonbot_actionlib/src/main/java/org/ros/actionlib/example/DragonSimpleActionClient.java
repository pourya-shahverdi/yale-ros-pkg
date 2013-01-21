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

import org.ros.actionlib.client.SimpleActionClient;
import org.ros.exception.RosException;
import dragon_msgs.DragonActionFeedback;
import dragon_msgs.DragonActionGoal;
import dragon_msgs.DragonActionResult;
import dragon_msgs.DragonFeedback;
import dragon_msgs.DragonGoal;
import dragon_msgs.DragonResult;

/**
 * The DragonSimpleActionClient is a specialized SimpleActionClient that is
 * intended to work with an action server offering services related to the
 * Dragon action. The DragonSimpleActionClient completely hides the
 * Generics approach of the SimpleActionClient's implementation.
 * 
 * @author Alexander C. Perzylo, perzylo@cs.tum.edu
 * 
 * @see SimpleActionClient
 */
public class DragonSimpleActionClient
    extends
    SimpleActionClient<DragonActionFeedback, DragonActionGoal, DragonActionResult, DragonFeedback, DragonGoal, DragonResult> {

  public DragonSimpleActionClient(String nameSpace, DragonActionSpec spec)
      throws RosException {
    super(nameSpace, spec);
  }

}
