/*
 * dragonbot_android
 * Copyright (c) 2012, David Feil-Seifer
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the yale-ros-pkgs nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package edu.yale.dragonbot_driver;

import com.google.common.base.Preconditions;

import android.app.Application;
import android.content.res.*;

import org.ros.node.NodeMainExecutor;

import java.net.*;

// RosActivity Includes
import android.content.ServiceConnection;
import android.content.Intent;
import android.content.ComponentName;
import android.os.AsyncTask;
import android.os.IBinder;
import android.widget.Toast;


public class DragonbotAndroidApplication extends Application {
  private static DragonbotAndroidApplication mSingleton;

  private static URI mMasterURI;
  private static NodeMainExecutorService mNodeMainExecutorService;
  private static DragonbotAndroidDriver dragonbotNode;

  private static AssetManager assetManager;

  public static DragonbotAndroidApplication getInstance() {return mSingleton;}

  // exchange asset manager
  public static void setAssetManager(AssetManager am) {assetManager = am;}
  public static AssetManager getAssetManager() {return assetManager;}

  // exchange main ROS node
  public static void setDragonbotNode(DragonbotAndroidDriver dn) {dragonbotNode = dn;}
  public static DragonbotAndroidDriver getDragonbotNode() {return dragonbotNode;}

  // exchange master URI
  public static void setMasterURI(URI u) {
    mMasterURI = u;
    mNodeMainExecutorService.setMasterUri(mMasterURI);
  }
  public static URI getMasterURI(){return mMasterURI;}

  // exchange NodeMainExecutor
  public static NodeMainExecutor getNodeMainExecutor(){return mNodeMainExecutorService;}

  /***************************************/

  // RosActivity Transplants
  private ServiceConnection nodeMainExecutorServiceConnection;
  private String notificationTicker;
  private String notificationTitle;


  private final class NodeMainExecutorServiceConnection implements ServiceConnection {
    @Override
    public void onServiceConnected(ComponentName name, IBinder binder) {
      mNodeMainExecutorService = ((NodeMainExecutorService.LocalBinder) binder).getService();
      mNodeMainExecutorService.addListener(new NodeMainExecutorServiceListener() {
        @Override
        public void onShutdown(NodeMainExecutorService nodeMainExecutorService) {
          //RosActivity.this.finish();
          android.os.Process.killProcess(android.os.Process.myPid());
        }
      });
    }

    @Override
    public void onServiceDisconnected(ComponentName name) {
    }
  };

  /******************************************/

  @Override
  public final void onCreate(){
    super.onCreate();
    assetManager = getAssets();
    mSingleton = this;

    // notification ticket stuff
    //this.notificationTicker = notificationTicker;
    //this.notificationTitle = notificationTitle;
    nodeMainExecutorServiceConnection = new NodeMainExecutorServiceConnection();
  
    notificationTicker = "Tap to shutdown";
    notificationTitle = "DragonbotAndroid";

    Intent intent = new Intent(this, NodeMainExecutorService.class);
    intent.setAction(NodeMainExecutorService.ACTION_START);
    intent.putExtra(NodeMainExecutorService.EXTRA_NOTIFICATION_TICKER, notificationTicker);
    intent.putExtra(NodeMainExecutorService.EXTRA_NOTIFICATION_TITLE, notificationTitle);
    startService(intent);
    Preconditions.checkState(
        bindService(intent, nodeMainExecutorServiceConnection, BIND_AUTO_CREATE),
        "Failed to bind NodeMainExecutorService.");


  }




}
