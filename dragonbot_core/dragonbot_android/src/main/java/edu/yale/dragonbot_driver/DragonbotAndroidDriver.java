/*
 * dragonbot_android_driver
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

import edu.yale.dragonbot_driver.*;

import android.os.Bundle;
import android.app.Activity;
import android.content.res.AssetManager;

import java.io.*;

import android.util.Log;

import mcbmini.utils.XMLUtils;
import mcbmini.utils.XMLUtils.XMLResults;
import mcbmini.AndroidIOIOMCBMiniServer;



public class DragonbotAndroidDriver extends DragonbotDriver {

  private static DragonbotAndroidDriver instance;
  //private static NodeConfiguration nc;

  public static DragonbotAndroidDriver getInstance()
  {
    if(instance == null)
    {
      instance = new DragonbotAndroidDriver();
    }
    return instance;
  }


  // uses regular file system, override for android
  protected int parseConfigFile( String filename )
  {
    // Load XML for parsing.
    AssetManager assetManager = DragonbotAndroidApplication.getAssetManager();

    InputStream xmlFileStream = null;
    try {
      xmlFileStream = assetManager.open("dragonbot_config.xml");
    } catch (IOException e) {
      Log.e("dragonbot_android", e.getMessage() );
      return -1;
    }

    try {
      results = AndroidXMLUtils.parseMCBMiniConfigStream(xmlFileStream);
    } catch (Exception e) {
      Log.e("dragonbot_android", e.getMessage());
      return -1;
    }

    return 0;
    //finish();
  }

  // uses tty Serial, override for Android
  protected int initServer()
  {
    int ioio_rx_pin = 12;
    int ioio_tx_pin = 11;

    try {
      // This allows us to not have a connected stack, for debugging purposes
      server = new AndroidIOIOMCBMiniServer(results.boards, ioio_rx_pin, ioio_tx_pin);
    } catch (IOException e) {

      // TODO Auto-generated catch block
      Log.e( "dragonbot_android", "error opening MCBMini server: " + e.toString() );
      // TODO: double-check to make sure correct behavior
      return -1;
    }

    return 0;
  }
}
