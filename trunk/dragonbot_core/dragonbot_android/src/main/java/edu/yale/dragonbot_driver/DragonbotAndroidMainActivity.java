/*
 * dragonbot_driver
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


import android.os.Bundle;
import android.app.Activity;
import android.content.Intent;
import android.content.res.AssetManager;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.content.pm.ResolveInfo;
import android.net.Uri;
import android.os.Bundle;
import android.view.*;
import android.widget.*;

import java.io.*;
//import java.io.InputStream;

import edu.yale.dragonbot_driver.*;

import android.util.Log;

import mcbmini.utils.XMLUtils;
import mcbmini.utils.XMLUtils.XMLResults;

import org.ros.node.NodeConfiguration;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;

public class DragonbotAndroidMainActivity extends Activity {

  /**
   * The key with which the last used {@link URI} will be stored as a
   * preference.
   */
  private static final String PREFS_KEY_NAME = "URI_KEY";

  private String masterUri;
  private EditText uriText;

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.uri_chooser);
    uriText = (EditText) findViewById(R.id.master_chooser_uri);
    // Get the URI from preferences and display it. Since only primitive types
    // can be saved in preferences the URI is stored as a string.
    masterUri =
        getPreferences(MODE_PRIVATE).getString(PREFS_KEY_NAME,
            NodeConfiguration.DEFAULT_MASTER_URI.toString());
    uriText.setText(masterUri);

/*
    // Load XML for parsing.
    AssetManager assetManager = getAssets();

    InputStream xmlFileStream = null;
    try {
      xmlFileStream = assetManager.open("example_motor_config.xml");
    } catch (IOException e) {
      Log.e("xml", e.getMessage() );
    }
    XMLResults results = null;
    try {
      results = AndroidXMLUtils.parseMCBMiniConfigStream(xmlFileStream);
    } catch (Exception e) {
      Log.e("xml", e.getMessage());
    }
*/
  }

  @Override
  public void onActivityResult(int requestCode, int resultCode, Intent intent) {
    // If the Barcode Scanner returned a string then display that string.
    if (requestCode == 0) {
      if (resultCode == RESULT_OK) {
        Preconditions.checkState(intent.getStringExtra("SCAN_RESULT_FORMAT").equals("TEXT_TYPE"));
        String contents = intent.getStringExtra("SCAN_RESULT");
        uriText.setText(contents);
      }
    }
  }


  public void okButtonClicked(View unused) {
    // Get the current text entered for URI.
    String userUri = uriText.getText().toString();

    if (userUri.length() == 0) {
      // If there is no text input then set it to the default URI.
      userUri = NodeConfiguration.DEFAULT_MASTER_URI.toString();
      uriText.setText(userUri);
      Toast.makeText(DragonbotAndroidMainActivity.this, "Empty URI not allowed.", Toast.LENGTH_SHORT).show();
    }
    // Make sure the URI can be parsed correctly.
    try {
      new URI(userUri);
    } catch (URISyntaxException e) {
      Toast.makeText(DragonbotAndroidMainActivity.this, "Invalid URI.", Toast.LENGTH_SHORT).show();
      return;
    }

    // If the displayed URI is valid then pack that into the intent.
    masterUri = userUri;
    SharedPreferences.Editor editor = getPreferences(MODE_PRIVATE).edit();
    editor.putString(PREFS_KEY_NAME, masterUri);
    editor.commit();
    // Package the intent to be consumed by the calling activity.
    Intent intent = new Intent();
    intent.putExtra("ROS_MASTER_URI", masterUri);
    setResult(RESULT_OK, intent);
    finish();
  }

  public void cancelButtonClicked(View unused) {
    setResult(RESULT_CANCELED);
    finish();
  }



}
