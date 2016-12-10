import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import se.goransson.qatja.messages.*; 
import se.goransson.qatja.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class mqttcontroller extends PApplet {





Qatja client;

public void setup()
{
  client = new Qatja(this);
  registerMethod("dispose", this);
  client.DEBUG = true;
  client.setKeepalive(5000);
  //client.setHost("gorilla.fritz.box");
  //client.setPort(1883);
  //client.setClientIdentifier("qatja-processing");
  client.connect("gorilla.fritz.box",1883,"test");
}

public void mqttCallback(MQTTPublish msg)
{
} 

public void keyPressed() {
  if (key==CODED)
  {
    String msg="0";
    switch (keyCode)
    {
    
      case  UP: msg="1";  break; //forward
      case  DOWN: msg="2" ;  break; //backward
      case  LEFT: msg="3";  break; //turn ccw
      case  RIGHT: msg="4";  break; //turn cw
      default: msg="0";  break; //stop
    }
    
    client.publish("/car/command",msg);
  }
}

public void dispose() {
  client.disconnect();
}


public void draw()
{
}
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "mqttcontroller" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
