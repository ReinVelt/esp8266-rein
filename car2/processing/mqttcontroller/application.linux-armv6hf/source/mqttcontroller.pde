import se.goransson.qatja.messages.*;
import se.goransson.qatja.*;


Qatja client;

void setup()
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

void mqttCallback(MQTTPublish msg)
{
} 

void keyPressed() {
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

void dispose() {
  client.disconnect();
}


void draw()
{
}