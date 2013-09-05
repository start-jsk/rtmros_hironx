#!/bin/bash

commands="
  echo \"* Restart NameServer *\";
  nohup /opt/jsk/bin/NameServer.sh > /dev/null;
  echo \"* Restart ModelLoader *\";
  nohup /opt/jsk/bin/ModelLoader.sh > /dev/null;
  echo \"* Sleep 3sec *\";
  /usr/bin/env sleep 3;
  echo \"* Restart rtcd *\";
  slay -9 rtcd;
  echo \"* Finish Restart NameServer, ModelLoader and rtcd *\";
  "

ssh root@hiro014 -t $commands
