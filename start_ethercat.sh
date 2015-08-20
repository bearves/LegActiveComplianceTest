#!/bin/bash

sudo /opt/etherlab/etc/init.d/ethercat start
sudo /opt/etherlab/etc/init.d/ethercat restart
sleep 5
sudo /opt/etherlab/bin/ethercat sla
echo "Done"
