#!/bin/bash


/home/plrf/lte_modem /dev/ttyAMA0 on

systemctl restart systemd-timesyncd


readonly HALT_PIN=19   # halt by GPIO-19 (BCM naming)
readonly SYSUP_PIN=17  # output SYS_UP signal on GPIO-17 (BCM naming)
readonly CHRG_PIN=5    # input to detect charging status
readonly STDBY_PIN=26   # input to detect standby status

gpio -g mode $HALT_PIN up
gpio -g mode $HALT_PIN in
gpio -g mode $CHRG_PIN up
gpio -g mode $CHRG_PIN in
gpio -g mode $STDBY_PIN up
gpio -g mode $STDBY_PIN in


gpio -g mode $SYSUP_PIN out
gpio -g write $SYSUP_PIN 1
sleep 0.1
gpio -g write $SYSUP_PIN 0
sleep 0.1
gpio -g write $SYSUP_PIN 1
sleep 0.1
gpio -g write $SYSUP_PIN 0
sleep 0.1
gpio -g mode $SYSUP_PIN in


CNODE_DIR=/home/plrf
CNODE_STAGES=${CNODE_DIR}/post_processing_stages

/home/plrf/cnode -n -t 1ms --post-process-file ${CNODE_DIR}/object_detect_tf.json --post-process-libs ${CNODE_STAGES} --lores-width 300 --lores-height 300 --object person --verbose 1

while tc -s qdisc show dev ppp0 | grep -q "backlog [1-9]"; do sleep 1; done
sleep 1

/home/plrf/lte_modem /dev/ttyAMA0 off

pinctrl set 13 ip pd  # WittyPi power-down notification
poweroff
