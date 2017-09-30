#!/bin/bash
### BEGIN INIT INFO
# Provides:		utcoupe_hokuyo
# Required-Start:
# Required-Stop:
# Default-Start:	2 3 4 5
# Default-Stop:		0 1 6
# Short-Description:	UTCoupe node client for the hokuyo
### END INIT INFO

PID_FILE=/var/log/utcoupe/utcoupe_hokuyo_deamon.pid

start_hokuyo() {
	echo -n "Starting the utcoupe_hokuyo node client..."
	UTCOUPE_WORKSPACE=$UTCOUPE_WORKSPACE start-stop-daemon --start -m --pidfile $PID_FILE --background --exec /usr/local/bin/node -- $UTCOUPE_WORKSPACE/hokuyo/main.js
	echo "done."
}

stop_hokuyo() {
	echo -n "Terminating the utcoupe_hokuyo node client...."
	start-stop-daemon -K -v -p $PID_FILE --exec /usr/local/bin/node
	echo "done."
}

# Retrieves the UTCOUPE_WORKSPACE env variable
[ -f /etc/default/utcoupe ] && . /etc/default/utcoupe
if [ -z "$UTCOUPE_WORKSPACE" ] ;  then
  echo "UTCOUPE_WORKSPACE is not set, please set it in /etc/default/utcoupe" >&2
  exit 1
fi

case "$1" in
start)
	start_hokuyo
	;;
stop)
	stop_hokuyo
  ;;
restart|reload|force-reload)
	stop_hokuyo
	start_hokuyo
	;;
*)
  echo "Usage: /etc/init.d/utcoupe_hokuyo {start|stop|restart}"
  exit 1
  ;;
esac

exit 0
