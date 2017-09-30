#!/bin/bash
### BEGIN INIT INFO
# Provides:		utcoupe_server_websocket
# Required-Start:
# Required-Stop:
# Default-Start:	2 3 4 5
# Default-Stop:		0 1 6
# Short-Description:	UTCoupe node client for the websocket server
### END INIT INFO

PID_FILE=/var/log/utcoupe/utcoupe_server_websocket_daemon.pid

start_server() {
	echo -n "Starting the utcoupe_server node client..."
	UTCOUPE_WORKSPACE=$UTCOUPE_WORKSPACE start-stop-daemon --start -m --pidfile $PID_FILE --background --exec /usr/bin/node -- $UTCOUPE_WORKSPACE/utcoupe/utcoupe.js
	echo "done."
}

stop_server() {
	echo -n "Terminating the utcoupe_server node client...."
	start-stop-daemon -K -v -p $PID_FILE --exec /usr/bin/node
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
	start_server
	;;
stop)
	stop_server
  ;;
restart|reload|force-reload)
	stop_server
	start_server
	;;
*)
  echo "Usage: /etc/init.d/utcoupe_server {start|stop|restart}"
  exit 1
  ;;
esac

exit 0
