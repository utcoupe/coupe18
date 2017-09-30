# Raspberry Pi daemons

## Install

* run this from the repo root
* set `UTCOUPE_WORKSPACE` at `/etc/default/utcoupe` (the install script should have done that)
```bash
sudo su
echo "UTCOUPE_WORKSPACE=$PWD" >> /etc/default/utcoupe
exit
```
* create log directory :
```bash
sudo mkdir -p /var/log/utcoupe
```
* copy those files to `/etc/init.d/`
```bash
sudo cp daemons/utcoupe_* /etc/init.d/
```
* if necessary, install the corresponding npm packages globally :
```bash
npm install -g http-server
```
* reload the system daemons
```bash
sudo systemctl daemon-reload
```
* Services on boot :
```bash
update-rc.d utcoupe_server_http.sh defaults 99
update-rc.d utcoupe_server_websocket.sh defaults 99
```