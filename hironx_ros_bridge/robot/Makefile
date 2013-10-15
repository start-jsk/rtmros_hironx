MODULE_LOAD_PATH = /opt/jsk/lib
INSTALL_DIR=/opt/jsk
ROBOT=HIRONX
ROBOT_IP=hiro014

ROBOT_PATH = ${INSTALL_DIR}/etc/${ROBOT}
LOG_PATH =   ${INSTALL_DIR}/var/log

all: configure

configure:
	for file in *.in; do\
		echo "configure $$file"; \
		sed s!@MODULE_LOAD_PATH@!${MODULE_LOAD_PATH}! $$file | \
		sed s!@INSTALL_DIR@!${INSTALL_DIR}! | \
		sed s!@ROBOT_PATH@!${ROBOT_PATH}! | \
		sed s!@LOG_PATH@!${LOG_PATH}! | \
		sed s!@ROBOT_IP@!${ROBOT_IP}! > `basename $$file .in`; \
	done
	@echo "\n\n\`make install\` to install generated files\n\n"

install:
	mkdir -p ${INSTALL_DIR}/bin/ ${INSTALL_DIR}/etc/${ROBOT}/hrprtc/
	cp rtcd.sh		${INSTALL_DIR}/bin/ && chmod u+x ${INSTALL_DIR}/bin/rtcd.sh
	cp NameServer.sh	${INSTALL_DIR}/bin/ && chmod u+x ${INSTALL_DIR}/bin/NameServer.sh
	cp ModelLoader.sh	${INSTALL_DIR}/bin/ && chmod u+x ${INSTALL_DIR}/bin/ModelLoader.sh
	cp startJSK.sh		${INSTALL_DIR}/bin/ && chmod u+x ${INSTALL_DIR}/bin/startJSK.sh
	cp Robot.conf		${INSTALL_DIR}/etc/${ROBOT}/hrprtc/
	cp rtcdRobotMode.conf	${INSTALL_DIR}/etc/${ROBOT}/hrprtc/
	cp PDgains.sav          ${INSTALL_DIR}/etc/${ROBOT}/hrprtc/

test.linux:
	make -f Makefile configure install \
		MODULE_LOAD_PATH=`rospack find hrpsys`/lib \
		INSTALL_DIR=/tmp
	@mkdir -p /tmp/etc/HIRONX/model/
	@cp `rospack find hironx_ros_bridge`/models/kawada-hironx.dae /tmp/etc/HIRONX/model/main.wrl
	@rosrun openrtm_aist rtm-naming 15005 || (sh -c '/bin/echo -e "\e[1;32m\nplease try\n pkill -9 openhrp-model-l; pkill -9 rtcd; pkill -9 omniNames\n and make test.inux again\e[0m"'; false)
	@rosrun openhrp3 openhrp-model-loader -ORBInitRef NameService=corbaloc:iiop:localhost:15005/NameService &
	@rosrun openrtm_aist rtcd -f /tmp/etc/${ROBOT}/hrprtc/rtcdRobotMode.conf &
	@sleep 3
	@echo -e ''
	@echo -e '\e[1;32mtry following commad to check rtcd for hiro\e[0m'
	@echo -e '\e[1;32m  rosrun hironx_ros_bridge hironx.py -ORBInitRef NameService=corbaloc:iiop:localhost:15005/NameService\e[0m'
	@echo -e '\e[1;32m  RTCTREE_NAMESERVERS=localhost:15005 rtls localhost:15005/\e[0m'
	@echo -e ''
	@echo -e '\e[1;31m[hit return key to quit]\e[0m'
	@read variable
	pkill -9 openhrp-model-loader || pkill -9 openhrp-model-l
	pkill -9 rtcd
	pkill -9 omniNames

