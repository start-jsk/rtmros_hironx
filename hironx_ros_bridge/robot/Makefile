MODULE_LOAD_PATH = /opt/jsk/lib
INSTALL_DIR=/opt/jsk
ROBOT=HIRONX

ROBOT_PATH = ${INSTALL_DIR}/etc/${ROBOT}
LOG_PATH =   ${INSTALL_DIR}/var/log

all: configure install

configure:
	for file in *.in; do\
		sed s!@MODULE_LOAD_PATH@!${MODULE_LOAD_PATH}! $$file | \
		sed s!@ROBOT_PATH@!${ROBOT_PATH}! | \
		sed s!@LOG_PATH@!${LOG_PATH}! > `basename $$file .in`; \
	done

install:
	install -d ${INSTALL_DIR}/bin/  ${INSTALL_DIR}/etc/${ROBOT}/hrprtc/
	install rtcd.sh			${INSTALL_DIR}/bin/
	install Robot.conf		${INSTALL_DIR}/etc/${ROBOT}/hrprtc/
	install rtcdRobotMode.conf	${INSTALL_DIR}/etc/${ROBOT}/hrprtc/

