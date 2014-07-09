all: jr3pci3_driver  sample

jr3pci3_driver: jr3pci3_driver.cpp QNX101/jr3pci3.idm
	gcc -o $@ $^

sample: sample.c
	gcc -o $@ $^

QNX101/jr3pci3.idm:
	wget http://www.nitta.co.jp/images/product/sensor/pdf_ifs/qnx101.zip -O qnx.zip
	unzip qnx.zip