all: jr3_driver  sample

jr3_driver: jr3_driver.cpp QNX101/jr3pci3.idm
	g++ -o jr3_driver jr3_driver.cpp

sample: sample.c
	gcc -o $@ $^

QNX101/jr3pci3.idm:
	curl http://www.nitta.co.jp/images/product/sensor/pdf_ifs/qnx101.zip -O qnx101.zip
	unzip qnx101.zip