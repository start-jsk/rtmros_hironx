all: dynpick_driver sample_dynpick test-com

dynpick_driver: dynpick_driver.cpp
	g++ -o dynpick_driver dynpick_driver.cpp

sample_dynpick: sample_dynpick.c
	gcc -o $@ $^

test-com: test-com.c
	gcc -o $@ $^

