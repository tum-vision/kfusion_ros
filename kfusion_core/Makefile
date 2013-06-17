all: clean kfusion
    
clean:
	rm -rf kfusion_src include lib bin

kfusion:
	git clone https://github.com/GerhardR/kfusion.git kfusion_src && \
	cd kfusion_src && \
	git submodule update --init && \
	mkdir build && \
	cd build && \
	cmake .. && \
	make; \
	cd ../.. && \
	mkdir include && \
	mkdir lib && \
	mkdir bin && \
	cp kfusion_src/*.h include/ && \
	cp kfusion_src/build/libkfusion.a lib/ && \
	cp kfusion_src/build/devicequery bin/
	
