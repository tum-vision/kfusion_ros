all: clean kfusion
    
clean:
	rm -rf kfusion_src include lib bin

kfusion:
	git clone https://github.com/GerhardR/kfusion.git kfusion_src && \
	cd kfusion_src && \
	git checkout 242cf4b35fc2e89e33bdfd06ced69c69a1e5f832 && \
	git submodule update --init && \
	git apply ../CMakeLists.txt.patch && \
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
	
