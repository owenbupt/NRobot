# NRobot top level Makefile #####################################################
.SILENT:

all: debug

debug:
	cmake -H. -Bbuild/debug -DCMAKE_BUILD_TYPE=Debug
	cmake --build build/debug -- -j3

release:
	cmake -H. -Bbuild/release -DCMAKE_BUILD_TYPE=Release
	cmake --build build/release -- -j3

install: release
	cd build/release/; make install

uninstall:
	sudo ldconfig -n /usr/local/lib/

clean:
	rm -rf bin/*
	rm -rf build/*
