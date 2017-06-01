# NPart top level Makefile #####################################################
.SILENT:

all: debug

debug:
	cmake -H. -Bbuild/debug -DCMAKE_BUILD_TYPE=Debug
	cmake --build build/debug -- -j3

release:
	cmake -H. -Bbuild/release -DCMAKE_BUILD_TYPE=Release
	cmake --build build/release -- -j3

install: release
	cd build/release/; sudo make install

uninstall:
	sudo rm -f /usr/local/include/NPart/NPBase.hpp
	sudo rm -f /usr/local/include/NPart/NPClip.hpp
	sudo rm -f /usr/local/include/NPart/NPart.hpp
	sudo rm -f /usr/local/include/NPart/NPSDL.hpp
	sudo rm -f /usr/local/include/NPart/NPConfig.hpp
	sudo rmdir --ignore-fail-on-non-empty /usr/local/include/NPart/

	sudo rm -f /usr/local/lib/libnpart.so
	sudo rm -f /usr/local/lib/libnpsdl.so

	sudo ldconfig -n /usr/local/lib/

clean:
	sudo rm -rf bin/*
	sudo rm -rf build/*

fix:
	sudo cp sdl2-config.cmake /usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake
