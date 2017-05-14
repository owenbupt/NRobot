# NRobot makefile ##############################################################
.SILENT:
.PHONY: release debug clean

release:
	cmake -H. -Bbuild/release -DCMAKE_BUILD_TYPE=Release
	cmake --build build/release -- -j3

debug:
	cmake -H. -Bbuild/debug -DCMAKE_BUILD_TYPE=Debug
	cmake --build build/debug -- -j3

clean:
	rm -rf bin
	rm -rf build

# End of makefile ##############################################################
