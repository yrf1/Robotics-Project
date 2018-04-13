include $(shell rospack find mk)/cmake.mk

.SILENT:

all: build build/CMakeLists.txt.copy
	$(MAKE) --no-print-directory -C build

build:
	mkdir -p build

build/CMakeLists.txt.copy: CMakeLists.txt
	cp CMakeLists.txt build/CMakeLists.txt.copy
	cd build && cmake ..

clean:
	rm -rf build bin lib src/cs403proj
