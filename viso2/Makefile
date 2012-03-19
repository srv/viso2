all: installed

TARBALL = build/libviso2.zip
TARBALL_URL = http://www.cvlibs.net/downloads/libviso2.zip
SOURCE_DIR = build/libviso2
MD5SUM_FILE = libviso2.zip.md5sum
UNPACK_CMD = unzip -d libviso2
TARBALL_PATCH = cmake_lib_build.patch

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	mkdir -p $(CURDIR)/include
	mkdir -p $(CURDIR)/lib
	cd $(SOURCE_DIR) && cmake . && make
	cp $(SOURCE_DIR)/src/*.h $(CURDIR)/include/
	cp $(SOURCE_DIR)/libviso2.a $(CURDIR)/lib/
	touch installed

clean:
	-rm -rf $(SOURCE_DIR) installed lib include

wipe:
	make clean
	-rm -rf build


.PHONY: clean

