all: installed

TARBALL = build/libviso2.zip
TARBALL_URL = http://www.cvlibs.net/software/libviso2/libviso2.zip
SOURCE_DIR = build/libviso2
MD5SUM_FILE = libviso2.zip.md5sum
UNPACK_CMD = unzip
include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && cmake . && make
	cp $(SOURCE_DIR)/src/*.h `rospack find viso2`/include
	cp $(SOURCE_DIR)/libviso2.a `rospack find viso2`/lib
	touch installed

clean:
	-rm -rf $(SOURCE_DIR) installed lib/libviso2.a include/*.h

.PHONY: clean

