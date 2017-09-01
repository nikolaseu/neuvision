TEMPLATE  = subdirs

SUBDIRS  += 3rdparty

SUBDIRS  += lib
lib.subdir  = Z3DScanner/lib
lib.depends = 3rdparty

SUBDIRS  += Z3DScanner
Z3DScanner.depends = lib
