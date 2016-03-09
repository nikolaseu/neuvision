TEMPLATE = subdirs

SUBDIRS += src

SUBDIRS += patterns
patterns.depends = src

SUBDIRS += slsystems
slsystems.depends = src
