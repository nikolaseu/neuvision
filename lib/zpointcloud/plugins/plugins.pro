TEMPLATE = subdirs

SUBDIRS += zqtgeometryloaders

# zpcl is included if env variable PCL_DIR is set
win32:PCL_DIR = $$(PCL_DIR)
macx:PCL_DIR=/usr/local/opt/pcl

isEmpty(PCL_DIR){
    message('zpcl: PCL_DIR not set, skipping plugin')
} else {
    message('zpcl: Building plugin using PCL_DIR='$$PCL_DIR)
    SUBDIRS += zpcl
}

message('Configured for building point cloud plugins:' $$SUBDIRS)
