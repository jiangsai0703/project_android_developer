LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	AmMediaDefsExt.cpp	\
	AmExtratorSupport.cpp\
	AmLoadAmlogicScreenMediaSource.cpp\
	AmLoadAmlogicMetadataRetriever.cpp\
	AmLoadAmlogicPlayers.cpp\
	AmSupportModules.cpp\
	AmSharedLibrary.cpp\


LOCAL_C_INCLUDES+= \
	$(TOP)/frameworks/av/media/libstagefright/include  \

LOCAL_SHARED_LIBRARIES := \
        libbinder \
        libcutils \
        libdl \
        libutils \


#	 libmediaplayerservice\


include  $(TOP)/frameworks/av/amlogic/config.mk
LOCAL_MODULE:= libmedia_amlogic_support
include $(BUILD_SHARED_LIBRARY)

