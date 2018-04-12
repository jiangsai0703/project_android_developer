LOCAL_PATH:= $(call my-dir)

ifneq ($(BOARD_USE_CUSTOM_MEDIASERVEREXTENSIONS),true)
include $(CLEAR_VARS)
include  $(TOP)/frameworks/av/amlogic/config.mk
LOCAL_SRC_FILES := register.cpp
LOCAL_MODULE := libregistermsext
LOCAL_MODULE_TAGS := optional
LOCAL_CFLAGS := -Werror -Wall
include $(BUILD_STATIC_LIBRARY)
endif

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	main_mediaserver.cpp

LOCAL_SHARED_LIBRARIES := \
	libcamera_metadata \
	libcamera_client \
	libcameraservice \
	libresourcemanagerservice \
	libcutils \
	libmedia \
	libmediaplayerservice \
	libutils \
	libbinder \
	libicuuc \

ifneq ($(BOARD_USE_CUSTOM_MEDIASERVEREXTENSIONS),true)
LOCAL_SHARED_LIBRARIES += libmedia_amlogic_support
endif

LOCAL_STATIC_LIBRARIES := \
        libicuandroid_utils \
        libregistermsext

ifeq ($(BUILD_WITH_VIEWRIGHT_STB), true)
LOCAL_WHOLE_STATIC_LIBRARIES := libvmlogger
LOCAL_SHARED_LIBRARIES += libvmdrm libvmclient
endif

LOCAL_C_INCLUDES := \
    frameworks/av/media/libmediaplayerservice \
    frameworks/av/media/libmedia \
    frameworks/av/services/medialog \
    frameworks/av/services/audioflinger \
    frameworks/av/services/audiopolicy \
    frameworks/av/services/audiopolicy/common/managerdefinitions/include \
    frameworks/av/services/audiopolicy/common/include \
    frameworks/av/services/audiopolicy/engine/interface \
    frameworks/av/services/camera/libcameraservice \
    frameworks/av/services/mediaresourcemanager \

LOCAL_MODULE:= mediaserver
LOCAL_32_BIT_ONLY := true

LOCAL_INIT_RC := mediaserver.rc

LOCAL_CFLAGS := -Werror -Wall

include $(BUILD_EXECUTABLE)
