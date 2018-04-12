
#define LOG_TAG "ACodec_amlogic"

#include <inttypes.h>
#include <gui/Surface.h>
#include <media/stagefright/ACodec.h>
#include <OMX_IndexExt.h>

#include "omx/OMXUtils.h"


namespace android {
void ACodec::configureCodec_amlogic_processVideoHwComposed(bool videoDecoder, sp<ANativeWindow> nativeWindow) {

	//ALOGV("configureCodec_amlogic_processVideoHwComposed,videoDecoder %d",videoDecoder);
	if (!videoDecoder) {
		return;
	}

	int systemComposer = 1;
	status_t err = nativeWindow->query(
			nativeWindow.get(), NATIVE_WINDOW_QUEUES_TO_WINDOW_COMPOSER,
			&systemComposer);
	ALOGD("query systemComposer %d",systemComposer);
	if ((err == NO_ERROR) && !systemComposer) {
		ALOGI("Setting OMX_IndexParamVideoHWComposed");
		OMX_INDEXTYPE index;
		if (mOMX->getExtensionIndex(mNode,"OMX.amlogic.index.videoHWComposed", &index) == OK) {
			OMX_CONFIG_BOOLEANTYPE params;
			InitOMXParams(&params);
			params.bEnabled = OMX_TRUE;
			mOMX->setParameter(mNode, index, &params, sizeof(params));
		}
	}

}

} //namespace android
