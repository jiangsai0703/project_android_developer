/*
 * Copyright (C) 2011 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//#define LOG_NDEBUG 0
#define LOG_TAG "SoftOMXPlugin"
#include <utils/Log.h>

#include "SoftOMXPlugin.h"
#include "include/SoftOMXComponent.h"

#include <media/stagefright/foundation/ADebug.h>
#include <media/stagefright/foundation/AString.h>

#include <dlfcn.h>

namespace android {

static const struct {
    const char *mName;
    const char *mLibNameSuffix;
    const char *mRole;

} kComponents[] = {
    { "OMX.google.aac.decoder", "aacdec", "audio_decoder.aac" },
    { "OMX.google.aac.encoder", "aacenc", "audio_encoder.aac" },
    { "OMX.google.amrnb.decoder", "amrdec", "audio_decoder.amrnb" },
    { "OMX.google.amrnb.encoder", "amrnbenc", "audio_encoder.amrnb" },
    { "OMX.google.amrwb.decoder", "amrdec", "audio_decoder.amrwb" },
    { "OMX.google.amrwb.encoder", "amrwbenc", "audio_encoder.amrwb" },
    { "OMX.google.h264.decoder", "avcdec", "video_decoder.avc" },
#ifdef WITH_AMLOGIC_MEDIA_EX_SUPPORT
    { "OMX.google.vp6.decoder", "amsoftdec", "video_decoder.amvp6" },
    { "OMX.google.vp6a.decoder", "amsoftdec", "video_decoder.amvp6a" },
    { "OMX.google.vp6f.decoder", "amsoftdec", "video_decoder.amvp6f" },
    { "OMX.google.vp8.decoder", "amsoftdec", "video_decoder.vp8" },
    { "OMX.google.h264.decoder", "amsoftdec", "video_decoder.avc" },
    { "OMX.google.rm10.decoder", "amsoftdec", "video_decoder.rm" },
    { "OMX.google.rm20.decoder", "amsoftdec", "video_decoder.rm" },
    { "OMX.google.rm40.decoder", "amsoftdec", "video_decoder.rm" },
    { "OMX.google.wmv2.decoder", "amsoftdec", "video_decoder.wmv2" },
    { "OMX.google.wmv1.decoder", "amsoftdec", "video_decoder.wmv1" },
//audio enhanced decoder here
    { "OMX.google.ac3.decoder", "dcvdec", "audio_decoder.ac3" },
    { "OMX.google.eac3.decoder", "dcvdec", "audio_decoder.eac3" },
    { "OMX.google.dtshd.decoder", "dtshd", "audio_decoder.dtshd"},
    { "OMX.google.adif.decoder", "adifdec", "audio_decoder.adif" },
    { "OMX.google.adts.decoder", "adtsdec", "audio_decoder.adts" },
    { "OMX.google.latm.decoder", "latmdec", "audio_decoder.latm" },
    { "OMX.google.mp2.decoder", "mp2dec", "audio_decoder.mp2" },
    { "OMX.google.wma.decoder", "wmadec", "audio_decoder.wma" },
    { "OMX.google.wmapro.decoder", "wmaprodec", "audio_decoder.wmapro" },
    { "OMX.google.truehd.decoder", "truehddec", "audio_decoder.truehd" },
    { "OMX.google.ffmpeg.decoder", "amsoftadec", "audio_decoder.ffmpeg" },
#else
    { "OMX.google.vp8.decoder", "vpxdec", "video_decoder.vp8" },
#endif
    { "OMX.google.h264.encoder", "avcenc", "video_encoder.avc" },
    { "OMX.google.hevc.decoder", "hevcdec", "video_decoder.hevc" },
    { "OMX.google.g711.alaw.decoder", "g711dec", "audio_decoder.g711alaw" },
    { "OMX.google.g711.mlaw.decoder", "g711dec", "audio_decoder.g711mlaw" },
    { "OMX.google.mpeg2.decoder", "mpeg2dec", "video_decoder.mpeg2" },
    { "OMX.google.h263.decoder", "mpeg4dec", "video_decoder.h263" },
    { "OMX.google.h263.encoder", "mpeg4enc", "video_encoder.h263" },
    { "OMX.google.mpeg4.decoder", "mpeg4dec", "video_decoder.mpeg4" },
    { "OMX.google.mpeg4.encoder", "mpeg4enc", "video_encoder.mpeg4" },
    { "OMX.google.mp3.decoder", "mp3dec", "audio_decoder.mp3" },
    { "OMX.google.vorbis.decoder", "vorbisdec", "audio_decoder.vorbis" },
    { "OMX.google.opus.decoder", "opusdec", "audio_decoder.opus" },
    { "OMX.google.vp9.decoder", "vpxdec", "video_decoder.vp9" },
    { "OMX.google.vp8.encoder", "vpxenc", "video_encoder.vp8" },
    { "OMX.google.raw.decoder", "rawdec", "audio_decoder.raw" },
    { "OMX.google.flac.encoder", "flacenc", "audio_encoder.flac" },
    { "OMX.google.gsm.decoder", "gsmdec", "audio_decoder.gsm" },
};

static const size_t kNumComponents =
    sizeof(kComponents) / sizeof(kComponents[0]);

SoftOMXPlugin::SoftOMXPlugin() {
}

OMX_ERRORTYPE SoftOMXPlugin::makeComponentInstance(
        const char *name,
        const OMX_CALLBACKTYPE *callbacks,
        OMX_PTR appData,
        OMX_COMPONENTTYPE **component) {
    ALOGV("makeComponentInstance '%s'", name);

    for (size_t i = 0; i < kNumComponents; ++i) {
        if (strcmp(name, kComponents[i].mName)) {
            continue;
        }

        AString libName = "libstagefright_soft_";
        libName.append(kComponents[i].mLibNameSuffix);
        libName.append(".so");

        void *libHandle = dlopen(libName.c_str(), RTLD_NOW);

        if (libHandle == NULL) {
            /*support muti-component support same format.
            for vp8 decoder.
            */
            ALOGE("unable to dlopen %s: %s, try next.\n", libName.c_str(), dlerror());
            continue;
        }

        typedef SoftOMXComponent *(*CreateSoftOMXComponentFunc)(
                const char *, const OMX_CALLBACKTYPE *,
                OMX_PTR, OMX_COMPONENTTYPE **);

        CreateSoftOMXComponentFunc createSoftOMXComponent =
            (CreateSoftOMXComponentFunc)dlsym(
                    libHandle,
                    "_Z22createSoftOMXComponentPKcPK16OMX_CALLBACKTYPE"
                    "PvPP17OMX_COMPONENTTYPE");

        if (createSoftOMXComponent == NULL) {
            dlclose(libHandle);
            libHandle = NULL;

            return OMX_ErrorComponentNotFound;
        }

        sp<SoftOMXComponent> codec =
            (*createSoftOMXComponent)(name, callbacks, appData, component);

        if (codec == NULL) {
            dlclose(libHandle);
            libHandle = NULL;

            return OMX_ErrorInsufficientResources;
        }

        OMX_ERRORTYPE err = codec->initCheck();
        if (err != OMX_ErrorNone) {
            dlclose(libHandle);
            libHandle = NULL;

            return err;
        }

        codec->incStrong(this);
        codec->setLibHandle(libHandle);

        return OMX_ErrorNone;
    }

    return OMX_ErrorInvalidComponentName;
}

OMX_ERRORTYPE SoftOMXPlugin::destroyComponentInstance(
        OMX_COMPONENTTYPE *component) {
    SoftOMXComponent *me =
        (SoftOMXComponent *)
            ((OMX_COMPONENTTYPE *)component)->pComponentPrivate;

    me->prepareForDestruction();

    void *libHandle = me->libHandle();

    CHECK_EQ(me->getStrongCount(), 1);
    me->decStrong(this);
    me = NULL;

    dlclose(libHandle);
    libHandle = NULL;

    return OMX_ErrorNone;
}

OMX_ERRORTYPE SoftOMXPlugin::enumerateComponents(
        OMX_STRING name,
        size_t /* size */,
        OMX_U32 index) {
    if (index >= kNumComponents) {
        return OMX_ErrorNoMore;
    }

    strcpy(name, kComponents[index].mName);

    return OMX_ErrorNone;
}

OMX_ERRORTYPE SoftOMXPlugin::getRolesOfComponent(
        const char *name,
        Vector<String8> *roles) {
    for (size_t i = 0; i < kNumComponents; ++i) {
        if (strcmp(name, kComponents[i].mName)) {
            continue;
        }

        roles->clear();
        roles->push(String8(kComponents[i].mRole));

        return OMX_ErrorNone;
    }

    return OMX_ErrorInvalidComponentName;
}

}  // namespace android
