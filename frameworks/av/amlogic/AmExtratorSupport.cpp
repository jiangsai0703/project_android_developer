/*
 * Copyright (C) 2012 The Android Open Source Project
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
#define LOG_TAG "AmExtractorSupport"
#include <utils/Log.h>

#include <dlfcn.h>

#include <media/stagefright/foundation/AMessage.h>
#include <media/stagefright/DataSource.h>
#include <media/stagefright/MediaExtractor.h>
#include <utils/String8.h>

#include <media/stagefright/AmMediaDefsExt.h>

namespace android
{

static void *gFfmpegLibHandle = NULL;

static void loadFFmpegAdapter(void)
{
    static int init = 0;
    if (init) {
        return;
    }
    init++;

    gFfmpegLibHandle = dlopen("libamffmpegadapter.so", RTLD_NOW);
    if (gFfmpegLibHandle) {
        ALOGI("load libamffmpegadapter OK\n");
    } else {
        ALOGE("load libamffmpegadapter failed!\n");
    }
    return ;
}

bool sniffFFmpegFormat(
    const sp<DataSource> &source, String8 *mimeType,
    float *confidence, sp<AMessage> *meta)
{
    static DataSource::SnifferFunc sniffFfmpegFormat = NULL;
    loadFFmpegAdapter();
    if (gFfmpegLibHandle && !sniffFfmpegFormat) {
        sniffFfmpegFormat = (DataSource::SnifferFunc)dlsym(
                                gFfmpegLibHandle, "amsniffFFmpegFormat");
        ALOGI("Load ffmpeg sniffer %s\n", sniffFfmpegFormat == NULL ? "Failed" : "OK");
    }
    if (sniffFfmpegFormat) {
        return (*sniffFfmpegFormat)(source, mimeType, confidence, meta);
    }
    return false;
}
sp<MediaExtractor> createFFmpegExtractor(
    const sp<DataSource> &source, const char *mime)
{
    typedef MediaExtractor * (*CreateAmExtractor_t)(
        const sp<DataSource> &source, const char * mime);
    static CreateAmExtractor_t createAmExtractor = NULL;
    loadFFmpegAdapter();
    if (gFfmpegLibHandle && !createAmExtractor) {
        createAmExtractor = (CreateAmExtractor_t)dlsym(
                                gFfmpegLibHandle, "amcreateFFmpegExtractor");
        ALOGI("Load ffmpeg extrator %s\n", createAmExtractor == NULL ? "Failed" : "OK");
    }
    if (createAmExtractor) {
        sp<MediaExtractor> AmExtractor =
            (*createAmExtractor)(source, mime);
        if (AmExtractor != NULL) {
            return AmExtractor;
        }
    }
    return NULL;
}

static void *gExExtratorHandle = NULL;


static void loadAmlogicExtratorAdapters(void)
{
    static int init = 0;
    if (init) {
        return;
    }
    init++;
    gExExtratorHandle = dlopen("libstagefright_extrator.so", RTLD_NOW);

    if (gExExtratorHandle) {
        ALOGI("load libstagefright_extrator OK\n");
    } else {
        ALOGE("load libstagefright_extrator open failed!\n");
    }
    return ;
}



int registerAmExExtratorSniffers(void)
{
    typedef int (*registerExSniffs_t)(void);
    registerExSniffs_t registerExSniffs = NULL;
    static int registered = 0;
    if (registered) {
        return 0;
    }
    registered ++;

    loadAmlogicExtratorAdapters();
    if (!gExExtratorHandle || registerExSniffs) {
        return 0;
    }

    if (gExExtratorHandle) {
        registerExSniffs = (registerExSniffs_t)dlsym(
                               gExExtratorHandle, "am_registerAmExSniffs");
        ALOGI("find registerExSniffs %s\n", registerExSniffs != 0 ? "OK" : "Failed");
        if (registerExSniffs) {
            int ret = registerExSniffs();
            ALOGI("register amlogic extrator sniffers %s\n", ret == 0 ? "OK" : "Failed");
        }
    }
    return 0;
}


sp<MediaExtractor> createAmExExtractor(
    const sp<DataSource> &source, const char* mime, const sp<AMessage> &meta)
{
    typedef MediaExtractor * (*CreateExExtractor)(
        const sp<DataSource> &source, const char * mime, const sp<AMessage> &meta);
    static CreateExExtractor creator = NULL;

    loadAmlogicExtratorAdapters();

    if (gExExtratorHandle && !creator) {
        creator = (CreateExExtractor)dlsym(
                      gExExtratorHandle, "am_createAmExExtractor");
        ALOGI("Load createAmExExtractor %s\n", creator == NULL ? "Failed" : "OK");
    }
    if (creator) {
        sp<MediaExtractor> AmExtractor =
            (*creator)(source, mime, meta);
        if (AmExtractor != NULL) {
            return AmExtractor;
        }
    }
    return NULL;

}

}  // android
