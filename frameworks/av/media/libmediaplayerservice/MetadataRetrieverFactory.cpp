/*
**
** Copyright 2014, The Android Open Source Project
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#define LOG_TAG "MetadataRetrieverFactory"
#include <utils/Log.h>

#include <cutils/properties.h>
#include <media/IMediaPlayer.h>
#include <media/stagefright/foundation/ADebug.h>
#include <media/MediaMetadataRetrieverInterface.h>
#include <media/MediaPlayerInterface.h>
#include <utils/Errors.h>
#include <utils/misc.h>

#include "MetadataRetrieverFactory.h"

namespace android {

Mutex MediaMetadataRetrieverFactory::sLock;
MediaMetadataRetrieverFactory::tFactoryMap MediaMetadataRetrieverFactory::sFactoryMap;
bool MediaMetadataRetrieverFactory::sInitComplete = false;

status_t MediaMetadataRetrieverFactory::registerFactory_l(RFactory* factory,
                                               player_type type) {
    if (NULL == factory) {
        ALOGE("Failed to register MediaMetadataRetrieverFactory of type %d, factory is"
              " NULL.", type);
        return BAD_VALUE;
    }

    if (sFactoryMap.indexOfKey(type) >= 0) {
        ALOGE("Failed to register MediaMetadataRetrieverFactory of type %d, type is"
              " already registered.", type);
        return ALREADY_EXISTS;
    }

    if (sFactoryMap.add(type, factory) < 0) {
        ALOGE("Failed to register MediaMetadataRetrieverFactory of type %d, failed to add"
              " to map.", type);
        return UNKNOWN_ERROR;
    }

    return OK;
}

status_t MediaMetadataRetrieverFactory::registerFactory(RFactory* factory,
                                             player_type type) {
    Mutex::Autolock lock_(&sLock);
    return registerFactory_l(factory, type);
}

void MediaMetadataRetrieverFactory::unregisterFactory(player_type type) {
    Mutex::Autolock lock_(&sLock);
    sFactoryMap.removeItem(type);
}

sp<MediaMetadataRetrieverBase> MediaMetadataRetrieverFactory::createRetriever(player_type playerType) {
    sp<MediaMetadataRetrieverBase> p;
    RFactory* factory;
    //status_t init_result;
    Mutex::Autolock lock_(&sLock);

    if (sFactoryMap.indexOfKey(playerType) < 0) {
        ALOGE("Failed to create player object of type %d, no registered"
              " factory", playerType);
        return p;
    }

    factory = sFactoryMap.valueFor(playerType);
    CHECK(NULL != factory);
    p = factory->createRetriever();

    if (p == NULL) {
        ALOGE("Failed to create player object of type %d, create failed",
               playerType);
        return p;
    }

    return p;
}

}
