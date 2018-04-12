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

#ifndef ANDROID_METADATARETRIEVERFACTORY_H
#define ANDROID_METADATARETRIEVERFACTORY_H

#include <media/MediaMetadataRetrieverInterface.h>
#include <media/MediaPlayerInterface.h>
#include <media/stagefright/foundation/ABase.h>

namespace android {

class MediaMetadataRetrieverFactory {
  public:
    class RFactory {
      public:
        virtual ~RFactory() { }
        virtual sp<MediaMetadataRetrieverBase> createRetriever() = 0;
    };

    static status_t registerFactory(RFactory* factory,
                                    player_type type);
    static void unregisterFactory(player_type type);

    static sp<MediaMetadataRetrieverBase> createRetriever(player_type playerType);

  private:
    typedef KeyedVector<player_type, RFactory*> tFactoryMap;

    MediaMetadataRetrieverFactory() { }

    static status_t registerFactory_l(RFactory* factory,
                                      player_type type);

    static Mutex       sLock;
    static tFactoryMap sFactoryMap;
    static bool        sInitComplete;

    DISALLOW_EVIL_CONSTRUCTORS(MediaMetadataRetrieverFactory);
};

}  // namespace android
#endif  // ANDROID_METADATARETRIEVERFACTORY_H

