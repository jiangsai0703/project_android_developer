/*
 * Copyright (C) 2007 The Android Open Source Project
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

#ifndef ANDROID_MEDIAPLAYER_H
#define ANDROID_MEDIAPLAYER_H

#include <arpa/inet.h>

#include <binder/IMemory.h>

#include <media/AudioResamplerPublic.h>
#include <media/IMediaPlayerClient.h>
#include <media/IMediaPlayer.h>
#include <media/IMediaDeathNotifier.h>
#include <media/IStreamSource.h>

#include <utils/KeyedVector.h>
#include <utils/String8.h>

class ANativeWindow;

namespace android {

struct AVSyncSettings;
class IGraphicBufferProducer;
class Surface;

enum media_event_type {
    MEDIA_NOP               = 0, // interface test message
    MEDIA_PREPARED          = 1,
    MEDIA_PLAYBACK_COMPLETE = 2,
    MEDIA_BUFFERING_UPDATE  = 3,
    MEDIA_SEEK_COMPLETE     = 4,
    MEDIA_SET_VIDEO_SIZE    = 5,
    MEDIA_STARTED           = 6,
    MEDIA_PAUSED            = 7,
    MEDIA_STOPPED           = 8,
    MEDIA_SKIPPED           = 9,
    MEDIA_TIMED_TEXT        = 99,
    MEDIA_ERROR             = 100,
    MEDIA_INFO              = 200,
    MEDIA_SUBTITLE_DATA     = 201,
    MEDIA_META_DATA         = 202,
    MEDIA_BLURAY_INFO       = 203,
};

// Generic error codes for the media player framework.  Errors are fatal, the
// playback must abort.
//
// Errors are communicated back to the client using the
// MediaPlayerListener::notify method defined below.
// In this situation, 'notify' is invoked with the following:
//   'msg' is set to MEDIA_ERROR.
//   'ext1' should be a value from the enum media_error_type.
//   'ext2' contains an implementation dependant error code to provide
//          more details. Should default to 0 when not used.
//
// The codes are distributed as follow:
//   0xx: Reserved
//   1xx: Android Player errors. Something went wrong inside the MediaPlayer.
//   2xx: Media errors (e.g Codec not supported). There is a problem with the
//        media itself.
//   3xx: Runtime errors. Some extraordinary condition arose making the playback
//        impossible.
//
enum media_error_type {
    // 0xx
    MEDIA_ERROR_UNKNOWN = 1,
    // 1xx
    MEDIA_ERROR_SERVER_DIED = 100,
    // 2xx
    MEDIA_ERROR_NOT_VALID_FOR_PROGRESSIVE_PLAYBACK = 200,
    // 3xx
};


// Info and warning codes for the media player framework.  These are non fatal,
// the playback is going on but there might be some user visible issues.
//
// Info and warning messages are communicated back to the client using the
// MediaPlayerListener::notify method defined below.  In this situation,
// 'notify' is invoked with the following:
//   'msg' is set to MEDIA_INFO.
//   'ext1' should be a value from the enum media_info_type.
//   'ext2' contains an implementation dependant info code to provide
//          more details. Should default to 0 when not used.
//
// The codes are distributed as follow:
//   0xx: Reserved
//   7xx: Android Player info/warning (e.g player lagging behind.)
//   8xx: Media info/warning (e.g media badly interleaved.)
//
enum media_info_type {
    // 0xx
    MEDIA_INFO_UNKNOWN = 1,
    // The player was started because it was used as the next player for another
    // player, which just completed playback
    MEDIA_INFO_STARTED_AS_NEXT = 2,
    // The player just pushed the very first video frame for rendering
    MEDIA_INFO_RENDERING_START = 3,
    // 7xx
    // The video is too complex for the decoder: it can't decode frames fast
    // enough. Possibly only the audio plays fine at this stage.
    MEDIA_INFO_VIDEO_TRACK_LAGGING = 700,
    // MediaPlayer is temporarily pausing playback internally in order to
    // buffer more data.
    MEDIA_INFO_BUFFERING_START = 701,
    // MediaPlayer is resuming playback after filling buffers.
    MEDIA_INFO_BUFFERING_END = 702,
    // Bandwidth in recent past
    MEDIA_INFO_NETWORK_BANDWIDTH = 703,

    // 8xx
    // Bad interleaving means that a media has been improperly interleaved or not
    // interleaved at all, e.g has all the video samples first then all the audio
    // ones. Video is playing but a lot of disk seek may be happening.
    MEDIA_INFO_BAD_INTERLEAVING = 800,
    // The media is not seekable (e.g live stream).
    MEDIA_INFO_NOT_SEEKABLE = 801,
    // New media metadata is available.
    MEDIA_INFO_METADATA_UPDATE = 802,

    //9xx
    MEDIA_INFO_TIMED_TEXT_ERROR = 900,

    //amlogic extend warning message type,just for notify,never force to exit player.
    MEDIA_INFO_AMLOGIC_BASE = 8000,
    MEDIA_INFO_AMLOGIC_VIDEO_NOT_SUPPORT=MEDIA_INFO_AMLOGIC_BASE+1,
    MEDIA_INFO_AMLOGIC_AUDIO_NOT_SUPPORT = MEDIA_INFO_AMLOGIC_BASE+2,
    MEDIA_INFO_AMLOGIC_NO_VIDEO = MEDIA_INFO_AMLOGIC_BASE+3,
    MEDIA_INFO_AMLOGIC_NO_AUDIO = MEDIA_INFO_AMLOGIC_BASE+4,
    MEDIA_INFO_AMLOGIC_SHOW_DTS_ASSET = MEDIA_INFO_AMLOGIC_BASE+5,
    MEDIA_INFO_AMLOGIC_SHOW_DTS_EXPRESS = MEDIA_INFO_AMLOGIC_BASE+6,
    MEDIA_INFO_AMLOGIC_SHOW_DTS_HD_MASTER_AUDIO = MEDIA_INFO_AMLOGIC_BASE+7,
    MEDIA_INFO_AMLOGIC_SHOW_AUDIO_LIMITED = MEDIA_INFO_AMLOGIC_BASE+8,
    MEDIA_INFO_AMLOGIC_SHOW_DTS_MULASSETHINT=MEDIA_INFO_AMLOGIC_BASE+9,
    MEDIA_INFO_AMLOGIC_SHOW_DTS_HPS_NOTSUPPORT=MEDIA_INFO_AMLOGIC_BASE+10,
    MEDIA_INFO_AMLOGIC_BLURAY_STREAM_PATH = MEDIA_INFO_AMLOGIC_BASE+11,

    //notify java app the download bitrate
    MEDIA_INFO_DOWNLOAD_BITRATE = 0x9001,
    //notify java app the buffering circle percent
    MEDIA_INFO_BUFFERING_PERCENT = 0x9002,
};



enum media_player_states {
    MEDIA_PLAYER_STATE_ERROR        = 0,
    MEDIA_PLAYER_IDLE               = 1 << 0,
    MEDIA_PLAYER_INITIALIZED        = 1 << 1,
    MEDIA_PLAYER_PREPARING          = 1 << 2,
    MEDIA_PLAYER_PREPARED           = 1 << 3,
    MEDIA_PLAYER_STARTED            = 1 << 4,
    MEDIA_PLAYER_PAUSED             = 1 << 5,
    MEDIA_PLAYER_STOPPED            = 1 << 6,
    MEDIA_PLAYER_PLAYBACK_COMPLETE  = 1 << 7
};

// Keep KEY_PARAMETER_* in sync with MediaPlayer.java.
// The same enum space is used for both set and get, in case there are future keys that
// can be both set and get.  But as of now, all parameters are either set only or get only.
enum media_parameter_keys {
    // Streaming/buffering parameters
    KEY_PARAMETER_CACHE_STAT_COLLECT_FREQ_MS = 1100,            // set only

    // Return a Parcel containing a single int, which is the channel count of the
    // audio track, or zero for error (e.g. no audio track) or unknown.
    KEY_PARAMETER_AUDIO_CHANNEL_COUNT = 1200,                   // get only

    // Playback rate expressed in permille (1000 is normal speed), saved as int32_t, with negative
    // values used for rewinding or reverse playback.
    KEY_PARAMETER_PLAYBACK_RATE_PERMILLE = 1300,                // set only

    // Set a Parcel containing the value of a parcelled Java AudioAttribute instance
    KEY_PARAMETER_AUDIO_ATTRIBUTES = 1400,                      // set only


    //AML Video INFO string,set only
    KEY_PARAMETER_AML_VIDEO_POSITION_INFO = 2000,

    //PLAYER TYPE STRING
    KEY_PARAMETER_AML_PLAYER_TYPE_STR = 2001,
    //PLAYER VIDEO  OUT/TYPE
    //public static final int VIDEO_OUT_SOFT_RENDER = 0;
    //public static final int VIDEO_OUT_HARDWARE    = 1;
    KEY_PARAMETER_AML_PLAYER_VIDEO_OUT_TYPE = 2002,

    //amlogic private API,set only.
    KEY_PARAMETER_AML_PLAYER_SWITCH_SOUND_TRACK     = 2003,     // string, refer to lmono,rmono,stereo, set only
    KEY_PARAMETER_AML_PLAYER_SWITCH_AUDIO_TRACK     = 2004,     // string, refer to audio track index, set only
    KEY_PARAMETER_AML_PLAYER_TRICKPLAY_FORWARD      = 2005,     // string, refer to forward:speed
    KEY_PARAMETER_AML_PLAYER_TRICKPLAY_BACKWARD     = 2006,     // string, refer to  backward:speed
    KEY_PARAMETER_AML_PLAYER_FORCE_HARD_DECODE      = 2007,     // string, refer to mp3,etc.
    KEY_PARAMETER_AML_PLAYER_FORCE_SOFT_DECODE      = 2008,     // string, refer to mp3,etc.
    KEY_PARAMETER_AML_PLAYER_GET_MEDIA_INFO         = 2009,     // string, get media info
    KEY_PARAMETER_AML_PLAYER_FORCE_SCREEN_MODE      = 2010,     // string, set screen mode
    KEY_PARAMETER_AML_PLAYER_SET_DISPLAY_MODE       = 2011,     // string, set display mode
    KEY_PARAMETER_AML_PLAYER_GET_DTS_ASSET_TOTAL    = 2012,     // string, get dts asset total number
    KEY_PARAMETER_AML_PLAYER_SET_DTS_ASSET          = 2013,     // string, set dts asset
    KEY_PARAMETER_AML_PLAYER_SWITCH_VIDEO_TRACK     = 2015,     //string,refer to video track index,set only
    KEY_PARAMETER_AML_PLAYER_SET_TRICKPLAY_MODE     = 2016,     //string,refer to video ff/fb flag,set only
    KEY_PARAMETER_AML_PLAYER_HWBUFFER_STATE         = 3001,     // string,refer to stream buffer info
    KEY_PARAMETER_AML_PLAYER_RESET_BUFFER           = 8000,     // top level seek..player need to reset & clearbuffers
    KEY_PARAMETER_AML_PLAYER_FREERUN_MODE           = 8002,     // play ASAP...
    KEY_PARAMETER_AML_PLAYER_ENABLE_OSDVIDEO        = 8003,     // play enable osd video for this player....
    KEY_PARAMETER_AML_PLAYER_DIS_AUTO_BUFFER        = 8004,     // play ASAP...
    KEY_PARAMETER_AML_PLAYER_ENA_AUTO_BUFFER        = 8005,     // play ASAP...
    KEY_PARAMETER_AML_PLAYER_USE_SOFT_DEMUX         = 8006,     // play use soft demux
    KEY_PARAMETER_AML_PLAYER_PR_CUSTOM_DATA         = 9001,     // string, playready, set only
};

enum video_out_type {
    VIDEO_OUT_SOFT_RENDER = 0,
    VIDEO_OUT_HARDWARE    = 1,
};

// Keep INVOKE_ID_* in sync with MediaPlayer.java.
enum media_player_invoke_ids {
    INVOKE_ID_GET_TRACK_INFO = 1,
    INVOKE_ID_ADD_EXTERNAL_SOURCE = 2,
    INVOKE_ID_ADD_EXTERNAL_SOURCE_FD = 3,
    INVOKE_ID_SELECT_TRACK = 4,
    INVOKE_ID_UNSELECT_TRACK = 5,
    INVOKE_ID_SET_VIDEO_SCALING_MODE = 6,
    INVOKE_ID_GET_SELECTED_TRACK = 7,
    INVOKE_ID_NETWORK_GET_LPBUF_BUFFERED_SIZE = 8,
    INVOKE_ID_NETWORK_GET_STREAMBUF_BUFFERED_SIZE = 9,
    INVOKE_ID_SET_TRACK_VOLUME = 10,
};

// Keep MEDIA_TRACK_TYPE_* in sync with MediaPlayer.java.
enum media_track_type {
    MEDIA_TRACK_TYPE_UNKNOWN = 0,
    MEDIA_TRACK_TYPE_VIDEO = 1,
    MEDIA_TRACK_TYPE_AUDIO = 2,
    MEDIA_TRACK_TYPE_TIMEDTEXT = 3,
    MEDIA_TRACK_TYPE_SUBTITLE = 4,
    MEDIA_TRACK_TYPE_METADATA = 5,
};

// ----------------------------------------------------------------------------
// ref-counted object for callbacks
class MediaPlayerListener: virtual public RefBase
{
public:
    virtual void notify(int msg, int ext1, int ext2, const Parcel *obj) = 0;
};

struct IMediaHTTPService;

class MediaPlayer : public BnMediaPlayerClient,
                    public virtual IMediaDeathNotifier
{
public:
    MediaPlayer();
    ~MediaPlayer();
            void            died();
            void            disconnect();

            status_t        setDataSource(
                    const sp<IMediaHTTPService> &httpService,
                    const char *url,
                    const KeyedVector<String8, String8> *headers);

            status_t        setDataSource(int fd, int64_t offset, int64_t length);
            status_t        setDataSource(const sp<IDataSource> &source);
            status_t        setVideoSurfaceTexture(
                                    const sp<IGraphicBufferProducer>& bufferProducer);
            status_t        setListener(const sp<MediaPlayerListener>& listener);
            status_t        prepare();
            status_t        prepareAsync();
            status_t        start();
            status_t        stop();
            status_t        pause();
            bool            isPlaying();
            status_t        setPlaybackSettings(const AudioPlaybackRate& rate);
            status_t        getPlaybackSettings(AudioPlaybackRate* rate /* nonnull */);
            status_t        setSyncSettings(const AVSyncSettings& sync, float videoFpsHint);
            status_t        getSyncSettings(
                                    AVSyncSettings* sync /* nonnull */,
                                    float* videoFps /* nonnull */);
            status_t        getVideoWidth(int *w);
            status_t        getVideoHeight(int *h);
            status_t        seekTo(int msec);
            status_t        getCurrentPosition(int *msec);
            status_t        getDuration(int *msec);
            status_t        reset();
            status_t        setAudioStreamType(audio_stream_type_t type);
            status_t        getAudioStreamType(audio_stream_type_t *type);
            status_t        setLooping(int loop);
            bool            isLooping();
            status_t        setVolume(float leftVolume, float rightVolume);
            void            notify(int msg, int ext1, int ext2, const Parcel *obj = NULL);
            status_t        invoke(const Parcel& request, Parcel *reply);
            status_t        setMetadataFilter(const Parcel& filter);
            status_t        getMetadata(bool update_only, bool apply_filter, Parcel *metadata);
            status_t        setAudioSessionId(audio_session_t sessionId);
            audio_session_t getAudioSessionId();
            status_t        setAuxEffectSendLevel(float level);
            status_t        attachAuxEffect(int effectId);
            status_t        setParameter(int key, const Parcel& request);
            status_t        getParameter(int key, Parcel* reply);
            status_t        setRetransmitEndpoint(const char* addrString, uint16_t port);
            status_t        setNextMediaPlayer(const sp<MediaPlayer>& player);

private:
            void            clear_l();
            status_t        seekTo_l(int msec);
            status_t        prepareAsync_l();
            status_t        getDuration_l(int *msec);
            status_t        attachNewPlayer(const sp<IMediaPlayer>& player);
            status_t        reset_l();
            status_t        doSetRetransmitEndpoint(const sp<IMediaPlayer>& player);
            status_t        checkStateForKeySet_l(int key);

    sp<IMediaPlayer>            mPlayer;
    thread_id_t                 mLockThreadId;
    Mutex                       mLock;
    Mutex                       mNotifyLock;
    Condition                   mSignal;
    sp<MediaPlayerListener>     mListener;
    void*                       mCookie;
    media_player_states         mCurrentState;
    int                         mCurrentPosition;
    int                         mSeekPosition;
    bool                        mPrepareSync;
    status_t                    mPrepareStatus;
    audio_stream_type_t         mStreamType;
    Parcel*                     mAudioAttributesParcel;
    bool                        mLoop;
    float                       mLeftVolume;
    float                       mRightVolume;
    int                         mVideoWidth;
    int                         mVideoHeight;
    audio_session_t             mAudioSessionId;
    float                       mSendLevel;
    struct sockaddr_in          mRetransmitEndpoint;
    bool                        mRetransmitEndpointValid;
};

}; // namespace android

#endif // ANDROID_MEDIAPLAYER_H
