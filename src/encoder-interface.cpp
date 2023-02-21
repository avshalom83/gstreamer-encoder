#include "encoder-interface.h"

#include <condition_variable>
#include <cstring>
#include <vector>
#include <utility>
#include <atomic>
#include <gst/gst.h>
#include <gst/rtp/gstrtpbuffer.h>
#include <gst/app/app.h>



#include <mutex>
#include <memory>
#include <assert.h>
#include <stdio.h>
#include <iostream>

uint64_t duEncoderLibraryVersion() {
    std::cout << "XYZ duEncoderLibraryVersion: : " << DU_CURRENT_ENCODER_VERSION << std::endl;
    return DU_CURRENT_ENCODER_VERSION;
}


static constexpr guint INVALID_BUS_WATCH_ID = 0;
std::atomic<bool>       _isRunning;
std::atomic<bool>       _EncodedFrameIsReady;

std::condition_variable cv_;
std::mutex _mutexEncodedFrameIsReady;

typedef unsigned int    guint;
guint                   _busWatchId             = INVALID_BUS_WATCH_ID;

GstElement *_currentPipelineElement = NULL;
GstBuffer *buffer = NULL;
GstFlowReturn ret;
int test_counter = 0;
char* encodedVideoBuffer;
int encodedVideoBufferSize;
bool                    _loop                   = false;
std::atomic<bool>       _restartPipeline        { false };
int MANDATORY_BASE_IN_ORDER_TO_PATH_WRAPAROUND = 2;
int _bufferNumberOfElement = MANDATORY_BASE_IN_ORDER_TO_PATH_WRAPAROUND*4;
std::vector< std::tuple < uint64_t, EncoderApi::PointerLengthReleaser > >  _encodedVideoBuffers(_bufferNumberOfElement);
std::atomic<int>       _readBufferIntex { 0 };
std::atomic<int>       _writeBufferIntex { 0 };
static GstClockTime timestamp = 0;
char* encodedVideoBuffer1;
char* filebuffer;


void releseFileBuffer( char* filebuffer) 
{
    free ( filebuffer);
}
void UnlockBitstream( ) 
{
}

gboolean busCallback( GstBus *bus, GstMessage *msg, gpointer unusedUserData) {
    GError *err = NULL;
    gchar *debug = NULL;

    std::cout << "Bus callback called with message of type: " << GST_MESSAGE_TYPE_NAME(msg) << std::endl;

    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS:
            g_print("got EOS\n");
            break;
        case GST_MESSAGE_WARNING:
            gst_message_parse_warning(msg, &err, &debug);
            g_print("[WARNING] %s\n%s\n", err->message, debug);
            break;
        case GST_MESSAGE_ERROR:
            gst_message_parse_error(msg, &err, &debug);
            g_print("[ERROR] %s\n%s\n", err->message, debug);
        default:
            std::cout << "Bus callback called with unhandled message type: " << GST_MESSAGE_TYPE_NAME(msg) << std::endl;
            break;
    }
    return TRUE;
}
void addBusWatch() {
    assert(_currentPipelineElement != nullptr);
    assert(_busWatchId == INVALID_BUS_WATCH_ID);

    GstBus *bus = gst_element_get_bus( _currentPipelineElement );
    constexpr auto UNUSED_USER_DATA = nullptr;
    _busWatchId = gst_bus_add_watch(bus, busCallback, UNUSED_USER_DATA);
    assert(_busWatchId != INVALID_BUS_WATCH_ID);
    gst_object_unref( bus );
}

void startPipeline() {
     if ( !_isRunning && _currentPipelineElement != NULL && _currentPipelineElement != NULL ) {
         gst_element_set_state( _currentPipelineElement, GST_STATE_PLAYING );
         _isRunning = true;
     }
}
 
int INIT_VALUE = -1;
GstFlowReturn onNewSample( GstElement *element, gpointer internal_buffer ) { 
    
    int writeBuferIndex = _writeBufferIntex++;
    writeBuferIndex = writeBuferIndex % _bufferNumberOfElement;
    assert( _readBufferIntex.load() != _writeBufferIntex.load() );

    GstBuffer *myBuffer ;
    GstAppSink *appsink = (GstAppSink *)element;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (sample == NULL) return GST_FLOW_ERROR;

    myBuffer = gst_sample_get_buffer(sample);
    
    GstMapInfo map_info;

    if (!gst_buffer_map ((myBuffer), &map_info, GST_MAP_READ)) {
        gst_buffer_unmap ((myBuffer), &map_info);
        gst_sample_unref(sample);
    }
    std::memcpy( std::get<1>(_encodedVideoBuffers[writeBuferIndex]).buffer, map_info.data, map_info.size); 
    std::get<1>(_encodedVideoBuffers[writeBuferIndex]).length = map_info.size;
    std::unique_lock<std::mutex> lk( _mutexEncodedFrameIsReady );
    cv_.wait(lk, []{return ( true );});
    lk.unlock();
    
    if ( writeBuferIndex == _readBufferIntex.load()){
       _EncodedFrameIsReady.store(true); 
        cv_.notify_all();
    } 
    return GST_FLOW_OK;
}

bool duInitEncoder(CUcontext cudaContext,
                unsigned width,
                unsigned height,
                EncoderApi::FrameRateFraction frameRate,
                EncoderApi::Codec codec,
                unsigned numberOfSlices = ENCODER_NO_SLICES)
{
    assert(_currentPipelineElement == nullptr);
    GError *e = NULL;

    filebuffer = (char*)malloc ((3 * width * height / 2));
    
    for (auto & _encodedVideoBuffer1: _encodedVideoBuffers){
        char* encodedVideoBuffer1 = (char*)malloc ( (3 * width * height / 2) );
        EncoderApi::PointerLengthReleaser retPointerLength;
        retPointerLength.buffer = encodedVideoBuffer1;
        retPointerLength.length = 0;
        retPointerLength.releaser =  []() { UnlockBitstream(); };
        std::get<1>(_encodedVideoBuffer1) = std::move(retPointerLength);

    }
    _EncodedFrameIsReady.store(false);
   
    const std::string pipe = "appsrc name=appsrc ! video/x-raw, format=I420, width=1920, height=1080, framerate=30/1 ! videoconvert ! x264enc key-int-max=120 speed-preset=ultrafast bitrate=2048 ! appsink name=appsink emit-signals=true sync=false";
    _currentPipelineElement = gst_parse_launch( pipe.c_str(), &e );
    if ( e != NULL || _currentPipelineElement == NULL ) {
        std::cerr << "[ERROR] Failed to run pipeline: \n ~~~ " << pipe << "\n"
                  << "[Error]: " << e->message << std::endl;
        throw std::runtime_error(e->message);
    }

    assert(_currentPipelineElement != nullptr);

    addBusWatch();
    GstElement *appsink = gst_bin_get_by_name( GST_BIN(_currentPipelineElement), "appsink" );
    g_signal_connect( appsink, "new_sample", G_CALLBACK(onNewSample), &buffer );

    startPipeline();

    try {
    } catch( std::exception &ex ) {
        std::cerr<<"NVIDIA EncoderFromCuda initialization failed: "<<ex.what()<<"\n";
        return false;
    }
    return true;
}

void duSetBitrateBytesPerFrame(unsigned bytesPerFrame) {
    return;
    // std::cout<<"duSetBitrateBytesPerFrame: "<< bytesPerFrame << std::endl;
}

void duSetSize( unsigned width, unsigned height) {
     std::cout<<"duSetSize: "<< width << ", " << height << std::endl;
}
bool duInvalidateFrame(unsigned long long frameIndex) {
    std::cout<<"duInvalidateFrame: "<< frameIndex << std::endl;
    return true;
}
void duExpireOlderFrames(uint64_t timestamp) {
     std::cout<<"duExpireOlderFrames: "<< timestamp << std::endl;
}

void duPutFrame(EncoderApi::DriveUVideoDeviceBuffer *frame,
                uint64_t timestamp) {
    GstElement *appsrc = gst_bin_get_by_name( GST_BIN(_currentPipelineElement), "appsrc" );

	if (filebuffer == NULL) {printf("Memory error\n"); exit (2);} //Errorcheck
    
    EncoderApi::PointerLengthReleaser retPointerLength;
    retPointerLength.buffer = filebuffer;
    retPointerLength.length = frame->size();
    retPointerLength.releaser = []() { };

    frame->encoderApiMemcpyToHost(retPointerLength );
    GstBuffer *pushbuffer;
    pushbuffer = gst_buffer_new_wrapped (filebuffer, frame->size());

	//Set frame timestamp
	GST_BUFFER_PTS      (pushbuffer) = timestamp;
	GST_BUFFER_DTS      (pushbuffer) = timestamp;	
	GST_BUFFER_DURATION (pushbuffer) = gst_util_uint64_scale_int (1, GST_SECOND, 1);

    g_signal_emit_by_name (appsrc, "push-buffer", pushbuffer, &ret);
    
    }



void duEncoderShutdown() {}

EncoderApi::PointerLengthReleaser duEncode(uint64_t                           timestamp,
                                bool                              forceIDR,
                                EncoderApi::EncodedFrameMetadata              & metadata,
                                bool                              advanceFrameIndex) {


    if ( _readBufferIntex.load() == _writeBufferIntex.load() ){
        _EncodedFrameIsReady.store(false);
    }
    std::unique_lock<std::mutex> lk( _mutexEncodedFrameIsReady );
    cv_.wait(lk, []{return ( _EncodedFrameIsReady.load() );});
    lk.unlock();
    int readBufferIntex = _readBufferIntex.load();
    _readBufferIntex.store( ( readBufferIntex + 1 ) % _bufferNumberOfElement );
    
    static int frameIndex = 0;
    metadata.frameIndex = frameIndex++;
    // if (frameIndex < 10 )
    //     metadata.frameType = EncoderApi::FrameType::SPSPPSIDR;
    // else
    metadata.frameType = EncoderApi::FrameType::IDR;
    metadata.timestamp = timestamp;

    return std::get<1>(_encodedVideoBuffers[readBufferIntex]);

}
