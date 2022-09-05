from pixelinkWrapper import*
import os
import time
import msvcrt
import glob
import grequests
import requests
import concurrent.futures
import numpy as np
from telnetlib import Telnet

t_initial = time.time()
print(f'Pixelink-{t_initial} s.......')

# A few useful constants.
A_OK = 0            # non-zero error codes
GENERAL_ERROR = 1

# Default global recording parameters
DEFAULT_PLAYBACK_FRAME_RATE = 25    # in frames/second. 
DEFAULT_RECORD_DURATION = 20    # in seconds
CLIP_PLAYBACK_BITRATE = int(PxLApi.ClipPlaybackDefaults.BITRATE_DEFAULT/3)

# global recording parameters
recordTime = 3600 * 24        # seconds
bitRate = 5000000       # bits/second
frameRate = 25          # frames/second
fileName = 'video_squiggleball'

numImagesStreamed = 0
captureRc = PxLApi.ReturnCode.ApiSuccess
captureFinished = False

"""
Function that's called when PxLApi.getEncodedClip is finished capturing frames, or can't continue
capturing frames.
"""
@PxLApi._terminationFunction
def term_fn_get_encoded_clip(hCamera, numberOfFrameBlocksStreamed, retCode):
    # Just record the capture information into our shared (global) varaibles so the main line
    # can report/take action on the result.
    global numImagesStreamed
    global captureRc
    global captureFinished
    numImagesStreamed = numberOfFrameBlocksStreamed
    captureRc = retCode
    captureFinished = True
    print(f'No. of images streamed : {numImagesStreamed}')
    print(f'Time passed : {time.perf_counter()-t} s')
    return PxLApi.ReturnCode.ApiSuccess


# Create a folder for a clip if it does not exist
Target_folder_img = 'E:\\Soumen\\Squiggle Balls\\Images FInal\\'
if not os.path.exists(Target_folder_img):
        os.makedirs(Target_folder_img)

# Start capturing the required images into the clip
h264File = Target_folder_img + fileName + ".h264"

clipInfo = PxLApi.ClipEncodingInfo()
clipInfo.uStreamEncoding = PxLApi.ClipEncodingFormat.H264
clipInfo.uDecimationFactor = 1 # No decimation
clipInfo.playbackFrameRate = frameRate
clipInfo.playbackBitRate = bitRate

ret = PxLApi.initialize(0)
if not PxLApi.apiSuccess(ret[0]):
    print("Error: Unable to initialize a camera! rc = %i" % ret[0])
    # return GENERAL_ERROR

hCamera = ret[1]

# Determine the effective frame rate for the camera, and the number of images we will need to
# capture the video of the requested length then start the stream
cameraFps = frameRate           # effective_frame_rate(hCamera)
numImages = int(recordTime * cameraFps)

time.sleep(5 - (time.time() - t_initial))
ret = PxLApi.setStreamState(hCamera, PxLApi.StreamState.START)
if not PxLApi.apiSuccess(ret[0]):
    print(" Error: Could not start the stream.")
    PxLApi.uninitialize(hCamera)
    # return GENERAL_ERROR


print(" Recording %i seconds of h264 compressed video (based on %i images). Press any key to abort...\n"
      % (recordTime, numImages))
print(f'Pixelink-{time.time()} s.......')
t = time.perf_counter()

ret = PxLApi.getEncodedClip(hCamera, numImages, h264File, clipInfo, term_fn_get_encoded_clip)
if PxLApi.apiSuccess(ret[0]):
    # try:
    # global captureFinished
    while not captureFinished:
        try:
            if msvcrt.kbhit() or os.path.exists('E:\\Soumen\\Squiggle Balls\\Python codes\\finished.txt'):
            #     # User wants to abort. Tell the API to abort the capture by stopping the stream. This should call our callback with
            #     # an error.
                PxLApi.setStreamState(hCamera, PxLApi.StreamState.STOP)
            else:
            # No need to steal a bunch of cpu cycles on a loop doing nothing -- sleep for a bit until it's time to check for keyboard
            # input again.
                time.sleep(0.5)
        except KeyboardInterrupt:
            captureFinished = not captureFinished
            print('KeyboardInterrupt')
     
PxLApi.setStreamState(hCamera, PxLApi.StreamState.STOP) # already stopped if user aborted, but that's OK


if not PxLApi.apiSuccess(ret[0]):
    print("Error\n PxLApi.getEncodedClip/PxLApi.formatClip returned %i" % ret[0])

PxLApi.uninitialize(hCamera)

print(f'Total time taken : {time.perf_counter()-t} s')   
    # return ret[0]
