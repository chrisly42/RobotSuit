#pragma once

#include <SdFat.h>
#include <Adafruit_ZeroDMA.h>
#include <SPI.h>

#define AUDIO_ZERO_TIMER TC5 // currently only possible to choose between TC4 and TC5
#define AUDIO_ZERO_DMA_TRIGGER TC5_DMAC_ID_OVF // corresponding trigger

// Single buffer file size big enough to hold enough samples between calls to poll()
#define AUDIO_BUFFER_SIZE (3*1024)

// Set to 1 if you want to use 16 bit samples (though you will only get 10 bit resolution from the DAC)
#define AUDIO_ZERO_DMA_16BIT_SAMPLES 1

// use these settings for sox
// - for 16 bit: sox <filename> -t raw -c 1 -b 16 -L -r <samplerate> -e unsigned-integer <targetfile>
// - for 8 bit: sox <filename> -t raw -c 1 -b 8 -r <samplerate> -e unsigned-integer <targetfile>

class AudioZeroDMA
{

    Adafruit_ZeroDMA dma;

    File audioFile;

    uint8_t sampleBuf[2][AUDIO_BUFFER_SIZE];

    volatile uint8_t currentDmaBufferSelect = 0;
    uint8_t nextReadBufferSelect = 0;
    volatile boolean bufferLoaded[2] = {false, false};
    boolean audioPlaying = false;
    boolean audioFileReady = false;

public:

    AudioZeroDMA() {};

    int begin();

    boolean play(uint32_t sampleRate, File file);

    void stop();

    void end();

    void poll();

    boolean isPlaying();

private:
    void dacConfigure();

    int configureDMA();

    static void dmaBlockDoneCallbackWrapper(Adafruit_ZeroDMA *returnedDma);

    void dmaBlockDoneCallback(Adafruit_ZeroDMA *returnedDma);

    void configurePlayerTimer();

    void syncPlayerTimer();

    void resetPlayerTimer();

    boolean enablePlayerTimer(uint32_t sampleRate);

    void disablePlayerTimer();
};
