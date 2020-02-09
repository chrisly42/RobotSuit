#include <Adafruit_ZeroDMA.h>
#include "AudioZeroDMA.h"

int AudioZeroDMA::begin()
{
    dacConfigure();

    configurePlayerTimer();
    return configureDMA();
}

void AudioZeroDMA::end()
{
    disablePlayerTimer();
    resetPlayerTimer();
    analogWrite(A0, 0);
    dma.free();
}

void AudioZeroDMA::stop()
{
    disablePlayerTimer();
    if (audioFileReady) {
        audioFileReady = false;
        audioFile.close();
    }
    audioPlaying = false;
}

boolean AudioZeroDMA::play(uint32_t sampleRate, File file)
{
    stop();

    audioFile = file;
    currentDmaBufferSelect = 0;
    nextReadBufferSelect = 0;
    bufferLoaded[0] = false;
    bufferLoaded[1] = false;

    audioPlaying = true;
    audioFileReady = true;
    poll();

    audioPlaying = enablePlayerTimer(sampleRate);
}

boolean AudioZeroDMA::isPlaying()
{
    return audioPlaying;
}

void AudioZeroDMA::dacConfigure()
{
    analogWriteResolution(10);
    analogWrite(A0, 0);

    DAC->CTRLA.bit.ENABLE = 0x01;
    DAC->CTRLB.bit.LEFTADJ = 1;
#if AUDIO_ZERO_DMA_16BIT_SAMPLES
    DAC->DATA.reg = 1 << 15;
#else
    DAC->DATA.reg = 1<<7;
#endif
    while (DAC->STATUS.bit.SYNCBUSY);
}

void AudioZeroDMA::dmaBlockDoneCallbackWrapper(Adafruit_ZeroDMA *returnedDma)
{
    // if you get a compile error here, add
    // void                       *userData;
    // to Adafruit_ZeroDMA.h in the public section of the class Adafruit_ZeroDMA
    ((AudioZeroDMA *) (returnedDma->userData))->dmaBlockDoneCallback(returnedDma);
}

void AudioZeroDMA::dmaBlockDoneCallback(Adafruit_ZeroDMA *returnedDma)
{
    bufferLoaded[currentDmaBufferSelect] = false;
    currentDmaBufferSelect = 1 - currentDmaBufferSelect;
    if (!audioFileReady && !bufferLoaded[currentDmaBufferSelect]) {
        audioPlaying = false;
        disablePlayerTimer();
#if AUDIO_ZERO_DMA_16BIT_SAMPLES
        DAC->DATA.reg = 1 << 15;
#else
        DAC->DATA.reg = 1<<7;
#endif
        while (DAC->STATUS.bit.SYNCBUSY);
    }
}

void AudioZeroDMA::configurePlayerTimer()
{
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5);
    while (GCLK->STATUS.bit.SYNCBUSY);

    resetPlayerTimer();

    AUDIO_ZERO_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
    AUDIO_ZERO_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
    AUDIO_ZERO_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
    syncPlayerTimer();
}

int AudioZeroDMA::configureDMA()
{
    dma.setTrigger(AUDIO_ZERO_DMA_TRIGGER);
    dma.setAction(DMA_TRIGGER_ACTON_BEAT);
    if (dma.allocate() == DMA_STATUS_OK) {
        DmacDescriptor *dmaDesc = dma.addDescriptor(
                sampleBuf[0],
#if AUDIO_ZERO_DMA_16BIT_SAMPLES
                (void *) &DAC->DATA.reg,
                AUDIO_BUFFER_SIZE >> 1,
                DMA_BEAT_SIZE_HWORD, // 2 bytes per sample
#else
        (void *) ((uint32_t) &DAC->DATA.reg + 1),
        AUDIO_BUFFER_SIZE,
        DMA_BEAT_SIZE_BYTE, // 1 byte per sample
#endif
                true, // increment source
                false); // do not increment target
        if (dmaDesc) {
            dmaDesc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;
            dmaDesc = dma.addDescriptor(
                    sampleBuf[1],
#if AUDIO_ZERO_DMA_16BIT_SAMPLES
                    (void *) &DAC->DATA.reg,
                    AUDIO_BUFFER_SIZE >> 1,
                    DMA_BEAT_SIZE_HWORD, // 2 bytes per sample
#else
            (void *) ((uint32_t) &DAC->DATA.reg + 1),
            AUDIO_BUFFER_SIZE,
            DMA_BEAT_SIZE_BYTE, // 1 byte per sample
#endif
                    true, // increment source
                    false); // do not increment target
            if (dmaDesc) {
                dmaDesc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;
                dma.loop(true);
                dma.userData = this;
                dma.setCallback(this->dmaBlockDoneCallbackWrapper);
                return 0;
            }
        }
        dma.free();
    }
    return 1;
}

void AudioZeroDMA::syncPlayerTimer()
{
    while (AUDIO_ZERO_TIMER->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
}

boolean AudioZeroDMA::enablePlayerTimer(uint32_t sampleRate)
{
    AUDIO_ZERO_TIMER->COUNT16.CC[0].reg = (uint16_t)(SystemCoreClock / sampleRate - 1);
    AUDIO_ZERO_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    syncPlayerTimer();

    return (dma.startJob() == DMA_STATUS_OK);
}

void AudioZeroDMA::resetPlayerTimer()
{
    AUDIO_ZERO_TIMER->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    syncPlayerTimer();
    while (AUDIO_ZERO_TIMER->COUNT16.CTRLA.bit.SWRST);
}

void AudioZeroDMA::disablePlayerTimer()
{
    dma.abort();
    AUDIO_ZERO_TIMER->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    syncPlayerTimer();
}

// poll needs to be called at regular intervals
void AudioZeroDMA::poll()
{
    if (audioPlaying) {
        while (!bufferLoaded[nextReadBufferSelect] && audioFile.available()) {
            int totalRead = 0;
            int bytesRead;
            do {
                // for some unknown reason, SDFat barfs on reading more than 1023 bytes at once. So chunk it.
                bytesRead = audioFile.read(sampleBuf[nextReadBufferSelect] + totalRead, 512);
                totalRead += bytesRead;
            } while ((bytesRead == 512) && (totalRead < AUDIO_BUFFER_SIZE));
            if ((totalRead < AUDIO_BUFFER_SIZE) || !audioFile.available()) {
                audioFileReady = false;
                audioFile.close();
                for (; totalRead < AUDIO_BUFFER_SIZE; totalRead++) {
#if AUDIO_ZERO_DMA_16BIT_SAMPLES
                    sampleBuf[nextReadBufferSelect][totalRead] = 1 << 7; // same value due to little endian
#else
                    sampleBuf[nextReadBufferSelect][totalRead] = 1<<7;
#endif
                }
            }
            if (totalRead > 0) {
                bufferLoaded[nextReadBufferSelect] = true;
                nextReadBufferSelect = 1 - nextReadBufferSelect;
            }
        }
    }
}
