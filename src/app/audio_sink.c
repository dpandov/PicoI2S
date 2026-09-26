#include "app/audio_sink.h"

#include "pico/time.h"
#include "usb/usb_audio.h"

static struct {
    bool     running;
    uint64_t last_us;
    uint64_t acc;             // sample rate x microseconds, remainder < 1e6
    uint32_t underruns;
} sink;

static uint8_t sink_buf[1024];

static void sink_reset(void) {
    sink.running = false;
    sink.acc = 0;
}

void audio_sink_task(void) {
    if (usb_audio_link_state() != USB_AUDIO_STREAMING) {
        sink_reset();
        return;
    }

    usb_audio_format_t fmt = usb_audio_format();
    usb_audio_stats_t st;
    usb_audio_get_stats(&st);
    uint32_t frame_bytes = (uint32_t)fmt.subslot_bytes * 2u;
    uint64_t now = time_us_64();

    if (!sink.running) {
        if (st.fifo_level < st.fifo_depth / 2u) {
            return;
        }
        sink.running = true;
        sink.last_us = now;
        sink.acc = 0;
        return;
    }

    // Carry the remainder so 44.1 kHz-family rates average out exactly.
    sink.acc += (uint64_t)fmt.sample_rate * (now - sink.last_us);
    sink.last_us = now;
    uint32_t frames = (uint32_t)(sink.acc / 1000000u);
    sink.acc -= (uint64_t)frames * 1000000u;

    uint32_t want = frames * frame_bytes;
    while (want > 0) {
        uint32_t chunk = want < sizeof(sink_buf) ? want : sizeof(sink_buf);
        chunk -= chunk % frame_bytes;
        uint32_t got = usb_audio_read(sink_buf, chunk);
        want -= got;
        if (got < chunk) {
            sink.underruns++;
            sink_reset();
            break;
        }
    }
}

bool audio_sink_running(void) {
    return sink.running;
}

uint32_t audio_sink_underruns(void) {
    return sink.underruns;
}
