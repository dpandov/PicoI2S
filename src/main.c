// usb2spdif - USB to S/PDIF bridge for the Raspberry Pi Pico 2.

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "tusb.h"
#include "app/audio_sink.h"
#include "app/monitor.h"
#include "app/status_led.h"
#include "diag/diag.h"
#include "usb/usb_audio.h"

// clk_peri (UART) runs from the fixed 48 MHz USB PLL so the console is not
// affected when the system PLL is retuned for a sample-rate family.
static void peri_clock_init(void) {
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);
}

int main(void) {
    diag_init();
    peri_clock_init();
    monitor_init();
    status_led_init();

    usb_audio_init();
    const tusb_rhport_init_t dev_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    tusb_init(0, &dev_init);

    while (true) {
        diag_loop();
        diag_mark(DIAG_MARK_TUD_TASK);
        tud_task();
        diag_mark(DIAG_MARK_SINK);
        audio_sink_task();

        uint32_t ev = usb_audio_take_events();
        bool stream_open = usb_audio_link_state() == USB_AUDIO_STREAMING;
        diag_mark(DIAG_MARK_LED);
        status_led_task(ev, stream_open, audio_sink_running());
        diag_mark(DIAG_MARK_LOG);
        monitor_task(ev);
    }
}
