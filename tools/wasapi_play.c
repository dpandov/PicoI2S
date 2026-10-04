// Plays the usb2spdif self-test pattern (src/selftest/selftest_pattern.h)
// through WASAPI in exclusive, event-driven mode, then a little silence so
// the stream ends cleanly. Bit-exact, unlike PortAudio's exclusive mode.
// Windows only; built by tools/selftest.py:
//
//   python -m ziglang cc -target x86_64-windows-gnu -O2 wasapi_play.c
//       -o wasapi_play.exe -lole32
//
// Usage: wasapi_play <rate> <16|24> <seconds> [device name part]
//        wasapi_play --set-volume-max [device name part]
// The device is the first active render endpoint whose name contains the
// given part (default: the bridge's name in either mode). Ctrl+C ends the
// pattern early.
//
// The device has a volume control (Feature Unit), which Windows applies in
// exclusive mode too, so anything below 100 % or muted is not bit-exact.
// Playing first sets the endpoint volume to 100 % and unmutes it (a
// failure is only reported); --set-volume-max does only that.
//
// Exit codes: 0 played (or volume set), 1 usage, 2 device not found,
// 3 format or stream setup rejected, 4 playback failed, 5 volume not set.
#define COBJMACROS
#define INITGUID
#include <windows.h>
#include <audioclient.h>
#include <endpointvolume.h>
#include <functiondiscoverykeys_devpkey.h>
#include <ksmedia.h>
#include <mmdeviceapi.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>

#include "../src/selftest/selftest_pattern.h"

enum { EXIT_OK = 0, EXIT_USAGE, EXIT_NO_DEVICE, EXIT_SETUP, EXIT_PLAYBACK, EXIT_VOLUME };

#define DEFAULT_DEVICE      L"USB to S/PDIF"     // UAC2 mode
#define DEFAULT_DEVICE_UAC1 L"USB to SPDIF"
#define TRAILING_BUFFERS    4u
#define EVENT_TIMEOUT_MS    2000u

//--------------------------------------------------------------------+
// Pattern
//--------------------------------------------------------------------+

// Writes frames of the pattern, continuing from *next: interleaved
// little-endian int16 (16 bits) or int32 with the value in bits 31..8
// (24 bits), as the device receives it.
static void pattern_fill(void *dst, uint32_t frames, unsigned bits, uint64_t *next) {
    for (uint32_t i = 0; i < frames; i++) {
        uint32_t n = (uint32_t)(*next)++;
        if (bits == 16) {
            uint16_t l = (uint16_t)(n * SELFTEST_STEP16);
            ((uint16_t *)dst)[2 * i] = l;
            ((uint16_t *)dst)[2 * i + 1] = (uint16_t)~l;
        } else {
            uint32_t l = (n * SELFTEST_STEP24) & SELFTEST_MASK24;
            ((uint32_t *)dst)[2 * i] = l << 8;
            ((uint32_t *)dst)[2 * i + 1] = (~l & SELFTEST_MASK24) << 8;
        }
    }
}

typedef struct {
    unsigned bits;
    uint64_t pattern_frames;    // pattern frames still to send
    uint64_t sent;              // pattern frames sent
    uint64_t silence;           // silence frames sent
} source_t;

static void source_fill(source_t *s, BYTE *dst, uint32_t frames) {
    uint32_t bytes_per_frame = s->bits == 16 ? 4u : 8u;
    uint32_t n = s->pattern_frames < frames ? (uint32_t)s->pattern_frames : frames;
    pattern_fill(dst, n, s->bits, &s->sent);
    s->pattern_frames -= n;
    memset(dst + (size_t)n * bytes_per_frame, 0, (size_t)(frames - n) * bytes_per_frame);
    s->silence += frames - n;
}

//--------------------------------------------------------------------+
// WASAPI
//--------------------------------------------------------------------+

static volatile LONG stop_requested;

static BOOL WINAPI on_ctrl(DWORD type) {
    (void)type;
    InterlockedExchange(&stop_requested, 1);
    return TRUE;
}

static const char *hr_hint(HRESULT hr) {
    switch (hr) {
    case AUDCLNT_E_UNSUPPORTED_FORMAT:        return "format not supported by the device";
    case AUDCLNT_E_DEVICE_IN_USE:             return "device in use by another exclusive-mode stream";
    case AUDCLNT_E_EXCLUSIVE_MODE_NOT_ALLOWED:
        return "exclusive mode disabled in the device's Advanced properties";
    case AUDCLNT_E_DEVICE_INVALIDATED:        return "device removed or reconfigured";
    case AUDCLNT_E_ENDPOINT_CREATE_FAILED:    return "endpoint creation failed";
    case AUDCLNT_E_INVALID_DEVICE_PERIOD:     return "invalid device period";
    case E_OUTOFMEMORY:                       return "out of memory";
    default:                                  return "";
    }
}

static void report_hr(const char *what, HRESULT hr) {
    const char *hint = hr_hint(hr);
    fprintf(stderr, "wasapi_play: %s failed: 0x%08lx%s%s\n", what, (unsigned long)hr,
            *hint ? ", " : "", hint);
}

static void release(void *p) {
    if (p) {
        IUnknown_Release((IUnknown *)p);
    }
}

// First active render endpoint whose friendly name contains part (or alt,
// if given). With list set, prints every name to stderr instead.
static IMMDevice *find_device(IMMDeviceEnumerator *en, const wchar_t *part, const wchar_t *alt,
                              BOOL list) {
    IMMDeviceCollection *col = NULL;
    IMMDevice *found = NULL;
    UINT count = 0;
    if (FAILED(IMMDeviceEnumerator_EnumAudioEndpoints(en, eRender, DEVICE_STATE_ACTIVE, &col))) {
        return NULL;
    }
    IMMDeviceCollection_GetCount(col, &count);
    for (UINT i = 0; i < count && !found; i++) {
        IMMDevice *d = NULL;
        IPropertyStore *ps = NULL;
        PROPVARIANT v;
        PropVariantInit(&v);
        if (SUCCEEDED(IMMDeviceCollection_Item(col, i, &d)) &&
            SUCCEEDED(IMMDevice_OpenPropertyStore(d, STGM_READ, &ps)) &&
            SUCCEEDED(IPropertyStore_GetValue(ps, &PKEY_Device_FriendlyName, &v)) &&
            v.vt == VT_LPWSTR) {
            if (list) {
                fprintf(stderr, "  %ls\n", v.pwszVal);
            } else if (wcsstr(v.pwszVal, part) || (alt && wcsstr(v.pwszVal, alt))) {
                printf("device: %ls\n", v.pwszVal);
                found = d;
                d = NULL;
            }
        }
        PropVariantClear(&v);
        release(ps);
        release(d);
    }
    release(col);
    return found;
}

// Endpoint volume to 100 % and unmuted, read back. False on any failure.
static BOOL set_volume_max(IMMDevice *dev) {
    IAudioEndpointVolume *ev = NULL;
    HRESULT hr = IMMDevice_Activate(dev, &IID_IAudioEndpointVolume, CLSCTX_ALL, NULL, (void **)&ev);
    if (FAILED(hr)) {
        report_hr("IMMDevice::Activate(IAudioEndpointVolume)", hr);
        return FALSE;
    }
    float level = 0.0f, db = 0.0f;
    BOOL mute = TRUE;
    if (FAILED(hr = IAudioEndpointVolume_SetMasterVolumeLevelScalar(ev, 1.0f, NULL)) ||
        FAILED(hr = IAudioEndpointVolume_SetMute(ev, FALSE, NULL)) ||
        FAILED(hr = IAudioEndpointVolume_GetMasterVolumeLevelScalar(ev, &level)) ||
        FAILED(hr = IAudioEndpointVolume_GetMasterVolumeLevel(ev, &db)) ||
        FAILED(hr = IAudioEndpointVolume_GetMute(ev, &mute))) {
        report_hr("endpoint volume", hr);
        release(ev);
        return FALSE;
    }
    release(ev);
    printf("volume: %.0f %% (%.1f dB)%s\n", 100.0 * level, db, mute ? ", muted" : "");
    fflush(stdout);
    if (level < 1.0f || mute) {
        fprintf(stderr, "wasapi_play: endpoint volume still %.0f %%%s\n", 100.0 * level,
                mute ? ", muted" : "");
        return FALSE;
    }
    return TRUE;
}

static void make_format(WAVEFORMATEXTENSIBLE *wf, uint32_t rate, unsigned bits) {
    memset(wf, 0, sizeof(*wf));
    wf->Format.wFormatTag = WAVE_FORMAT_EXTENSIBLE;
    wf->Format.nChannels = 2;
    wf->Format.nSamplesPerSec = rate;
    wf->Format.wBitsPerSample = (WORD)(bits == 16 ? 16 : 32);
    wf->Format.nBlockAlign = (WORD)(2 * wf->Format.wBitsPerSample / 8);
    wf->Format.nAvgBytesPerSec = rate * wf->Format.nBlockAlign;
    wf->Format.cbSize = sizeof(*wf) - sizeof(WAVEFORMATEX);
    wf->Samples.wValidBitsPerSample = (WORD)bits;
    wf->dwChannelMask = SPEAKER_FRONT_LEFT | SPEAKER_FRONT_RIGHT;
    wf->SubFormat = KSDATAFORMAT_SUBTYPE_PCM;
}

// Exclusive, event-driven, buffer = one device period. Re-initializes with
// an aligned period if the driver asks for it.
static HRESULT open_stream(IMMDevice *dev, const WAVEFORMATEXTENSIBLE *wf, IAudioClient **out) {
    IAudioClient *ac = NULL;
    HRESULT hr = IMMDevice_Activate(dev, &IID_IAudioClient, CLSCTX_ALL, NULL, (void **)&ac);
    if (FAILED(hr)) {
        report_hr("IMMDevice::Activate", hr);
        return hr;
    }
    hr = IAudioClient_IsFormatSupported(ac, AUDCLNT_SHAREMODE_EXCLUSIVE,
                                        (const WAVEFORMATEX *)wf, NULL);
    if (hr != S_OK) {
        report_hr("IAudioClient::IsFormatSupported", FAILED(hr) ? hr : AUDCLNT_E_UNSUPPORTED_FORMAT);
        release(ac);
        return FAILED(hr) ? hr : AUDCLNT_E_UNSUPPORTED_FORMAT;
    }
    REFERENCE_TIME def_period = 0, min_period = 0;
    IAudioClient_GetDevicePeriod(ac, &def_period, &min_period);
    hr = IAudioClient_Initialize(ac, AUDCLNT_SHAREMODE_EXCLUSIVE, AUDCLNT_STREAMFLAGS_EVENTCALLBACK,
                                 def_period, def_period, (const WAVEFORMATEX *)wf, NULL);
    if (hr == AUDCLNT_E_BUFFER_SIZE_NOT_ALIGNED) {
        UINT32 frames = 0;
        IAudioClient_GetBufferSize(ac, &frames);
        release(ac);
        ac = NULL;
        REFERENCE_TIME period = (REFERENCE_TIME)(10000000.0 * frames / wf->Format.nSamplesPerSec + 0.5);
        hr = IMMDevice_Activate(dev, &IID_IAudioClient, CLSCTX_ALL, NULL, (void **)&ac);
        if (SUCCEEDED(hr)) {
            hr = IAudioClient_Initialize(ac, AUDCLNT_SHAREMODE_EXCLUSIVE,
                                         AUDCLNT_STREAMFLAGS_EVENTCALLBACK, period, period,
                                         (const WAVEFORMATEX *)wf, NULL);
        }
    }
    if (FAILED(hr)) {
        report_hr("IAudioClient::Initialize", hr);
        release(ac);
        return hr;
    }
    *out = ac;
    return S_OK;
}

// Raises the thread to the MMCSS "Pro Audio" class if available.
static void mmcss_enter(void) {
    typedef HANDLE(WINAPI * set_fn)(LPCSTR, LPDWORD);
    HMODULE avrt = LoadLibraryA("avrt.dll");
    set_fn set = avrt ? (set_fn)(void *)GetProcAddress(avrt, "AvSetMmThreadCharacteristicsA") : NULL;
    DWORD task = 0;
    if (set) {
        set("Pro Audio", &task);
    }
}

static int play(IAudioClient *ac, source_t *src, uint32_t rate) {
    IAudioRenderClient *rc = NULL;
    HANDLE ev = CreateEventW(NULL, FALSE, FALSE, NULL);
    UINT32 buf_frames = 0;
    BYTE *p = NULL;
    int result = EXIT_PLAYBACK;
    HRESULT hr;

    if (!ev || FAILED(hr = IAudioClient_SetEventHandle(ac, ev)) ||
        FAILED(hr = IAudioClient_GetBufferSize(ac, &buf_frames)) ||
        FAILED(hr = IAudioClient_GetService(ac, &IID_IAudioRenderClient, (void **)&rc))) {
        report_hr("stream setup", ev ? hr : HRESULT_FROM_WIN32(GetLastError()));
        result = EXIT_SETUP;
        goto done;
    }
    printf("format: %u Hz %u-bit, buffer %u frames (%.2f ms)\n", (unsigned)rate, src->bits,
           (unsigned)buf_frames, 1000.0 * buf_frames / rate);
    fflush(stdout);

    if (FAILED(hr = IAudioRenderClient_GetBuffer(rc, buf_frames, &p))) {
        report_hr("IAudioRenderClient::GetBuffer", hr);
        goto done;
    }
    source_fill(src, p, buf_frames);
    IAudioRenderClient_ReleaseBuffer(rc, buf_frames, 0);

    mmcss_enter();
    uint64_t trailing = (uint64_t)TRAILING_BUFFERS * buf_frames;
    if (FAILED(hr = IAudioClient_Start(ac))) {
        report_hr("IAudioClient::Start", hr);
        goto done;
    }
    for (;;) {
        if (stop_requested && src->pattern_frames) {
            src->pattern_frames = 0;
        }
        if (src->pattern_frames == 0 && src->silence >= trailing) {
            break;
        }
        if (WaitForSingleObject(ev, EVENT_TIMEOUT_MS) != WAIT_OBJECT_0) {
            fprintf(stderr, "wasapi_play: no buffer event for %u ms\n", EVENT_TIMEOUT_MS);
            IAudioClient_Stop(ac);
            goto done;
        }
        if (FAILED(hr = IAudioRenderClient_GetBuffer(rc, buf_frames, &p))) {
            report_hr("IAudioRenderClient::GetBuffer", hr);
            IAudioClient_Stop(ac);
            goto done;
        }
        source_fill(src, p, buf_frames);
        IAudioRenderClient_ReleaseBuffer(rc, buf_frames, 0);
    }
    // Let the last buffers play out.
    Sleep((DWORD)(2000u * buf_frames / rate) + 20u);
    IAudioClient_Stop(ac);
    result = EXIT_OK;

done:
    release(rc);
    if (ev) {
        CloseHandle(ev);
    }
    return result;
}

//--------------------------------------------------------------------+
// Main
//--------------------------------------------------------------------+

static int usage(void) {
    fprintf(stderr, "usage: wasapi_play <rate> <16|24> <seconds> [device name part]\n"
                    "       wasapi_play --set-volume-max [device name part]\n");
    return EXIT_USAGE;
}

int main(int argc, char **argv) {
    BOOL volume_only = argc >= 2 && strcmp(argv[1], "--set-volume-max") == 0;
    int n_args = volume_only ? 1 : 3;
    if (argc < 1 + n_args || argc > 2 + n_args) {
        return usage();
    }
    char *end;
    unsigned long rate = 0, bits = 0;
    double seconds = 0;
    if (!volume_only) {
        rate = strtoul(argv[1], &end, 10);
        if (*end || rate < 8000 || rate > 384000) {
            return usage();
        }
        bits = strtoul(argv[2], &end, 10);
        if (*end || (bits != 16 && bits != 24)) {
            return usage();
        }
        seconds = strtod(argv[3], &end);
        if (*end || !(seconds > 0) || seconds > 1e6) {
            return usage();
        }
    }
    wchar_t part[128] = DEFAULT_DEVICE;
    const wchar_t *alt = DEFAULT_DEVICE_UAC1;
    if (argc == 2 + n_args) {
        if (!MultiByteToWideChar(CP_ACP, 0, argv[1 + n_args], -1, part,
                                 (int)(sizeof(part) / sizeof(part[0])))) {
            return usage();
        }
        alt = NULL;
    }

    SetConsoleCtrlHandler(on_ctrl, TRUE);
    HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);
    if (FAILED(hr)) {
        report_hr("CoInitializeEx", hr);
        return EXIT_SETUP;
    }

    int result;
    IMMDeviceEnumerator *en = NULL;
    IMMDevice *dev = NULL;
    IAudioClient *ac = NULL;
    hr = CoCreateInstance(&CLSID_MMDeviceEnumerator, NULL, CLSCTX_ALL, &IID_IMMDeviceEnumerator,
                          (void **)&en);
    if (FAILED(hr)) {
        report_hr("CoCreateInstance(MMDeviceEnumerator)", hr);
        result = EXIT_SETUP;
    } else if (!(dev = find_device(en, part, alt, FALSE))) {
        fprintf(stderr, "wasapi_play: no active playback device with \"%ls\" in its name; found:\n",
                part);
        find_device(en, part, alt, TRUE);
        result = EXIT_NO_DEVICE;
    } else if (volume_only) {
        result = set_volume_max(dev) ? EXIT_OK : EXIT_VOLUME;
    } else {
        if (!set_volume_max(dev)) {
            fprintf(stderr, "wasapi_play: warning: playing without full volume, "
                            "not bit-exact\n");
        }
        WAVEFORMATEXTENSIBLE wf;
        make_format(&wf, (uint32_t)rate, (unsigned)bits);
        if (FAILED(open_stream(dev, &wf, &ac))) {
            result = EXIT_SETUP;
        } else {
            source_t src = {.bits = (unsigned)bits,
                            .pattern_frames = (uint64_t)(seconds * (double)rate + 0.5)};
            result = play(ac, &src, (uint32_t)rate);
            printf("sent %llu pattern frames, %llu silence frames%s\n",
                   (unsigned long long)src.sent, (unsigned long long)src.silence,
                   stop_requested ? " (interrupted)" : "");
        }
    }

    release(ac);
    release(dev);
    release(en);
    CoUninitialize();
    return result;
}
