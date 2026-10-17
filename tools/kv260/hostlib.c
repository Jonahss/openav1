// Bulk register traffic for the KV260 host driver (tools/kv260/decode.py): the per-access cost from Python
// through /dev/mem is ~7 us, from here ~0.1 us, and a frame is tens of thousands of accesses (1269 CDF default
// rows x 9 words, the tile bytes, the picture). Built as a shared object (tools/kv260/build_hostlib.sh) and
// called through ctypes; the Python HostDecoder keeps the protocol, this only moves words. Every access is one
// aligned 32-bit volatile load or store on a device mapping (uncached, ordered on arm64).
#include <fcntl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define R_STATUS 0x010
#define R_DATA1  0x01C
#define R_DATA4  0x020
#define R_HADDR  0x024
#define R_HDATA  0x028
#define R_HDATA2 0x030
#define R_HDATA4 0x034

static volatile uint32_t *regs;

static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static inline void wr(uint32_t a, uint32_t v) { regs[a >> 2] = v; }
static inline uint32_t rd(uint32_t a) { return regs[a >> 2]; }

int hl_open(uint64_t base) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) return -1;
    void *m = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)base);
    close(fd);
    if (m == MAP_FAILED) return -2;
    regs = (volatile uint32_t *)m;
    return 0;
}

void hl_wr(uint32_t a, uint32_t v) { wr(a, v); }
uint32_t hl_rd(uint32_t a) { return rd(a); }

// (address, value) pairs in order: headers, CDF default rows (8 data words then the address write)
void hl_write_seq(const uint32_t *addr, const uint32_t *val, int n) {
    for (int i = 0; i < n; i++) wr(addr[i], val[i]);
}

// the tile bytes: whole words through DATA4, the tail through DATA1, never more than the FIFO's free count.
// Returns the bytes pushed, or -1 when the FIFO stayed full for timeout_s.
long hl_push(const uint8_t *data, long len, double timeout_s) {
    long pos = 0;
    double t0 = now();
    while (pos < len) {
        uint32_t free = rd(R_STATUS) >> 16;
        if (free == 0) {
            if (now() - t0 > timeout_s) return -1;
            continue;
        }
        t0 = now();
        for (uint32_t k = 0; k < free && pos < len; k++) {
            if (len - pos >= 4) {
                wr(R_DATA4, (uint32_t)data[pos] | (uint32_t)data[pos + 1] << 8 | (uint32_t)data[pos + 2] << 16 | (uint32_t)data[pos + 3] << 24);
                pos += 4;
            } else {
                wr(R_DATA1, data[pos++]);
            }
        }
    }
    return pos;
}

// poll until (reg & mask) == val; returns the last value read, or 0xFFFFFFFF on timeout
uint32_t hl_poll(uint32_t a, uint32_t mask, uint32_t val, double timeout_s) {
    double t0 = now();
    for (;;) {
        uint32_t v = rd(a);
        if ((v & mask) == val) return v;
        if (now() - t0 > timeout_s) return 0xFFFFFFFFu;
    }
}

static int set_haddr(uint32_t a) {
    wr(R_HADDR, a);
    for (int i = 0; i < 1000; i++) if (rd(R_HADDR) == a) return 0;    // the read can overtake the posted write
    return -1;
}

// one plane, W x H samples from (0, y0), packed reads (HDATA4 for 8-bit pictures, HDATA2 otherwise), the odd
// tail sample by sample. Returns 0, or -1 when HADDR never read back.
int hl_read_plane(int plane, int W, int H, int bd8, uint16_t *out) {
    for (int y = 0; y < H; y++) {
        uint16_t *row = out + (long)y * W;
        int x = 0;
        if (set_haddr((uint32_t)plane << 28 | (uint32_t)y << 14)) return -1;
        if (bd8) {
            for (; x + 4 <= W; x += 4) {
                uint32_t v = rd(R_HDATA4);
                row[x] = v & 0xFF; row[x + 1] = v >> 8 & 0xFF; row[x + 2] = v >> 16 & 0xFF; row[x + 3] = v >> 24;
            }
        } else {
            for (; x + 2 <= W; x += 2) {
                uint32_t v = rd(R_HDATA2);
                row[x] = v & 0xFFF; row[x + 1] = v >> 16 & 0xFFF;
            }
        }
        for (; x < W; x++) {
            if (set_haddr((uint32_t)plane << 28 | (uint32_t)y << 14 | (uint32_t)x)) return -1;
            row[x] = rd(R_HDATA) & 0xFFF;
        }
    }
    return 0;
}

// 8-bit 4:2:0 planes (the DMA image: y with ystride, u / v with cstride) to RGB565 at (x0, y0) of a 16-bit
// framebuffer with dst_stride pixels per line (BT.601 limited range, the usual for these clips). Both the
// staging area and the framebuffer are uncached device memory: a byte-wise walk costs ~0.3 us per access, so
// each row is first copied out with wide loads (memcpy) and the output row is assembled in a local buffer and
// copied in with wide stores.
#include <string.h>
// flip: the picture turned by 180 degrees within its rectangle (a panel mounted upside down).
void hl_yuv420_rgb565(const uint8_t *y, const uint8_t *u, const uint8_t *v, int W, int H, int ystride, int cstride,
                      uint16_t *dst, int dst_stride, int x0, int y0, int flip) {
    static uint8_t yr[4096], ur[2048], vr[2048];
    static uint16_t out[4096];
    if (W > 4096) return;
    for (int j = 0; j < H; j++) {
        memcpy(yr, y + (long)j * ystride, (size_t)W);
        if ((j & 1) == 0) {
            memcpy(ur, u + (long)(j >> 1) * cstride, (size_t)((W + 1) >> 1));
            memcpy(vr, v + (long)(j >> 1) * cstride, (size_t)((W + 1) >> 1));
        }
        for (int i = 0; i < W; i++) {
            int c = 298 * (yr[i] - 16), dd = ur[i >> 1] - 128, e = vr[i >> 1] - 128;
            int r = (c + 409 * e + 128) >> 8, g = (c - 100 * dd - 208 * e + 128) >> 8, b = (c + 516 * dd + 128) >> 8;
            r = r < 0 ? 0 : r > 255 ? 255 : r; g = g < 0 ? 0 : g > 255 ? 255 : g; b = b < 0 ? 0 : b > 255 ? 255 : b;
            out[i] = (uint16_t)((r >> 3) << 11 | (g >> 2) << 5 | (b >> 3));
        }
        if (flip) {
            for (int i = 0; i < W / 2; i++) { uint16_t t = out[i]; out[i] = out[W - 1 - i]; out[W - 1 - i] = t; }
            memcpy(dst + (long)(y0 + H - 1 - j) * dst_stride + x0, out, (size_t)W * 2);
        } else {
            memcpy(dst + (long)(y0 + j) * dst_stride + x0, out, (size_t)W * 2);
        }
    }
}
