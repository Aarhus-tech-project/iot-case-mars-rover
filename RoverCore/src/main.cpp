// main.cpp — LD06/LD19 serial reader (no gRPC)
// - Frames: start 0x54 0x2C, length 46 bytes, 12 samples/frame
// - One sample printed per line: "t_ns angle_cdeg distance_mm intensity"
// - ENV:
//     LIDAR_PORT    (default "/dev/serial0")
//     LIDAR_BAUD    (default 230400)
//     MAX_RANGE_MM  (default 6000)

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

static std::string env_str(const char* k, const char* d){ const char* v=getenv(k); return v? v : d; }
static int         env_int(const char* k, int d){ const char* v=getenv(k); return v? std::atoi(v) : d; }

static std::string LIDAR_PORT   = env_str("LIDAR_PORT", "/dev/serial0");
static int         LIDAR_BAUD   = env_int("LIDAR_BAUD", 230400);
static int         MAX_RANGE_MM = env_int("MAX_RANGE_MM", 6000);

static std::atomic<bool> g_run{true};

static inline uint64_t mono_ns(){
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

// -------------------- serial --------------------
static speed_t baud_to_term(int baud){
  switch(baud){
    case 9600: return B9600; case 19200: return B19200; case 38400: return B38400;
    case 57600: return B57600; case 115200: return B115200; case 230400: return B230400;
    default: return B230400;
  }
}

static int open_serial(const char* dev, int baud){
  int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if(fd < 0){ perror("open serial"); return -1; }
  termios tio{}; if(tcgetattr(fd, &tio)!=0){ perror("tcgetattr"); ::close(fd); return -1; }
  cfmakeraw(&tio);
  cfsetspeed(&tio, baud_to_term(baud));
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB; tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;  tio.c_cflag |= CS8;
  tio.c_cc[VMIN]=0; tio.c_cc[VTIME]=0;
  if(tcsetattr(fd, TCSANOW, &tio)!=0){ perror("tcsetattr"); ::close(fd); return -1; }
  tcflush(fd, TCIFLUSH);
  return fd;
}

// -------------------- lidar parser --------------------
// LD06/LD19: 46-byte frame starting with 0x54 0x2C, 12 samples/frame
struct LidarReader {
  std::vector<uint8_t> buf;

  // calls on_point(angle_cdeg, distance_mm, intensity, t_ns) for each valid sample
  bool pump(int fd, const std::function<void(uint32_t,uint32_t,uint32_t,uint64_t)>& on_point){
    bool any=false;

    // read ready bytes (up to 50 ms)
    pollfd pfd{fd, POLLIN, 0};
    int pr = ::poll(&pfd, 1, 50);
    if(pr>0 && (pfd.revents & POLLIN)){
      uint8_t tmp[512];
      ssize_t r = ::read(fd, tmp, sizeof(tmp));
      if(r>0) buf.insert(buf.end(), tmp, tmp + r);
    }

    // parse frames from buffer
    for(;;){
      // seek header
      size_t i=0; bool found=false;
      for(; i + 46 <= buf.size(); ++i){
        if(buf[i] == 0x54 && buf[i+1] == 0x2C){ found = true; break; }
      }
      if(!found){
        if(buf.size() > 2048) buf.erase(buf.begin(), buf.end() - 1024);
        break;
      }
      if(i + 46 > buf.size()) break;

      const uint8_t* f = &buf[i];
      buf.erase(buf.begin(), buf.begin() + (i + 46));

      int sa_cent = (int)f[4] | ((int)f[5] << 8);     // start angle * 100
      int ea_cent = (int)f[42] | ((int)f[43] << 8);   // end angle * 100
      int span    = ea_cent - sa_cent; if(span < 0) span += 36000;
      const int N = 12; const int denom = (N>1)? (N-1) : 1;
      uint64_t t  = mono_ns();

      for(int k=0;k<N;k++){
        const uint8_t dL = f[6 + k*3 + 0];
        const uint8_t dH = f[6 + k*3 + 1];
        const uint8_t I  = f[6 + k*3 + 2];
        uint32_t dmm = ((uint32_t)dH << 8) | (uint32_t)dL;
        if(dmm == 0 || dmm > (uint32_t)MAX_RANGE_MM) continue;
        uint32_t a_cdeg = (uint32_t)(sa_cent + (span * k) / denom) % 36000;
        on_point(a_cdeg, dmm, (uint32_t)I, t);
        any=true;
      }
    }
    return any;
  }
};

// -------------------- main --------------------
static void on_sigint(int){ g_run.store(false); }

int main(){
  std::signal(SIGINT, on_sigint);

  fprintf(stderr, "[lidar] serial=%s @%d  max_range=%dmm\n",
          LIDAR_PORT.c_str(), LIDAR_BAUD, MAX_RANGE_MM);

  int fd = open_serial(LIDAR_PORT.c_str(), LIDAR_BAUD);
  if(fd < 0){
    fprintf(stderr, "[lidar] failed to open %s (need dialout or sudo?)\n", LIDAR_PORT.c_str());
    return 1;
  }

  LidarReader reader;
  uint64_t count = 0, last_log = mono_ns();

  auto print_point = [&](uint32_t angle_cdeg, uint32_t dist_mm, uint32_t intensity, uint64_t t_ns){
    // One line per sample: t angle distance intensity
    // Example: 1234567890123 12345 987 42
    printf("%llu %u %u %u\n",
           (unsigned long long)t_ns, angle_cdeg, dist_mm, intensity);
    ++count;
  };

  while(g_run.load()){
    if(!reader.pump(fd, print_point)){
      // no bytes this tick; tiny sleep to avoid 100% CPU
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    uint64_t now = mono_ns();
    if(now - last_log > 1'000'000'000ULL){
      fprintf(stderr, "[lidar] samples=%llu\n", (unsigned long long)count);
      last_log = now;
    }
  }

  ::close(fd);
  fprintf(stderr, "[lidar] bye\n");
  return 0;
}