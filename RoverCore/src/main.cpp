// main.cpp — Tiny C++ server for LD06/LD19 on Raspberry Pi
// - Reads /dev/serial0 (230400 8N1) and parses 0x54 0x2C 46-byte frames
// - Exposes a minimal HTTP server on port 8000
//   * "/"      -> serves an HTML page with a Canvas viewer
//   * "/stream"-> Server-Sent Events (SSE), JSON lines: [[angle_deg_int, dist_mm_int], ...]
// Notes:
//   * No external deps (just POSIX + C++17)
//   * Add your user to 'dialout' or run with sudo to access /dev/serial0
//   * Env overrides: LIDAR_PORT (default "/dev/serial0"), LIDAR_BAUD (default 230400)
//                    MAX_RANGE_MM (default 6000), STREAM_HZ (default 20)
// Build (with CMake): see CMakeLists.txt example in chat
// Run: ./lidar_server  (then open http://<pi>:8000/?rot=90&cw=1)

#include <atomic>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <mutex>
#include <poll.h>
#include <signal.h>
#include <string>
#include <string_view>
#include <thread>
#include <unistd.h>
#include <vector>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <termios.h>

// ---------------- Config via environment ----------------
static std::string env_str(const char* k, const char* defv){
  const char* v = ::getenv(k); return v? std::string(v) : std::string(defv);
}
static int env_int(const char* k, int defv){
  const char* v = ::getenv(k); return v? std::atoi(v) : defv;
}
static double env_double(const char* k, double defv){
  const char* v = ::getenv(k); return v? std::atof(v) : defv;
}

static std::string g_port_dev = env_str("LIDAR_PORT", "/dev/serial0");
static int g_baud             = env_int("LIDAR_BAUD", 230400);
static int g_max_range_mm     = env_int("MAX_RANGE_MM", 6000);
static double g_stream_hz     = env_double("STREAM_HZ", 20.0);
static int g_http_port        = env_int("PORT", 8000);

// ---------------- Shared scan buffer ----------------
static std::array<int,360> g_dist_deg; // mm; 0 = no data
static std::mutex g_dist_mtx;
static std::atomic<bool> g_run{true};

// ---------------- Utilities ----------------
static uint64_t now_ms(){ struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return (uint64_t)ts.tv_sec*1000ULL + ts.tv_nsec/1000000ULL; }

// Minimal EMA smoothing
static inline int ema(int prev, int val, double alpha){
  if(prev<=0) return val;
  double nd = prev + alpha*(val - prev);
  if(nd < 1.0) nd = (double)val; // avoid zeroing
  return (int)(nd + 0.5);
}

// ---------------- Serial setup ----------------
static speed_t baud_to_constant(int baud){
  switch(baud){
    case 9600: return B9600; case 19200: return B19200; case 38400: return B38400;
    case 57600: return B57600; case 115200: return B115200; case 230400: return B230400;
    default: return B230400; // fallback
  }
}

static int open_serial(const char* dev, int baud){
  int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if(fd < 0){ perror("open serial"); return -1; }

  struct termios tio{};
  if(tcgetattr(fd, &tio) != 0){ perror("tcgetattr"); ::close(fd); return -1; }

  cfmakeraw(&tio); // raw mode
  cfsetspeed(&tio, baud_to_constant(baud));
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSTOPB; // 1 stop bit
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cflag &= ~PARENB; // no parity
  tio.c_cflag &= ~CSIZE;  // 8 data bits
  tio.c_cflag |= CS8;

  tio.c_cc[VMIN]  = 0; // non-blocking read
  tio.c_cc[VTIME] = 0;

  if(tcsetattr(fd, TCSANOW, &tio) != 0){ perror("tcsetattr"); ::close(fd); return -1; }
  tcflush(fd, TCIFLUSH);
  return fd;
}

// ---------------- LD06/LD19 frame parser ----------------
// Frame: 46 bytes: [0]=0x54 [1]=0x2C [2-3]=speed [4-5]=startAng*100 [6..41]=12*(dL,dH,intensity) [42-43]=endAng*100 [44-45]=ts/crc
static void lidar_reader(){
  int fd = open_serial(g_port_dev.c_str(), g_baud);
  if(fd < 0){ fprintf(stderr, "Failed to open %s\n", g_port_dev.c_str()); return; }

  std::vector<uint8_t> buf; buf.reserve(4096);
  const double alpha = 0.35; // EMA

  while(g_run.load()){
    // poll for up to 50ms
    struct pollfd pfd{fd, POLLIN, 0};
    int pr = ::poll(&pfd, 1, 50);
    if(pr > 0 && (pfd.revents & POLLIN)){
      uint8_t tmp[512];
      ssize_t r = ::read(fd, tmp, sizeof(tmp));
      if(r > 0) buf.insert(buf.end(), tmp, tmp + r);
    }

    // parse frames
    for(;;){
      // find header 0x54 0x2C
      size_t i = 0; bool found=false;
      for(; i + 46 <= buf.size(); ++i){
        if(buf[i] == 0x54 && buf[i+1] == 0x2C){ found = true; break; }
      }
      if(!found){
        // keep tail
        if(buf.size() > 2048){ buf.erase(buf.begin(), buf.end()-1024); }
        break;
      }
      if(i + 46 > buf.size()) break; // need more

      // got a frame
      const uint8_t* f = &buf[i];
      // erase up to end of frame
      buf.erase(buf.begin(), buf.begin() + (i + 46));

      // decode angles (centi-deg)
      int sa_cent = (int)f[4] | ((int)f[5] << 8);
      int ea_cent = (int)f[42] | ((int)f[43] << 8);
      int span_cent = ea_cent - sa_cent; if(span_cent < 0) span_cent += 36000;

      // distances
      int n = 12; int denom = (n>1)? (n-1) : 1;
      std::lock_guard<std::mutex> lk(g_dist_mtx);
      for(int k=0;k<12;k++){
        const uint8_t dL = f[6 + k*3 + 0];
        const uint8_t dH = f[6 + k*3 + 1];
        int dmm = ((int)dH << 8) | (int)dL;
        if(dmm <= 0 || dmm > g_max_range_mm) continue;
        int ang_cent = sa_cent + (span_cent * k) / denom;
        int ang_deg = (ang_cent / 100) % 360;
        int prev = g_dist_deg[ang_deg];
        if(prev && std::abs(dmm - prev) > 1200) continue; // outlier
        g_dist_deg[ang_deg] = ema(prev, dmm, alpha);
      }
    }
  }
  ::close(fd);
}

// ---------------- HTTP server ----------------
static const char* kIndexHTML = R"HTML(<!doctype html>
<meta charset="utf-8">
<title>LIDAR live points</title>
<style>
  html,body{margin:0;height:100%;background:#0b0b0b;color:#cfd3d7;font:13px system-ui,Segoe UI,Roboto,Ubuntu,sans-serif}
  #hud{position:fixed;left:10px;top:8px;opacity:.9;user-select:none}
  canvas{display:block;width:100vw;height:100vh}
  .chip{background:#12161a;border:1px solid #1e242a;border-radius:8px;padding:6px 8px;margin-right:6px;display:inline-block}
  .chip code{color:#9ad7ff}
</style>
<div id="hud">
  <span class="chip">drag to pan</span>
  <span class="chip">wheel to zoom</span>
  <span class="chip">R to reset</span>
  <span class="chip">rot=<code>?rot=90</code> cw=<code>?cw=1</code></span>
  <span id="stat" class="chip">fps: --  pts: --</span>
</div>
<canvas id="c"></canvas>
<script>
const canvas=document.getElementById('c');
const ctx=canvas.getContext('2d',{alpha:false});
let W=0,H=0; function resize(){W=canvas.width=innerWidth;H=canvas.height=innerHeight} addEventListener('resize',resize); resize();
// Angle mapping controls
const params=new URLSearchParams(location.search);
let ANGLE_OFFSET=Number(params.get('rot')??90); // default 0° up
let CW=(params.get('cw')??'1')!=='0';
function mapAngle(a){ let t=((CW?-a:a)+ANGLE_OFFSET)%360; if(t<0) t+=360; return t|0; }
// State
let zoom=120, cx=0, cy=0; let pts=[];
const SIN=new Float32Array(360), COS=new Float32Array(360); for(let a=0;a<360;a++){const r=a*Math.PI/180; SIN[a]=Math.sin(r); COS[a]=Math.cos(r);} 
let lastT=performance.now(), frames=0, fps=0; const stat=document.getElementById('stat');
function draw(){
  const now=performance.now(); frames++; if(now-lastT>500){ fps=Math.round(frames*1000/(now-lastT)); frames=0; lastT=now; }
  ctx.fillStyle='#0b0b0b'; ctx.fillRect(0,0,W,H);
  ctx.save(); ctx.translate(W/2,H/2); ctx.scale(1,-1);
  ctx.strokeStyle='#1f2429'; ctx.lineWidth=1; for(let r=1;r<=5;r++){ ctx.beginPath(); ctx.arc(cx*zoom,cy*zoom,r*zoom,0,Math.PI*2); ctx.stroke(); }
  ctx.beginPath(); ctx.moveTo(-W,cy*zoom); ctx.lineTo(W,cy*zoom); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx*zoom,-H); ctx.lineTo(cx*zoom,H); ctx.stroke();
  ctx.fillStyle='#89d2ff'; const z=zoom/1000; const ox=cx*zoom, oy=cy*zoom;
  for(let i=0;i<pts.length;i++){ const a=mapAngle(pts[i][0]); const d=pts[i][1]|0; const x=ox+COS[a]*d*z; const y=oy+SIN[a]*d*z; ctx.fillRect(x-1,y-1,2,2); }
  ctx.restore(); stat.textContent=`fps: ${fps}  pts: ${pts.length}  rot=${ANGLE_OFFSET}°  cw=${CW?1:0}`; requestAnimationFrame(draw);
}
requestAnimationFrame(draw);
let dragging=false,lx=0,ly=0; canvas.addEventListener('mousedown',e=>{dragging=true;lx=e.clientX;ly=e.clientY;}); addEventListener('mouseup',_=>dragging=false);
addEventListener('mousemove',e=>{ if(!dragging) return; const dx=e.clientX-lx, dy=e.clientY-ly; lx=e.clientX; ly=e.clientY; cx+=dx/zoom; cy-=dy/zoom; });
canvas.addEventListener('wheel',e=>{ const k=e.deltaY<0?1.12:1/1.12; zoom=Math.max(20,Math.min(900,zoom*k)); e.preventDefault(); },{passive:false});
addEventListener('keydown',e=>{ if(e.key==='r'||e.key==='R'){ zoom=120; cx=0; cy=0; } });
const es=new EventSource('/stream'); es.onmessage=e=>{ try{ pts=JSON.parse(e.data); }catch(_){} };
</script>)HTML";

static const char* kHTTP200 = "HTTP/1.1 200 OK\r\n";
static const char* kHTTP404 = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";

static void http_send_all(int fd, const char* data, size_t len){
  size_t off=0; while(off<len){ ssize_t w = ::send(fd, data+off, len-off, 0); if(w<=0){ if(errno==EINTR) continue; break; } off += (size_t)w; }
}
static void http_send_all_str(int fd, const std::string& s){ http_send_all(fd, s.c_str(), s.size()); }

static void handle_client(int cfd){
  // Read a very small request header
  char req[1024]; int r = ::recv(cfd, req, sizeof(req)-1, 0); if(r<=0){ ::close(cfd); return; } req[r]=0;
  // Parse first line
  std::string_view sv(req);
  size_t sp1 = sv.find(' '); if(sp1==std::string_view::npos){ ::close(cfd); return; }
  size_t sp2 = sv.find(' ', sp1+1); if(sp2==std::string_view::npos){ ::close(cfd); return; }
  std::string path(sv.substr(sp1+1, sp2-(sp1+1)));

  if(path == "/" || path.rfind("/index.html",0)==0 || path.rfind("/?",0)==0){
    // Serve index
    std::string hdr = std::string(kHTTP200) +
      "Content-Type: text/html; charset=utf-8\r\n" +
      "Content-Length: " + std::to_string(::strlen(kIndexHTML)) + "\r\n" +
      "Connection: close\r\n\r\n";
    http_send_all_str(cfd, hdr);
    http_send_all(cfd, kIndexHTML, ::strlen(kIndexHTML));
    ::close(cfd);
    return;
  }

  if(path == "/stream"){
    // SSE: never ends until client disconnects
    std::string hdr = std::string(kHTTP200) +
      "Content-Type: text/event-stream\r\n" +
      "Cache-Control: no-cache\r\n" +
      "Connection: keep-alive\r\n\r\n";
    http_send_all_str(cfd, hdr);

    const uint64_t period_ms = (uint64_t)std::max(10.0, 1000.0 / (g_stream_hz>0? g_stream_hz : 20.0));
    std::string line;

    while(g_run.load()){
      // snapshot distances
      std::array<int,360> snap{}; {
        std::lock_guard<std::mutex> lk(g_dist_mtx);
        snap = g_dist_deg;
      }
      // build compact JSON [[a,d],...]
      line.clear();
      line.reserve(4096);
      line += "data:";
      line += "[";
      bool first=true;
      for(int a=0;a<360;a++){
        int d = snap[a]; if(d<=0) continue;
        if(!first) line += ","; first=false;
        line += "[" + std::to_string(a) + "," + std::to_string(d) + "]";
      }
      line += "]\n\n";

      ssize_t w = ::send(cfd, line.c_str(), line.size(), 0);
      if(w <= 0){ break; }
      // sleep
      uint64_t start = now_ms();
      while(now_ms() - start < period_ms) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    }
    ::close(cfd);
    return;
  }

  // 404
  http_send_all(cfd, kHTTP404, ::strlen(kHTTP404));
  ::close(cfd);
}

static void http_server(){
  int sfd = ::socket(AF_INET, SOCK_STREAM, 0);
  if(sfd < 0){ perror("socket"); return; }
  int yes=1; setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  sockaddr_in addr{}; addr.sin_family=AF_INET; addr.sin_port=htons((uint16_t)g_http_port); addr.sin_addr.s_addr=INADDR_ANY;
  if(bind(sfd, (sockaddr*)&addr, sizeof(addr))<0){ perror("bind"); ::close(sfd); return; }
  if(listen(sfd, 8)<0){ perror("listen"); ::close(sfd); return; }
  printf("Serving on http://0.0.0.0:%d\n", g_http_port);
  printf("Env: LIDAR_PORT=%s  LIDAR_BAUD=%d  MAX_RANGE_MM=%d  STREAM_HZ=%.1f\n",
         g_port_dev.c_str(), g_baud, g_max_range_mm, g_stream_hz);
  printf("Open: /?rot=90&cw=1 for typical CW 0° up.\n");

  while(g_run.load()){
    struct pollfd pfd{sfd, POLLIN, 0};
    int pr = ::poll(&pfd, 1, 200);
    if(pr > 0 && (pfd.revents & POLLIN)){
      sockaddr_in cli{}; socklen_t cl = sizeof(cli);
      int cfd = ::accept(sfd, (sockaddr*)&cli, &cl);
      if(cfd < 0){ if(errno==EINTR) continue; else continue; }
      std::thread(handle_client, cfd).detach();
    }
  }
  ::close(sfd);
}

static void on_sigint(int){ g_run.store(false); }

int main(){
  signal(SIGINT, on_sigint);
  g_dist_deg.fill(0);

  std::thread t_reader(lidar_reader);
  std::thread t_http(http_server);

  t_reader.join();
  t_http.join();
  return 0;
}
