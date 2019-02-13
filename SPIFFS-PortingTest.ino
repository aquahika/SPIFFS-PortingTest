//
// SPI シリアルフラッシュメモリ W25Q64 操作検証プログラム
// W25Q64のメモリ領域構造
//   総バイト数 8388608
//   メモリ空間 24ビットアドレス指定 0x00000 - 0x7FFFFF
//   ブロック数 128 (64KB/ブロック)
//   セクタ数 2048  ( 4KB/セクタ)
//   総セクタ数 2048

#include <Arduino.h>
#include <string.h>
#include <SPI.h>

#include "src/W25Q64/W25Q64.h"
#include "src/spiffs/spiffs.h"
#include "src/spiffs_hal.h"

//
// 書込みデータのダンプリスト
// dt(in) : データ格納先頭アドレス
// n(in)  : 表示データ数
//
void dump(byte *dt, uint32_t n) {
  char buf[64];
  uint16_t clm = 0;
  byte data;
  byte sum;
  byte vsum[16];
  byte total =0;
  uint32_t saddr =0;
  uint32_t eaddr =n-1;

  Serial.println("----------------------------------------------------------");
  for (uint16_t i=0;i<16;i++) vsum[i]=0;
  for (uint32_t addr = saddr; addr <= eaddr; addr++) {
    data = dt[addr];
    if (clm == 0) {
      sum =0;
      sprintf(buf,"%05lx: ",addr);
      Serial.print(buf);
    }

    sum+=data;
    vsum[addr % 16]+=data;

    sprintf(buf,"%02x ",data);
    Serial.print(buf);
    clm++;
    if (clm == 16) {
      sprintf(buf,"|%02x ",sum);
      Serial.print(buf);
      Serial.println("");
      clm = 0;
    }
  }
  Serial.println("----------------------------------------------------------");
  Serial.print("       ");
  for (uint16_t i=0; i<16;i++) {
    total+=vsum[i];
    sprintf(buf,"%02x ",vsum[i]);
    Serial.print(buf);
  }
  sprintf(buf,"|%02x ",total);
  Serial.print(buf);
  Serial.println("");
  Serial.println("");
}

#define LOG_PAGE_SIZE       256

static u8_t spiffs_work_buf[LOG_PAGE_SIZE*2];
static u8_t spiffs_fds[32*4];
static u8_t spiffs_cache_buf[(LOG_PAGE_SIZE+32)*4];
 static spiffs fs;

s32_t my_spiffs_mount() {
    spiffs_config cfg;
    cfg.hal_read_f = spiffs_hal_read;
    cfg.hal_write_f = spiffs_hal_write;
    cfg.hal_erase_f = spiffs_hal_erase;

    int res = SPIFFS_mount(&fs,
      &cfg,
      spiffs_work_buf,
      spiffs_fds,
      sizeof(spiffs_fds),
      spiffs_cache_buf,
      sizeof(spiffs_cache_buf),
      0);
    printf("mount res: %i\n", res);

    return res;
}

static void test_spiffs() {
  char buf[12];

  // Surely, I've mounted spiffs before entering here

  spiffs_file fd = SPIFFS_open(&fs, "my_file", SPIFFS_CREAT | SPIFFS_TRUNC | SPIFFS_RDWR, 0);
  if (SPIFFS_write(&fs, fd, (u8_t *)"Hello world", 12) < 0) printf("errno %i\n", SPIFFS_errno(&fs));
  SPIFFS_close(&fs, fd);

  fd = SPIFFS_open(&fs, "my_file", SPIFFS_RDWR, 0);
  if (SPIFFS_read(&fs, fd, (u8_t *)buf, 12) < 0) printf("errno %i\n", SPIFFS_errno(&fs));
  SPIFFS_close(&fs, fd);

  printf("--> %s <--\n", buf);
}



SPIClass *vspi = NULL;

void setup() {
    byte buf[256];        // 取得W25Q64_setSPIPortデータ
    byte wdata[16];       // 書込みデータ

    uint16_t n;           // 取得データ数

    Serial.begin(115200);

    for(int i=5;i>=1;i--){
      Serial.println(i);
      delay(1000);
    }

    vspi = new SPIClass(VSPI);
    W25Q64_setSPIPort(*vspi);
    W25Q64_begin(22);     // フラッシュメモリ利用開始

    s32_t res = my_spiffs_mount();
    printf("%d\n",res);
    if(res == SPIFFS_ERR_NOT_A_FS){
      printf("Formatting");
      SPIFFS_format(&fs);
      abort();
    }

    test_spiffs();

}

void loop() {

}
