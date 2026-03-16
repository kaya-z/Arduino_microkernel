/*
 * Microkernel for Arduino UNO R4
 * Copyright YUKARI Semiconductor Devices.
 * Version: 20260317_fixed
 *
 * Update Log:
 * - Fix [C-1]: SysTick_Handler が複数タスクを同時起床できるよう全件走査に変更
 * - Fix [C-2]: tickless と固定1ms tickの混在を解消。Task_Idle を __WFI() のみに統一
 * - Fix [C-3]: get_next_wake_delay() の uint32_t アンダーフローを修正
 * - Fix [C-4]: Task_MemMgr のアロケーション応答メッセージを MSG_ACK に修正
 * - Fix [M-1]: Task_Shell の sleep コマンド内ローカル変数 msg → buf に改名（シャドーイング解消）
 * - Fix [M-2]: Task_SmellSensor のローカル変数 msg → buf に改名（シャドーイング解消）
 * - Fix [M-3]: Task_Shell の ss コマンドブロックの波括弧ミスを修正（cmdIndex=0 漏れ解消）
 * - Fix [M-4]: 未使用の MAX_TASKS 定義を削除し PID_MAX に統一
 * - Fix [M-5]: PIN_OUTPUT を 3(D3) から A3(=17) に修正
 * - Fix [Blinky-1]: static ローカル変数をグローバル変数に変更（非標準呼び出しでの初期化問題を回避）
 * - Fix [Blinky-2]: MSG_LED_ON 受信時に blinky_led_on をリセット
 * - Fix [Blinky-3]: led_on==true ブランチ末尾の冗長な digitalWrite(LOW) を削除
 * - Fix [Blinky-4]: 非アクティブ時の不要な毎ループ digitalWrite(LOW) を sys_yield() のみに変更
 * - Fix [m-1]: PID_MAX コメントの誤り（= 6 → = 7）を修正
 */

#include <Arduino.h>

// ==========================================
// 1. システム設定 & 定数
// ==========================================

#define STACK_SIZE 512  // スタックサイズ (uint32_t単位 = 2048 Bytes)
// [Fix M-4] MAX_TASKS を削除。PID_MAX に統一。

#define PIN_HEATER 14    // D14(A0)
#define PIN_SENSOR 15    // D15(A1)
#define PIN_OUTPUT A3    // [Fix M-5] 3(D3) → A3(=17) に修正

// プロセスID
enum PID {
  PID_IDLE = 0,
  PID_MEM_MGR,
  PID_FILE_MGR,
  PID_DRIVER_SERIAL,
  PID_SHELL,
  PID_BLINKY,
  PID_SMELL_SENSOR,
  PID_MAX       // [Fix m-1] PID_MAX = 7
};

// タスク名配列
const char* TASK_NAMES[PID_MAX] = {
    "IDLE",
    "MEM_MGR",
    "FILE_MGR",
    "SERIAL_DRV",
    "SHELL",
    "BLINKY",
    "SMELL_SENS"
};

// メッセージタイプ
enum MsgType {
  MSG_NONE = 0,
  MSG_ACK,
  MSG_ERROR,
  MSG_MEM_ALLOC_REQ,
  MSG_MEM_FREE_REQ,
  MSG_MEM_FREE_OK,
  MSG_FS_CREATE,
  MSG_FS_WRITE,
  MSG_FS_LIST,
  MSG_SERIAL_IN,
  MSG_SERIAL_OUT,
  MSG_SERIAL_OUT_LN,
  MSG_SS_ON,
  MSG_SS_OFF,
  MSG_LED_ON,
  MSG_LED_OFF
};

// メッセージ構造体
struct Message {
  PID sender;
  PID receiver;
  MsgType type;
  int param1;
  char payload[16]; // IPCペイロードは16バイトまで
};

// ==========================================
// 2. IPC (Inter-Process Communication)
// ==========================================

#define QUEUE_SIZE 10

struct Mailbox {
  Message buffer[QUEUE_SIZE];
  int head;
  int tail;
  int count;
};

Mailbox mailboxes[PID_MAX];

bool ipc_send(PID receiver, MsgType type, int param1, const char* payload, PID sender) {
  bool success = false;

  __disable_irq();

  Mailbox* mb = &mailboxes[receiver];
  if (mb->count < QUEUE_SIZE) {
    Message* msg = &mb->buffer[mb->tail];
    msg->sender   = sender;
    msg->receiver = receiver;
    msg->type     = type;
    msg->param1   = param1;
    memset(msg->payload, 0, 16);
    if (payload) strncpy(msg->payload, payload, 15);

    mb->tail = (mb->tail + 1) % QUEUE_SIZE;
    mb->count++;
    success = true;
  }

  __enable_irq();
  return success;
}

bool ipc_receive(PID receiver, Message* outMsg) {
  bool success = false;

  __disable_irq();

  Mailbox* mb = &mailboxes[receiver];
  if (mb->count > 0) {
    *outMsg = mb->buffer[mb->head];
    mb->head = (mb->head + 1) % QUEUE_SIZE;
    mb->count--;
    success = true;
  }

  __enable_irq();
  return success;
}

// OSシステムコール: CPU時間を自発的に譲る
void sys_yield() {
  SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
  __ISB();
  __DSB();
}

// ==========================================
// 3. スケジューラ データ構造
// ==========================================

enum TaskState {
    STATE_READY = 0,
    STATE_SLEEPING
};

struct TCB {
  uint32_t* sp;
  uint32_t* stack_base;
  TaskState state;
  uint32_t  wake_time;
}; // TCBサイズは16バイト

TCB      tasks[PID_MAX];
uint32_t taskStacks[PID_MAX][STACK_SIZE];

volatile int      currentTask   = 0;
volatile int      nextTask      = 0;
volatile uint32_t global_time_ms = 0;

// OSシステムコール: タスクをスリープ状態にする
void sys_sleep(uint32_t duration_ms) {
  __disable_irq();
  tasks[currentTask].state     = STATE_SLEEPING;
  tasks[currentTask].wake_time = global_time_ms + duration_ms;
  __enable_irq();

  sys_yield();
}

// ==========================================
// 4. サーバタスク実装
// ==========================================

// --- Memory Manager ---
#define HEAP_SIZE 1024
bool heap_map[HEAP_SIZE / 32]; // 32バイト単位で32スロット

void Task_MemMgr() {
  Message msg;
  while (1) {
    if (ipc_receive(PID_MEM_MGR, &msg)) {

      if (msg.type == MSG_MEM_ALLOC_REQ) {
        int size          = msg.param1;
        int blocks_needed = (size + 31) / 32;
        int start         = -1;

        for (int i = 0; i <= (HEAP_SIZE / 32 - blocks_needed); i++) {
          bool found = true;
          for (int j = 0; j < blocks_needed; j++) {
            if (heap_map[i + j]) {
              found = false;
              i += j;
              break;
            }
          }
          if (found) {
            start = i;
            break;
          }
        }

        if (start >= 0) {
          for (int j = 0; j < blocks_needed; j++) {
            heap_map[start + j] = true;
          }
          int  addr     = start * 32;
          char addr_str[16];
          sprintf(addr_str, "%d", addr);
          // [Fix C-4] 応答は MSG_ACK を使用（従来 MSG_MEM_ALLOC_REQ を誤って返していた）
          ipc_send(msg.sender, MSG_ACK, addr, addr_str, PID_MEM_MGR);
        } else {
          ipc_send(msg.sender, MSG_ERROR, 0, "No Mem", PID_MEM_MGR);
        }
      }

      else if (msg.type == MSG_MEM_FREE_REQ) {
        int addr = msg.param1;
        if (addr < 0 || addr >= HEAP_SIZE || (addr % 32 != 0)) {
          ipc_send(msg.sender, MSG_ERROR, 0, "Bad Addr", PID_MEM_MGR);
          continue;
        }

        int index = addr / 32;
        if (!heap_map[index]) {
          ipc_send(msg.sender, MSG_ERROR, 0, "Not Alloc", PID_MEM_MGR);
          continue;
        }

        heap_map[index] = false;
        ipc_send(msg.sender, MSG_MEM_FREE_OK, addr, "Freed", PID_MEM_MGR);
      }

    } else {
      sys_yield();
    }
  }
}

// --- File Manager ---
struct FileNode {
  char name[12];
  char content[32];
  bool active;
  int  heap_addr;
};
FileNode fileSystem[5];

void Task_FileMgr() {
  Message msg;
  while (1) {
    if (ipc_receive(PID_FILE_MGR, &msg)) {

      if (msg.type == MSG_FS_CREATE) {
        bool created = false;

        for (int i = 0; i < 5; i++) {
          if (!fileSystem[i].active) {
            ipc_send(PID_MEM_MGR, MSG_MEM_ALLOC_REQ, 32, nullptr, PID_FILE_MGR);

            Message reply;
            while (!ipc_receive(PID_FILE_MGR, &reply)) {
              sys_yield();
            }

            // [Fix C-4] MSG_ACK で正常判定
            if (reply.type == MSG_ACK) {
              fileSystem[i].active    = true;
              strncpy(fileSystem[i].name, msg.payload, 11);
              fileSystem[i].name[11]  = '\0';
              memset(fileSystem[i].content, 0, 32);
              fileSystem[i].heap_addr = reply.param1;
              ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, "File Created", PID_FILE_MGR);
              created = true;
            } else {
              ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, "No Mem", PID_FILE_MGR);
            }
            break;
          }
        }

        if (!created) {
          ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, "FS Full", PID_FILE_MGR);
        }
      }

      else if (msg.type == MSG_FS_LIST) {
        ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, "--- Files ---", PID_FILE_MGR);
        for (int i = 0; i < 5; i++) {
          if (fileSystem[i].active) {
            char output[64];
            sprintf(output, "%s: %s", fileSystem[i].name, fileSystem[i].content);
            ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, output, PID_FILE_MGR);
          }
        }
      }

      else if (msg.type == MSG_FS_WRITE) {
        bool found = false;

        char temp_payload[16];
        strncpy(temp_payload, msg.payload, 15);
        temp_payload[15] = '\0';

        char* colon = strchr(temp_payload, ':');

        // --- 削除処理 ---
        if (msg.param1 == -1) {
          for (int i = 0; i < 5; i++) {
            if (fileSystem[i].active && strcmp(fileSystem[i].name, temp_payload) == 0) {
              fileSystem[i].active = false;

              ipc_send(PID_MEM_MGR, MSG_MEM_FREE_REQ, fileSystem[i].heap_addr, nullptr, PID_FILE_MGR);

              Message reply;
              while (!ipc_receive(PID_FILE_MGR, &reply)) {
                sys_yield();
              }

              ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, "File Deleted", PID_FILE_MGR);
              found = true;
              break;
            }
          }
          if (!found) {
            ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, "File Not Found", PID_FILE_MGR);
          }
          continue;
        }

        // --- 通常の書き込み処理 ---
        if (!colon) {
          ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, "Format Err", PID_FILE_MGR);
          continue;
        }

        *colon = '\0';
        char* filename = temp_payload;
        char* filedata = colon + 1;

        for (int i = 0; i < 5; i++) {
          if (fileSystem[i].active && strcmp(fileSystem[i].name, filename) == 0) {
            strncpy(fileSystem[i].content, filedata, 31);
            fileSystem[i].content[31] = '\0';
            ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, "Write OK", PID_FILE_MGR);
            found = true;
            break;
          }
        }

        if (!found) {
          ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, "File Not Found", PID_FILE_MGR);
        }
      }

    } else {
      sys_yield();
    }
  }
}

// --- Serial Driver ---
void Task_SerialDriver() {
  while (1) {
    if (Serial.available() > 0) {
      char c   = Serial.read();
      char s[2] = {c, '\0'};

      // 受信バッファが一杯の場合、送れるようになるまで yield して再試行
      while (!ipc_send(PID_SHELL, MSG_SERIAL_IN, (int)c, s, PID_DRIVER_SERIAL)) {
        sys_yield();
      }
    }

    Message msg;
    if (ipc_receive(PID_DRIVER_SERIAL, &msg)) {
      if (msg.type == MSG_SERIAL_OUT) {
        Serial.print(msg.payload);
      } else if (msg.type == MSG_SERIAL_OUT_LN) {
        Serial.println(msg.payload);
      }
    }
    sys_yield();
  }
}

// --- LED点滅タスク (PID_BLINKY) ---
bool blinky_active = false;
bool blinky_led_on = false;

void Task_Blinky() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // [Fix Blinky-r5] 絶対時刻スケジューリング方式に変更
  //
  // 旧実装の問題:
  //   sys_sleep(500) は「500ms後に READY にする」だけで、
  //   「500ms後に必ず実行される」保証がない。
  //   起床後に他タスクが実行中だとその分待たされ、誤差が毎サイクル
  //   積み重なって点滅間隔がずれていた。
  //
  // 新実装:
  //   next_toggle に「次に切り替える絶対時刻」を記録する。
  //   next_toggle += 500 で絶対時刻を加算することで、
  //   仮に今回数ms遅れても次回以降に誤差が持ち越されない。
  uint32_t next_toggle = 0;

  while (1) {
    // メッセージ処理
    Message msg;
    while (ipc_receive(PID_BLINKY, &msg)) {
      if (msg.type == MSG_LED_ON) {
        blinky_active = true;
        blinky_led_on = true;
        next_toggle   = global_time_ms + 500; // 今から500ms後が最初の切り替え
        digitalWrite(LED_BUILTIN, HIGH);      // 即座に点灯
        ipc_send(msg.sender, MSG_ACK, 0, "LED ON/BLINKING", PID_BLINKY);
      } else if (msg.type == MSG_LED_OFF) {
        blinky_active = false;
        blinky_led_on = false;
        digitalWrite(LED_BUILTIN, LOW);
        ipc_send(msg.sender, MSG_ACK, 0, "LED OFF", PID_BLINKY);
      }
    }

    if (blinky_active) {
      // (int32_t)キャストで uint32_t オーバーフロー時も正しく比較できる
      if ((int32_t)(global_time_ms - next_toggle) >= 0) {
        next_toggle += 500; // 絶対時刻で次の切り替えをスケジュール（誤差を積み重ねない）
        if (blinky_led_on) {
          digitalWrite(LED_BUILTIN, LOW);
          blinky_led_on = false;
        } else {
          digitalWrite(LED_BUILTIN, HIGH);
          blinky_led_on = true;
        }
      }
      sys_sleep(1); // 1msごとに起床して時刻チェック
    } else {
      sys_yield();
    }
  }
}

// --- 匂いセンサタスク ---
void Task_SmellSensor() {
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_SENSOR, OUTPUT);
  digitalWrite(PIN_HEATER, HIGH); // Heater Off
  digitalWrite(PIN_SENSOR, LOW);  // Sensor Pullup Off

  bool active = false;

  while (1) {
    Message msg;
    while (ipc_receive(PID_SMELL_SENSOR, &msg)) {
      if (msg.type == MSG_SS_ON) {
        active = true;
        ipc_send(msg.sender, MSG_ACK, 0, "Smell Sensor ON", PID_SMELL_SENSOR);
      } else if (msg.type == MSG_SS_OFF) {
        active = false;
        ipc_send(msg.sender, MSG_ACK, 0, "Smell Sensor OFF", PID_SMELL_SENSOR);
      }
    }

    if (active) {
      int val = 0;
      sys_sleep(237);
      digitalWrite(PIN_SENSOR, HIGH); // Sensor Pullup On
      sys_sleep(3);
      val = analogRead(PIN_OUTPUT);   // Get Sensor Voltage
      sys_sleep(2);
      digitalWrite(PIN_SENSOR, LOW);  // Sensor Pullup Off
      digitalWrite(PIN_HEATER, LOW);  // Heater On
      sys_sleep(8);
      digitalWrite(PIN_HEATER, HIGH); // Heater Off

      // [Fix M-2] ローカル変数名 msg → buf（Message msg のシャドーイング解消）
      char buf[32];
      sprintf(buf, "Smell: %d", val);
      ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, buf, PID_SMELL_SENSOR);
    }

    sys_sleep(500);
  }
}

// ==========================================
// 4.5. シェルタスク
// ==========================================

char cmdBuffer[32];
int  cmdIndex = 0;

void Task_Shell() {
  Message msg;
  ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, "\n> ", PID_SHELL);

  while (1) {
    if (ipc_receive(PID_SHELL, &msg)) {
      if (msg.type == MSG_SERIAL_IN) {
        char c    = (char)msg.param1;
        char s[2] = {c, '\0'};
        ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, s, PID_SHELL);

        if (c == '\r' || c == '\n') {
          ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "", PID_SHELL);
          cmdBuffer[cmdIndex] = '\0';

          if (strncmp(cmdBuffer, "ls", 2) == 0) {
            ipc_send(PID_FILE_MGR, MSG_FS_LIST, 0, "", PID_SHELL);
          }
          else if (strncmp(cmdBuffer, "touch ", 6) == 0) {
            ipc_send(PID_FILE_MGR, MSG_FS_CREATE, 0, &cmdBuffer[6], PID_SHELL);
          }
          else if (strncmp(cmdBuffer, "write ", 6) == 0) {
            char* args  = &cmdBuffer[6];
            char* colon = strchr(args, ':');
            if (colon) {
              ipc_send(PID_FILE_MGR, MSG_FS_WRITE, 0, args, PID_SHELL);
            } else {
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "Usage: write <fname>:<data>", PID_SHELL);
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT,    0, "> ", PID_SHELL);
            }
          }
          else if (strncmp(cmdBuffer, "blink ", 6) == 0) {
            if (strncmp(&cmdBuffer[6], "on", 2) == 0) {
              ipc_send(PID_BLINKY, MSG_LED_ON, 0, "", PID_SHELL);
            } else if (strncmp(&cmdBuffer[6], "off", 3) == 0) {
              ipc_send(PID_BLINKY, MSG_LED_OFF, 0, "", PID_SHELL);
            } else {
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "Usage: blink (on|off)", PID_SHELL);
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT,    0, "> ", PID_SHELL);
            }
          }
          else if (strncmp(cmdBuffer, "sleep ", 6) == 0) {
            int duration = atoi(&cmdBuffer[6]);
            if (duration > 0) {
              // [Fix M-1] ローカル変数名 msg → buf（Message msg のシャドーイング解消）
              char buf[32];
              sprintf(buf, "Sleeping %d ms...", duration);
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, buf, PID_SHELL);
              sys_sleep(duration);
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "Woken up", PID_SHELL);
            } else {
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "Usage: sleep <ms>", PID_SHELL);
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT,    0, "> ", PID_SHELL);
            }
          }
          else if (strncmp(cmdBuffer, "ps", 2) == 0) {
            Serial.println("--- Task Status ---");
            Serial.println("PID | Task Name | Used/Total | State");
            Serial.println("-----------------------------------");

            for (int i = 0; i < PID_MAX; i++) {
              char     output[64];
              uint32_t current_sp  = (uint32_t)tasks[i].sp;
              uint32_t base_addr   = (uint32_t)tasks[i].stack_base;
              uint32_t used_bytes  = base_addr - current_sp;
              uint32_t total_bytes = STACK_SIZE * sizeof(uint32_t);
              const char* state_str = (tasks[i].state == STATE_READY) ? "RDY" : "SLP";

              sprintf(output, " %d  | %-9s | %lu / %lu | %s",
                      i, TASK_NAMES[i],
                      (unsigned long)used_bytes,
                      (unsigned long)total_bytes,
                      state_str);
              Serial.println(output);
            }
            ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, "> ", PID_SHELL);
          }
          else if (strncmp(cmdBuffer, "heap", 4) == 0) {
            char line[48];
            strcpy(line, "Heap: ");
            for (int i = 0; i < HEAP_SIZE / 32; i++) {
              strcat(line, heap_map[i] ? "#" : ".");
            }
            ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, line, PID_SHELL);
            ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT,    0, "> ", PID_SHELL);
          }
          else if (strncmp(cmdBuffer, "rm ", 3) == 0) {
            ipc_send(PID_FILE_MGR, MSG_FS_WRITE, -1, &cmdBuffer[3], PID_SHELL);
          }
          else if (strncmp(cmdBuffer, "ss ", 3) == 0) {
            if (strncmp(&cmdBuffer[3], "on", 2) == 0) {
              ipc_send(PID_SMELL_SENSOR, MSG_SS_ON, 0, "", PID_SHELL);
            } else if (strncmp(&cmdBuffer[3], "off", 3) == 0) {
              ipc_send(PID_SMELL_SENSOR, MSG_SS_OFF, 0, "", PID_SHELL);
            } else {
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "Usage: ss (on|off)", PID_SHELL);
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT,    0, "> ", PID_SHELL);
            }
          }
          // [Fix M-3] cmdIndex=0 は全コマンド共通でここに配置（ss コマンド後の漏れを修正）
          cmdIndex = 0;

        } else if (cmdIndex < 31) {
          cmdBuffer[cmdIndex++] = c;
        }
      }
      else if (msg.sender == PID_FILE_MGR || msg.sender == PID_MEM_MGR || msg.sender == PID_BLINKY) {
        ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, msg.payload, PID_SHELL);
        ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT,    0, "> ", PID_SHELL);
      }
    } else {
      sys_yield();
    }
  }
}

// --- Idle Task ---
// [Fix C-2] tickless ロジックを廃止。固定1ms tick (SysTick_Config) と統一したため
//           Task_Idle は __WFI() で待つだけ。
void Task_Idle() {
  while (1) {
    __WFI();
  }
}

// ==========================================
// 5. カーネルコア (ASM Context Switch)
// ==========================================

extern "C" void PendSV_Handler(void) __attribute__((naked));

void PendSV_Handler(void) {
  __asm volatile (
    " cpsid i                 \n"
    " mrs r0, psp             \n"
    " isb                     \n"
    " ldr r3, =currentTask    \n"
    " ldr r2, [r3]            \n"
    " stmdb r0!, {r4-r11}     \n"
    " ldr r1, =tasks          \n"
    " lsl r2, r2, #4          \n" // TCB 16B
    " add r1, r1, r2          \n"
    " str r0, [r1]            \n"
    " ldr r3, =nextTask       \n"
    " ldr r2, [r3]            \n"
    " ldr r3, =currentTask    \n"
    " str r2, [r3]            \n"
    " ldr r1, =tasks          \n"
    " lsl r2, r2, #4          \n" // TCB 16B
    " add r1, r1, r2          \n"
    " ldr r0, [r1]            \n"
    " ldmia r0!, {r4-r11}     \n"
    " msr psp, r0             \n"
    " isb                     \n"
    " cpsie i                 \n"
    " bx lr                   \n"
    " .align 4                \n"
  );
}

// ==========================================
// SysTick 割り込みハンドラ
// ==========================================
extern "C" void SysTick_Handler(void) {
  // 固定 1ms tick なので elapsed は常に 1
  global_time_ms++;

  // [Fix C-1] 全スリープタスクの起床チェックを全件走査に変更
  //   旧実装: do-while で最初の READY タスクが見つかったら break していたため
  //           複数タスクが同時に wake_time を迎えても1つしか起床させられなかった。
  for (int i = 1; i < PID_MAX; i++) {
    if (tasks[i].state == STATE_SLEEPING &&
        global_time_ms >= tasks[i].wake_time) {
      tasks[i].state = STATE_READY;
    }
  }

  // 次に実行するタスクをラウンドロビンで選択
  int next = currentTask;
  do {
    next++;
    if (next >= PID_MAX) next = 1;
    if (tasks[next].state == STATE_READY) break;
  } while (next != currentTask);

  nextTask = next;
  SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

// ==========================================
// 6. 初期化
// ==========================================

void task_create(int pid, void (*taskFunc)()) {
  uint32_t* sp = &taskStacks[pid][STACK_SIZE - 1];

  memset(&tasks[pid], 0, sizeof(TCB));

  tasks[pid].stack_base = sp;

  *(--sp) = 0x01000000;                                   // xPSR
  *(--sp) = (uint32_t)taskFunc;                           // PC
  *(--sp) = 0xFFFFFFFD;                                   // LR (EXC_RETURN)
  *(--sp) = 0; *(--sp) = 0; *(--sp) = 0; *(--sp) = 0; *(--sp) = 0; // R12,R3,R2,R1,R0
  for (int i = 0; i < 8; i++) *(--sp) = 0;               // R11-R4

  tasks[pid].sp         = sp;
  tasks[pid].state      = STATE_READY;
  tasks[pid].wake_time  = 0;
}

int get_free_heap_bytes() {
  int free_blocks = 0;
  for (int i = 0; i < HEAP_SIZE / 32; i++) {
    if (!heap_map[i]) free_blocks++;
  }
  return free_blocks * 32;
}

// [Fix C-3] uint32_t アンダーフロー対策済み
//   旧実装: wake_time < global_time_ms のとき uint32_t 減算がラップアラウンドして
//           巨大な delay 値になり、SysTick が誤った遅延で設定されていた。
//   現在: Task_Idle が __WFI() のみになったため本関数は使われないが、
//         将来の tickless 復活に備えて正しい実装で残しておく。
uint32_t get_next_wake_delay() {
  uint32_t min_delay = 0xFFFFFFFF;
  uint32_t now       = global_time_ms;

  for (int i = 1; i < PID_MAX; i++) {
    if (tasks[i].state == STATE_SLEEPING) {
      if (tasks[i].wake_time <= now) {
        return 0; // 即時起床が必要なタスクあり
      }
      uint32_t delay = tasks[i].wake_time - now;
      if (delay < min_delay) min_delay = delay;
    }
  }
  return (min_delay == 0xFFFFFFFF) ? 0 : min_delay;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  for (int i = 0; i < PID_MAX; i++) {
    mailboxes[i].head  = 0;
    mailboxes[i].tail  = 0;
    mailboxes[i].count = 0;
  }

  Serial.println("Microkernel for Arduino UNO R4 Copyright YUKARI Semiconductor Devices.");
  Serial.println("Version: 20260317_fixed_r5");
  Serial.println("Commands: ls, touch <n>, write <fname>:<data>, rm <fname>, ps, heap,");
  Serial.println("          blink (on|off), sleep <ms>, ss (on|off)");

  task_create(PID_IDLE,         Task_Idle);
  task_create(PID_MEM_MGR,      Task_MemMgr);
  task_create(PID_FILE_MGR,     Task_FileMgr);
  task_create(PID_DRIVER_SERIAL,Task_SerialDriver);
  task_create(PID_SHELL,        Task_Shell);
  task_create(PID_BLINKY,       Task_Blinky);
  task_create(PID_SMELL_SENSOR, Task_SmellSensor);

  NVIC_SetPriority(PendSV_IRQn, 0xFF);

  __set_PSP((uint32_t)tasks[PID_IDLE].sp);
  __set_CONTROL(0x02);
  __ISB();

  currentTask = PID_IDLE;

  // [Fix C-2] 固定 1ms tick に統一。Task_Idle 内の tickless 再設定は廃止。
  SysTick_Config(SystemCoreClock / 1000);

  char mem_msg[32];
  sprintf(mem_msg, "Free Heap: %d bytes", get_free_heap_bytes());
  Serial.println(mem_msg);

  Serial.println("YUKARI OS Booted!");

  Task_Idle(); // 無限ループに突入
}

void loop() {
  // ここには到達しない
}