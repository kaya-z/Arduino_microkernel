/*
 * Microkernel for Arduino UNO R4
 * Copyright YUKARI Semiconductor Devices.
 * * Version: 20251214_01
 * * Update Log:
 * - Fix: Added retry logic to Task_SerialDriver to prevent input character loss during fast typing.
 * - Feature: File Write 'write <filename>:<data>'
 * - Feature: Stable L-Chika (Blinky) with sys_sleep
 * - Config: Stack Size 512 (Stable)
 */

#include <Arduino.h>

// ==========================================
// 1. システム設定 & 定数
// ==========================================

#define STACK_SIZE 512  // スタックサイズ (uint32_t単位 = 2048 Bytes)
#define MAX_TASKS  6    

// プロセスID
enum PID {
  PID_IDLE = 0, 
  PID_MEM_MGR,
  PID_FILE_MGR,
  PID_DRIVER_SERIAL,
  PID_SHELL,
  PID_BLINKY,   
  PID_MAX       // PID_MAX = 6
};

// タスク名配列
const char* TASK_NAMES[PID_MAX] = {
    "IDLE",
    "MEM_MGR",
    "FILE_MGR",
    "SERIAL_DRV",
    "SHELL",
    "BLINKY" 
};

// メッセージタイプ
enum MsgType {
  MSG_NONE = 0,
  MSG_ACK,
  MSG_ERROR,           // ← これが既存のエラー通知
  MSG_MEM_ALLOC_REQ,
  MSG_MEM_FREE_REQ,    // ← 追加
  MSG_MEM_FREE_OK,     // ← 追加
  MSG_FS_CREATE,
  MSG_FS_WRITE,
  MSG_FS_LIST,
  MSG_SERIAL_IN,
  MSG_SERIAL_OUT,
  MSG_SERIAL_OUT_LN,
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
  
  __disable_irq(); // 割り込み禁止
  
  Mailbox* mb = &mailboxes[receiver];
  if (mb->count < QUEUE_SIZE) {
    Message* msg = &mb->buffer[mb->tail];
    msg->sender = sender;
    msg->receiver = receiver;
    msg->type = type;
    msg->param1 = param1;
    memset(msg->payload, 0, 16);
    if (payload) strncpy(msg->payload, payload, 15); 

    mb->tail = (mb->tail + 1) % QUEUE_SIZE;
    mb->count++;
    success = true;
  }
  
  __enable_irq(); // 割り込み許可
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

// タスクの状態
enum TaskState { 
    STATE_READY = 0,    // 実行可能
    STATE_SLEEPING      // スリープ中
};

struct TCB {
  uint32_t* sp;
  uint32_t* stack_base; 
  TaskState state;       // タスク状態
  uint32_t wake_time;    // 起床時刻
}; // TCBサイズは16バイト

TCB tasks[PID_MAX];
uint32_t taskStacks[PID_MAX][STACK_SIZE];

volatile int currentTask = 0;
volatile int nextTask = 0;

volatile uint32_t global_time_ms = 0;

// OSシステムコール: タスクをスリープ状態にする
void sys_sleep(uint32_t duration_ms) {
    __disable_irq();

    // 現在のタスクの状態と起床時刻を設定
    tasks[currentTask].state = STATE_SLEEPING;
    tasks[currentTask].wake_time = global_time_ms + duration_ms;

    __enable_irq();
    
    // PendSVをトリガーしてコンテキストスイッチを実行し、CPUを解放
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
        int size = msg.param1;
        int blocks_needed = (size + 31) / 32;
        int start = -1;

        for (int i = 0; i <= (HEAP_SIZE / 32 - blocks_needed); i++) {
          bool found = true;
          for (int j = 0; j < blocks_needed; j++) {
            if (heap_map[i + j]) {
              found = false;
              i += j;  // スキップ
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
          int addr = start * 32;
          char addr_str[16];
          sprintf(addr_str, "%d", addr);
          ipc_send(msg.sender, MSG_MEM_ALLOC_REQ, addr, addr_str, PID_MEM_MGR);
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

        // 単純に1ブロックだけ解放（将来的にサイズ指定も可）
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
  int heap_addr; // ← これを追加！  
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
            // メモリ確保を依頼（32バイト）
            ipc_send(PID_MEM_MGR, MSG_MEM_ALLOC_REQ, 32, nullptr, PID_FILE_MGR);

            // 応答を待つ
            Message reply;
            while (!ipc_receive(PID_FILE_MGR, &reply)) {
              sys_yield();
            }

            if (reply.type == MSG_MEM_ALLOC_REQ) {
              fileSystem[i].active = true;
              strncpy(fileSystem[i].name, msg.payload, 11);
              memset(fileSystem[i].content, 0, 32);
              fileSystem[i].heap_addr = reply.param1;  // ← 確保したアドレスを記録
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
      char c = Serial.read();
      char s[2] = {c, '\0'};
      
      // 【修正】再試行ロジック
      // 受信バッファが一杯の場合、送れるようになるまで待機(yield)して再試行する
      // これにより、高速入力時の取りこぼしを防ぐ
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
void Task_Blinky() {
  pinMode(LED_BUILTIN, OUTPUT);
  static bool blink_active = false;
  static bool led_on = false;

  digitalWrite(LED_BUILTIN, LOW);

  while (1) {
    // メッセージ処理
    Message msg;
    while (ipc_receive(PID_BLINKY, &msg)) {
      if (msg.type == MSG_LED_ON) {
        blink_active = true;
        ipc_send(msg.sender, MSG_ACK, 0, "LED ON/BLINKING", PID_BLINKY);
      } else if (msg.type == MSG_LED_OFF) {
        blink_active = false;
        digitalWrite(LED_BUILTIN, LOW);
        led_on = false;
        ipc_send(msg.sender, MSG_ACK, 0, "LED OFF", PID_BLINKY);
      }
    }

    sys_yield();

    if (blink_active) {
      if (!led_on) {
        digitalWrite(LED_BUILTIN, HIGH);
        led_on = true;
        sys_sleep(2000);
      } else {
        digitalWrite(LED_BUILTIN, LOW);
        led_on = false;
        sys_sleep(2000);
        digitalWrite(LED_BUILTIN, LOW);  
      }
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      led_on = false;
    }
  }
}


// ==========================================
// 4.5. シェルタスク
// ==========================================

// --- Shell (User Task) ---
char cmdBuffer[32];
int cmdIndex = 0;

void Task_Shell() {
  Message msg;
  ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, "\n> ", PID_SHELL);

  while (1) {
    if (ipc_receive(PID_SHELL, &msg)) {
      if (msg.type == MSG_SERIAL_IN) {
        char c = (char)msg.param1;
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
             char* args = &cmdBuffer[6];
             char* colon = strchr(args, ':');
             if (colon) {
                 ipc_send(PID_FILE_MGR, MSG_FS_WRITE, 0, args, PID_SHELL);
             } else {
                 ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "Usage: write <fname>:<data>", PID_SHELL);
                 ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, "> ", PID_SHELL);
             }
          }
          else if (strncmp(cmdBuffer, "blink ", 6) == 0) {
             if (strncmp(&cmdBuffer[6], "on", 2) == 0) {
                 ipc_send(PID_BLINKY, MSG_LED_ON, 0, "", PID_SHELL);
             } else if (strncmp(&cmdBuffer[6], "off", 3) == 0) {
                 ipc_send(PID_BLINKY, MSG_LED_OFF, 0, "", PID_SHELL);
             } else {
                 ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "Usage: blink (on|off)", PID_SHELL);
                 ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, "> ", PID_SHELL);
             }
          }
          else if (strncmp(cmdBuffer, "sleep ", 6) == 0) {
            int duration = atoi(&cmdBuffer[6]);
            if (duration > 0) {
              char msg[32];
              sprintf(msg, "Sleeping %d ms...", duration);
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, msg, PID_SHELL);
              sys_sleep(duration); 
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "Woken up", PID_SHELL);
            } else {
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "Usage: sleep <ms>", PID_SHELL);
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, "> ", PID_SHELL);
            }
          }
          else if (strncmp(cmdBuffer, "ps", 2) == 0) {
             Serial.println("--- Task Status ---");
             Serial.println("PID | Task Name | Used/Total | State");
             Serial.println("-----------------------------------");
             
             for(int i = 1; i < PID_MAX; i++) { 
                 char output[64]; 
                 uint32_t current_sp = (uint32_t)tasks[i].sp;
                 uint32_t base_addr = (uint32_t)tasks[i].stack_base;
                 uint32_t used_bytes = base_addr - current_sp;
                 uint32_t total_bytes = STACK_SIZE * sizeof(uint32_t); 
                 const char* state_str = (tasks[i].state == STATE_READY) ? "RDY" : "SLP";

                 sprintf(output, " %d  | %-9s | %lu / %lu | %s", 
                         i, TASK_NAMES[i], (unsigned long)used_bytes, (unsigned long)total_bytes, state_str);
                 Serial.println(output);
             }
             ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, "> ", PID_SHELL); 
          }
          
          cmdIndex = 0;
        } 
        else if (cmdIndex < 31) {
          cmdBuffer[cmdIndex++] = c;
        }
      }
      else if (msg.sender == PID_FILE_MGR || msg.sender == PID_MEM_MGR || msg.sender == PID_BLINKY) {
          ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, msg.payload, PID_SHELL);
          ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, "> ", PID_SHELL);
      }
    } else {
      sys_yield();
    }
  }
}

// ------------------------------------------

// --- Idle Task ---
// ==========================================
// 修正: Idleタスクをtickless対応に
// ==========================================
void Task_Idle() {
  while (1) {
    uint32_t delay = get_next_wake_delay();

    if (delay == 0) {
      __WFI();  // 他タスクがREADYならすぐ復帰
    } else {
      // SysTickをdelay ms後に1回だけ発火するよう設定
      SysTick->CTRL = 0; // 停止
      SysTick->LOAD = (SystemCoreClock / 1000) * delay - 1;
      SysTick->VAL = 0;
      SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                      SysTick_CTRL_TICKINT_Msk |
                      SysTick_CTRL_ENABLE_Msk;

      __WFI();  // スリープ
    }
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
// 修正: SysTick割り込みで経過時間を加算
// ==========================================
extern "C" void SysTick_Handler(void) {
  uint32_t elapsed = (SysTick->LOAD + 1) / (SystemCoreClock / 1000);
  global_time_ms += elapsed;

  int startTask = currentTask;
  int next = startTask;

  do {
    next++;
    if (next >= PID_MAX) next = 1;

    if (tasks[next].state == STATE_SLEEPING &&
        global_time_ms >= tasks[next].wake_time) {
      tasks[next].state = STATE_READY;
    }

    if (tasks[next].state == STATE_READY) break;

  } while (next != startTask);

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

  *(--sp) = 0x01000000; 
  *(--sp) = (uint32_t)taskFunc; 
  *(--sp) = 0xFFFFFFFD; 
  *(--sp) = 0; *(--sp) = 0; *(--sp) = 0; *(--sp) = 0; *(--sp) = 0; 
  for (int i = 0; i < 8; i++) *(--sp) = 0; 
  
  tasks[pid].sp = sp;
  tasks[pid].state = STATE_READY; 
  tasks[pid].wake_time = 0;
}

int get_free_heap_bytes() {
  int free_blocks = 0;
  for (int i = 0; i < HEAP_SIZE / 32; i++) {
    if (!heap_map[i]) free_blocks++;
  }
  return free_blocks * 32;
}

// ==========================================
// 追加: 次のスリープ解除までの最短時間を取得
// ==========================================
uint32_t get_next_wake_delay() {
  uint32_t min_delay = 0xFFFFFFFF;
  for (int i = 1; i < PID_MAX; i++) {
    if (tasks[i].state == STATE_SLEEPING) {
      uint32_t delay = tasks[i].wake_time - global_time_ms;
      if (delay < min_delay) min_delay = delay;
    }
  }
  return (min_delay == 0xFFFFFFFF) ? 0 : min_delay;
}

// ==========================================
// 修正: setup()からSysTick_Config()を削除
// ==========================================
void setup() {
  Serial.begin(115200);
  while(!Serial); 
  
  for (int i = 0; i < PID_MAX; i++) {
    mailboxes[i].head = 0;
    mailboxes[i].tail = 0;
    mailboxes[i].count = 0;
  }

  Serial.println("Microkernel for Arduino UNO R4 Copyright YUKARI Semiconductor Devices.");
  Serial.println("Version: 20251214_01");
  Serial.println("Commands: ls, touch <name>, write <fname>:<data>, ps, blink (on|off), sleep <ms>");

  task_create(PID_IDLE, Task_Idle);
  task_create(PID_MEM_MGR, Task_MemMgr);
  task_create(PID_FILE_MGR, Task_FileMgr);
  task_create(PID_DRIVER_SERIAL, Task_SerialDriver);
  task_create(PID_SHELL, Task_Shell);
  task_create(PID_BLINKY, Task_Blinky); 

  NVIC_SetPriority(PendSV_IRQn, 0xFF); 
  
  __set_PSP((uint32_t)tasks[PID_IDLE].sp);
  __set_CONTROL(0x02); 
  __ISB();

  currentTask = PID_IDLE;

  // SysTick_Config() は不要（ticklessで動的設定するため）

    SysTick_Config(SystemCoreClock / 1000); 

  // 空きメモリ表示 
  char mem_msg[32];
  sprintf(mem_msg, "Free Heap: %d bytes", get_free_heap_bytes()); 
  Serial.println(mem_msg);

  Serial.println("YUKARI OS Booted!");

  Task_Idle();  // 無限ループに突入
}


void loop() {
  // ここには到達しない
}