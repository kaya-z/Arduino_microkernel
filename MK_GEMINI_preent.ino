/*
 * Microkernel for Arduino UNO R4
 * Copyright YUKARI Semiconductor Devices.
 * * Version: 20251214_04
 * * Update Log:
 * - Fix: Solved 'ps' output corruption by adding ANSI escape sequences to clear the command line 
 * before printing the static 'ps' table.
 * - Fix: Delayed input echo until command is submitted (from 20251214_03).
 * - Fix: Added retry logic to Task_SerialDriver (from 20251214_01).
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
  MSG_ERROR,
  MSG_MEM_ALLOC_REQ,
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
bool heap_map[HEAP_SIZE / 32]; 

void Task_MemMgr() {
  Message msg;
  while (1) {
    if (ipc_receive(PID_MEM_MGR, &msg)) {
      if (msg.type == MSG_MEM_ALLOC_REQ) {
        ipc_send(msg.sender, MSG_ACK, 0, "ALLOC_OK", PID_MEM_MGR);
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
            fileSystem[i].active = true;
            strncpy(fileSystem[i].name, msg.payload, 11);
            memset(fileSystem[i].content, 0, 32);
            ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, "File Created", PID_FILE_MGR);
            created = true;
            break;
          }
        }
        if (!created) ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, "FS Full", PID_FILE_MGR);
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
        // write <filename>:<data>
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
        
        for(int i = 0; i < 5; i++){
            if(fileSystem[i].active && strcmp(fileSystem[i].name, filename) == 0){
                strncpy(fileSystem[i].content, filedata, 31);
                fileSystem[i].content[31] = '\0';
                ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, "Write OK", PID_FILE_MGR);
                found = true;
                break;
            }
        }
        if (!found) ipc_send(msg.sender, MSG_SERIAL_OUT_LN, 0, "File Not Found", PID_FILE_MGR);
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
      
      // 受信バッファが一杯の場合、送れるようになるまで待機(yield)して再試行する
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
  led_on = false;

  while (1) {
    // 1. メッセージ処理
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

    // 2. 点滅処理（3000ms ON / 1000ms OFF）
    if (blink_active) {
      // 点灯
      digitalWrite(LED_BUILTIN, HIGH);
      led_on = true;
      sys_sleep(3000);

      if (!blink_active) {
        digitalWrite(LED_BUILTIN, LOW);
        led_on = false;
        continue;
      }

      // 消灯
      digitalWrite(LED_BUILTIN, LOW);
      led_on = false;
      sys_sleep(1000);

    } else {
      // 非アクティブ時は確実に消灯
      if (led_on) {
        digitalWrite(LED_BUILTIN, LOW);
        led_on = false;
      }
      sys_yield();
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
        
        // 文字入力時のエコーバックは削除済み

        if (c == '\r' || c == '\n') {
          // 【修正ポイント2】コマンドを実行する前に、ホストのターミナル上の入力行を消去する
          // キャリッジリターン(\r)でカーソルを行頭に戻し、ANSIエスケープシーケンス(\x1b[K)でカーソル位置から行末までを消去する
          ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, "\r\x1b[K", PID_SHELL);
          
          // 新しいプロンプトを出力する前に、入力されたコマンドを再度エコーバックして表示する
          ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, "> ", PID_SHELL);
          ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, cmdBuffer, PID_SHELL);
          
          // コマンド実行前の改行
          ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "", PID_SHELL);
          
          cmdBuffer[cmdIndex] = '\0';
          
          bool executed = false;
          
          if (strncmp(cmdBuffer, "ls", 2) == 0) {
             ipc_send(PID_FILE_MGR, MSG_FS_LIST, 0, "", PID_SHELL);
             executed = true;
          } 
          else if (strncmp(cmdBuffer, "touch ", 6) == 0) {
             ipc_send(PID_FILE_MGR, MSG_FS_CREATE, 0, &cmdBuffer[6], PID_SHELL);
             executed = true;
          }
          else if (strncmp(cmdBuffer, "write ", 6) == 0) {
             char* args = &cmdBuffer[6];
             char* colon = strchr(args, ':');
             if (colon) {
                 ipc_send(PID_FILE_MGR, MSG_FS_WRITE, 0, args, PID_SHELL);
             } else {
                 ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "Usage: write <fname>:<data>", PID_SHELL);
             }
             executed = true;
          }
          else if (strncmp(cmdBuffer, "blink ", 6) == 0) {
             if (strncmp(&cmdBuffer[6], "on", 2) == 0) {
                 ipc_send(PID_BLINKY, MSG_LED_ON, 0, "", PID_SHELL);
             } else if (strncmp(&cmdBuffer[6], "off", 3) == 0) {
                 ipc_send(PID_BLINKY, MSG_LED_OFF, 0, "", PID_SHELL);
             } else {
                 ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "Usage: blink (on|off)", PID_SHELL);
             }
             executed = true;
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
            }
            executed = true;
          }
          else if (strncmp(cmdBuffer, "ps", 2) == 0) {
             // psコマンドは Serial.println で直接出力する
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
             // 処理完了後、プロンプトを出力する (次のプロンプトはIPCの応答を待たずに、ここで直接出す)
             ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, "> ", PID_SHELL); 
             executed = true;
          }
          
          if (!executed) {
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT_LN, 0, "Unknown command", PID_SHELL);
              ipc_send(PID_DRIVER_SERIAL, MSG_SERIAL_OUT, 0, "> ", PID_SHELL); 
          }

          cmdIndex = 0;
          memset(cmdBuffer, 0, sizeof(cmdBuffer)); // バッファもクリア

        } 
        else if (cmdIndex < 31) {
          cmdBuffer[cmdIndex++] = c;
        }
      }
      // 【修正ポイント3】IPCで応答が来た場合、プロンプトを再出力する前に、入力行を再度クリアする
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
void Task_Idle() {
  while(1) {
    __asm volatile("nop");
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

extern "C" void SysTick_Handler(void) {
  global_time_ms++;

  for (int i = 1; i < PID_MAX; i++) {
    if (tasks[i].state == STATE_SLEEPING && global_time_ms >= tasks[i].wake_time) {
      tasks[i].state = STATE_READY;
    }
  }

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

  *(--sp) = 0x01000000; 
  *(--sp) = (uint32_t)taskFunc; 
  *(--sp) = 0xFFFFFFFD; 
  *(--sp) = 0; *(--sp) = 0; *(--sp) = 0; *(--sp) = 0; *(--sp) = 0; 
  for (int i = 0; i < 8; i++) *(--sp) = 0; 
  
  tasks[pid].sp = sp;
  tasks[pid].state = STATE_READY; 
  tasks[pid].wake_time = 0;
}

void setup() {
  Serial.begin(115200);
  while(!Serial); 
  
  for (int i = 0; i < PID_MAX; i++) {
    mailboxes[i].head = 0;
    mailboxes[i].tail = 0;
    mailboxes[i].count = 0;
  }

  Serial.println("Microkernel for Arduino UNO R4 Copyright YUKARI Semiconductor Devices.");
  Serial.println("Version: 20251214_04");
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
  
  SysTick_Config(SystemCoreClock / 1000); 
  
  Task_Idle();
}

void loop() {
  // ここには到達しない
}