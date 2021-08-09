#include <Arduino.h>
#include <math.h>

#include <driver/adc.h>
#include <driver/dac.h>

#define MODE_ANA

#define TIMER_FAST_NUMBER 0
#define TIMER_PRESCALER 80
#define TIMER_FAST_INTERVAL 1*104  // in microseconds, 1000 = 1Hz
#define ANA_READ_TASK_ESP_CORE 1

#define USE_DOUBLE_AMPLTIUDE

#define PWM_FREQ 50000

hw_timer_t * g_FastTimerhandle = NULL;
SemaphoreHandle_t g_FastTimerSemaphore;
SemaphoreHandle_t g_SignalSendSemaphore;
SemaphoreHandle_t g_GlobalVariablesAccessSemaphore;

QueueHandle_t g_FastTimerQueue;
# define TIMER_FAST_QUEUE_LEN 20

TaskHandle_t g_SignalSendTaskHandle;

portMUX_TYPE g_FastTimerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_MainMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_LoopMux = portMUX_INITIALIZER_UNLOCKED;

volatile long g_inQueue = 0;
volatile int g_inQueueMax = 0;
//volatile unsigned long g_FastAnaReadBytesRead = 0;
volatile long g_SenderLoopDurationMax = 0;
volatile long g_SenderLoopDuration = 0;
volatile long g_SenderLoopDurationMin = 9999;

volatile unsigned long g_FastTimerCount = 0;
volatile unsigned long g_FastLoopCount = 0;
//volatile unsigned long g_delay = 0;

volatile long g_writeTime = 0;
volatile int g_delay = 0;
volatile int g_Qfull = 0;

const int8_t g_sigcode[] = {1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1}; // ORIGINAL
//const int8_t g_sigcode[] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0};  //motor driver friendly signal
//const int8_t g_sigcode[] = {1, 1, 1, 0, 0, 0, -1, -1, -1, 0, 0, 0, 1, 1, 1, 0, 0, 0, -1, -1, -1, 0, 0, 0};
//const int8_t g_sigcode[] = {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1};
//const int8_t g_sigcode[] = {1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1};
//const int8_t g_sigcode[] = {1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1};
//const int8_t g_sigcode[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//const int8_t g_sigcode[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

// const int8_t g_sigcode[] = {1, 1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1}; // pseudonoise5_nrz

volatile boolean g_enableSender = true;
volatile int g_SentStep = 0;

#define PIN_ESP_SENDER_PERIMETER_ENABLE 16
#define PIN_ESP_SENDER_PERIMETER_IN1 25
#define PIN_ESP_SENDER_PERIMETER_IN2 26


#define LEDCHANNEL 0

void SendCode()
{
  unsigned long startSend = micros();

  if (g_enableSender)
  {
    if (g_sigcode[g_SentStep] == 1)
    {
      digitalWrite(PIN_ESP_SENDER_PERIMETER_IN1, LOW);
#ifdef USE_DOUBLE_AMPLTIUDE
      digitalWrite(PIN_ESP_SENDER_PERIMETER_IN2, HIGH);
#endif
      digitalWrite(PIN_ESP_SENDER_PERIMETER_ENABLE, HIGH);
    }
    else if (g_sigcode[g_SentStep] == -1)
    {
      digitalWrite(PIN_ESP_SENDER_PERIMETER_IN1, HIGH);
      digitalWrite(PIN_ESP_SENDER_PERIMETER_IN2, LOW);
      digitalWrite(PIN_ESP_SENDER_PERIMETER_ENABLE, HIGH);
    }
    else
    {
      digitalWrite(PIN_ESP_SENDER_PERIMETER_ENABLE, LOW);
    }
    g_SentStep = g_SentStep + 1;
    if (g_SentStep == sizeof(g_sigcode))
    {
      g_SentStep = 0;
    }
  }
  else
  {
    digitalWrite(PIN_ESP_SENDER_PERIMETER_ENABLE, LOW);
  }

  portENTER_CRITICAL_SAFE(&g_LoopMux);
  xSemaphoreTake(g_GlobalVariablesAccessSemaphore, portMAX_DELAY);
  g_writeTime = g_writeTime + micros()- startSend;
  xSemaphoreGive(g_GlobalVariablesAccessSemaphore);
  portEXIT_CRITICAL_SAFE(&g_LoopMux);
}


void SendCodeAna()
{
  unsigned long startSend = micros();
  static int lastStep = 0;

  if (g_enableSender)
  {
//    Serial.print(g_sigcode[g_SentStep]);
    if (g_sigcode[g_SentStep] != g_sigcode[lastStep])     // AVOID SENDING AGAIN IF VALUE SAME
    {
      if (g_sigcode[g_SentStep] == 1)
      {
  //      ledcWrite(LEDCHANNEL, 4096);
        dac_output_voltage(DAC_CHANNEL_1, 255);
      }
      else if (g_sigcode[g_SentStep] == -1)
      {
  //      ledcWrite(LEDCHANNEL, 0);
        dac_output_voltage(DAC_CHANNEL_1, 0);
      }
      else
      {
  //      ledcWrite(LEDCHANNEL, 2048);
        dac_output_voltage(DAC_CHANNEL_1, 128);
      }
    }

    lastStep = g_SentStep;
    g_SentStep = g_SentStep + 1;
    if (g_SentStep == sizeof(g_sigcode))
    {
      g_SentStep = 0;
    }
  }
  else
  {
//      ledcWrite(LEDCHANNEL, 0);
      dac_output_voltage(DAC_CHANNEL_1, 0);
  }

  portENTER_CRITICAL_SAFE(&g_LoopMux);
  xSemaphoreTake(g_GlobalVariablesAccessSemaphore, portMAX_DELAY);
  g_writeTime = g_writeTime + micros()- startSend;
  xSemaphoreGive(g_GlobalVariablesAccessSemaphore);
  portEXIT_CRITICAL_SAFE(&g_LoopMux);
}

//---------------
// Timer
//---------------

ICACHE_RAM_ATTR void FastSendTimerISR(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;     // We have not woken a task at the start of the ISR. 
  BaseType_t QueueReturn;
  byte Message = 1;
  portENTER_CRITICAL_ISR(&g_FastTimerMux);
  g_FastTimerCount = g_FastTimerCount + 1;
//  xSemaphoreGive(g_FastTimerSemaphore);  // IF KEPT !!!
  QueueReturn = xQueueSendToBackFromISR( g_FastTimerQueue, &Message, &xHigherPriorityTaskWoken );
  if (QueueReturn != pdPASS) {
    g_Qfull = g_Qfull + 1;
  }
  if( xHigherPriorityTaskWoken )
 	{
  	portYIELD_FROM_ISR ();
 	}
  portEXIT_CRITICAL_ISR(&g_FastTimerMux);
}

void InitFastTimer(void)
{
    // Fast timer setup
  g_FastTimerSemaphore = xSemaphoreCreateMutex();

  g_FastTimerhandle = timerBegin(TIMER_FAST_NUMBER, TIMER_PRESCALER, true);

  timerAttachInterrupt(g_FastTimerhandle, &FastSendTimerISR, true);
  timerAlarmWrite(g_FastTimerhandle, TIMER_FAST_INTERVAL, true);
  timerAlarmEnable(g_FastTimerhandle);

  Serial.println("Timer init at " + String(TIMER_FAST_INTERVAL));
  Serial.println();
}

void InitQueue(void)
{
    // Fast timer queue

    /* Create a queue capable of containing bytes (used as booleans) */
    g_FastTimerQueue = xQueueCreate( TIMER_FAST_QUEUE_LEN, sizeof(byte) );
    if( g_FastTimerQueue == NULL )
    {
      Serial.println("Queue creation problem !!");
    }
    else
    {
      Serial.println("Queue init for " + String(TIMER_FAST_QUEUE_LEN));
    }

  Serial.println();
}


//---------------
// Task
//---------------

void SignalSendLoopTask(void * dummyParameter )
{
  static unsigned long StartMicros = micros();
  static bool SetupDone = false;
  static unsigned long Delay = 0;
  long duration = 0;

  for (;;)
  {

//-----------------------------------
// Task Setup (done only on 1st call)
//-----------------------------------

    if (!SetupDone)
    {
      delay(100);
      Serial.println("Sending Task Started on core " + String(xPortGetCoreID()));
      InitQueue();
      InitFastTimer();
#ifdef MODE_ANA         
      ledcSetup(LEDCHANNEL, PWM_FREQ, 12);
      ledcAttachPin(PIN_ESP_SENDER_PERIMETER_ENABLE, LEDCHANNEL);
      dac_output_enable(DAC_CHANNEL_1); // GPIO25
#endif
      SetupDone = true;
      xQueueReset(g_FastTimerQueue);
    }

//-----------------------------------
// Task Loop (done on each timer semaphore release)
//-----------------------------------

    bool evt;
    while (xQueueReceive(g_FastTimerQueue, &evt, portMAX_DELAY) == pdPASS)
    {
      int inQueue = uxQueueMessagesWaiting(g_FastTimerQueue);

      if (evt == 1)
  //    if (xSemaphoreTake(g_FastTimerSemaphore, portMAX_DELAY) == pdPASS)
      {
        StartMicros = micros();

        portENTER_CRITICAL_SAFE(&g_LoopMux);     
#ifndef MODE_ANA
        SendCode();
#endif
#ifdef MODE_ANA         
        SendCodeAna();
#endif
        portEXIT_CRITICAL_SAFE(&g_LoopMux);
          //time consuming loop (if necessary !!!)
        Delay = max(0UL,g_delay - (micros()-StartMicros));
        if(Delay > g_delay) {Delay = 0;}

        delayMicroseconds(Delay);

        portENTER_CRITICAL_SAFE(&g_LoopMux);

        xSemaphoreTake(g_GlobalVariablesAccessSemaphore, portMAX_DELAY);
        g_inQueue = g_inQueue + inQueue;
        g_inQueueMax = max(inQueue, (int) g_inQueueMax);
        g_FastLoopCount = g_FastLoopCount + 1;
        duration = micros() - StartMicros;
        g_SenderLoopDuration = g_SenderLoopDuration + duration;
        g_SenderLoopDurationMax = max((long) g_SenderLoopDurationMax, duration);
        g_SenderLoopDurationMin = min((long) g_SenderLoopDurationMin, duration);
        xSemaphoreGive(g_GlobalVariablesAccessSemaphore);

        portEXIT_CRITICAL_SAFE(&g_LoopMux);
      }
    }
//    else
//    {
//      Serial.println("Send task trigger emaphore timeout expired !!!");
//    }
  }
}

void SignalSendLoopTaskCreate(void)
{
  BaseType_t xReturned;
  xReturned = xTaskCreatePinnedToCore(
              SignalSendLoopTask,  /* Task function. */
              "SendSignalTsk",     /* String with name of task. */
              12000,            /* Stack size in bytes. */
              NULL,             /* Parameter passed as input of the task */
              2,                /* Priority of the task. */
              &g_SignalSendTaskHandle,   /* Task handle. */
              ANA_READ_TASK_ESP_CORE);

    if(xReturned == pdPASS)
    {
      Serial.println("Signal send task created on Core " + String(ANA_READ_TASK_ESP_CORE));
    }
    else
    {
      Serial.println("Signal send task creation failled (" + String(xReturned) + ")");
      //errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY	( -1 )
      //errQUEUE_BLOCKED						( -4 )
      //errQUEUE_YIELD							( -5 )
    }
}

void SignalSendLoopTaskSuspend(void)
{
  vTaskSuspend(g_SignalSendTaskHandle);
  Serial.println("Signal send task suspended");
}

void SignalSendLoopTaskResume(void)
{
  vTaskResume(g_SignalSendTaskHandle);
  Serial.println("Signal send task resumed");
}


//---------------
// CPU LOAD
//---------------

void ArtificialLoad(long duration)
{
    unsigned long Start = millis();

    while ( millis() - Start < duration && duration !=0 ) {
        double count = 0;
        long iterations = 2147483647;
        for (long i=0; i<iterations; i++) { 
            for (long j=0; j<iterations; j ++) {
                for (long k=0; k<iterations; k ++) {
                    count = (double) count + (count * PI * i ) / j + k;}
            }
        }
        delayMicroseconds(5);
    }
}

void ArtificialLoadIO(long duration)
{
    unsigned long Start = millis();

    while ( millis() - Start < duration && duration !=0 ) {
      for (int i=32; i<39; i++) { 
        analogRead(i);
        digitalRead(i);
        digitalWrite(i,HIGH);
        delayMicroseconds(5);
      }
    }
}

//---------------
// Main setup
//---------------

void setup() 
{
  Serial.begin(115200); // For debug
  Serial.println();


for (int i= 0; i<sizeof(g_sigcode); i++){ Serial.print(String(g_sigcode[i]) + " ");}
Serial.println();

//#ifndef MODE_ANA         
  pinMode(PIN_ESP_SENDER_PERIMETER_IN1, OUTPUT);
  pinMode(PIN_ESP_SENDER_PERIMETER_IN2, OUTPUT);
  pinMode(PIN_ESP_SENDER_PERIMETER_ENABLE, OUTPUT);
//#endif

  g_GlobalVariablesAccessSemaphore = xSemaphoreCreateMutex();

  SignalSendLoopTaskCreate();
/*
  delay(1000);
  SignalSendLoopTaskSuspend();
  delay(1000);
  SignalSendLoopTaskResume();
  */
}

//---------------
// Main Loop
//---------------

void loop() {
    static unsigned long previous = 0;
//    unsigned long AnaReadBytesRead = 0;
    long inQueue = 0;
    int inQueueMax = 0;
    int Qfull = 0;

    unsigned long now = 0;
//    g_delay = TIMER_FAST_FREQUENCY - 10;  // 10 microsecs is assumed task processing fixed overhead
    g_delay = 0;
    static long load = 0;

      //time consuming loop

    ArtificialLoad(load*1/4);
    ArtificialLoadIO(load*3/4);

    long usedTime = micros() - previous;
//    Serial.println("Used time:" + String(usedTime) + " ms");
    delayMicroseconds(1000*10000UL - (micros()-previous));
    now = micros();

    portENTER_CRITICAL_SAFE(&g_LoopMux);
    xSemaphoreTake(g_GlobalVariablesAccessSemaphore, portMAX_DELAY);

    unsigned long Timerscalls = g_FastTimerCount;
    unsigned long Loopcalls = g_FastLoopCount;
//    unsigned long timeouts = g_timeout;
    unsigned long Writetime = g_writeTime;
    unsigned long SendLoopDuration =  g_SenderLoopDuration;
    unsigned long SendLoopDurationMax =  g_SenderLoopDurationMax;
//    unsigned long AnaReadLoopDurationMaxloop =  g_FastAnaReadLoopDurationMaxLoop;
    unsigned long SendLoopDurationMin =  g_SenderLoopDurationMin;
    inQueue = g_inQueue;
    inQueueMax = g_inQueueMax;
    Qfull = g_Qfull;

    g_FastTimerCount = 0;
    g_FastLoopCount = 0;
//    g_timeout = 0;
    g_writeTime = 0;
    g_inQueue = 0;
    g_inQueueMax = 0;
    g_Qfull = 0;
    g_SenderLoopDuration = 0;
    g_SenderLoopDurationMax = 0;
    g_SenderLoopDurationMin = 9999;

    xSemaphoreGive(g_GlobalVariablesAccessSemaphore);
    portEXIT_CRITICAL_SAFE(&g_LoopMux);

    long missed = Timerscalls - Loopcalls;
    float missedPct = (float) missed/Timerscalls*100;

    Serial.print("TCalls:" + String(Timerscalls));
    Serial.print(" => " + String((float)Timerscalls/(now-previous)*1000,2) + " kHz");
    Serial.print(" |LCalls:" + String(Loopcalls));
    Serial.print(" => " + String((float)Loopcalls/(now-previous)*1000,2) + " kHz");
    Serial.print(" |Missed:" + String(missed) + " " + String(missedPct,3) + " %");
//    Serial.print(" |Timeout:" + String(timeouts));
    Serial.print(" |WriteTime:" + String((float) Writetime/Loopcalls) + " us");
//    Serial.print(" |LTime:" + String(SendLoopDuration) + " us");
    Serial.print(" |LTime:" + String((float) SendLoopDuration/Loopcalls) + " us");
//    Serial.print(" [" + String(AnaReadLoopDurationMin) + "-" + String(AnaReadLoopDurationMax) + "] @" + String(AnaReadLoopDurationMaxloop));
    Serial.print(" [" + String(SendLoopDurationMin) + "-" + String(SendLoopDurationMax) + "]");
//    Serial.print(" | Bytes:" + String(AnaReadBytesRead));
    Serial.print(" || g_delay:" + String(g_delay) );
    Serial.print(" | load:" + String(usedTime/1000) + " ms" );
//    Serial.print(" | FreeHeap:" + String((esp_get_minimum_free_heap_size())));
    Serial.print(" || Queue:" + String((float) inQueue /Loopcalls,4));
    Serial.print(" | Max:" + String(inQueueMax));
    Serial.print(" | Full:" + String(Qfull));
    
    Serial.println();

/*
    portENTER_CRITICAL_SAFE(&g_LoopMux);
    xSemaphoreTake(g_GlobalVariablesAccessSemaphore, portMAX_DELAY);
    g_FastTimerCount = 0;
    g_FastLoopCount = 0;
//    g_timeout = 0;
    g_writeTime = 0;
    g_inQueue = 0;
    g_inQueueMax = 0;
    g_Qfull = 0;
    g_SenderLoopDuration = 0;
    g_SenderLoopDurationMax = 0;
    g_SenderLoopDurationMin = 9999;
    xSemaphoreGive(g_GlobalVariablesAccessSemaphore);
    portEXIT_CRITICAL_SAFE(&g_LoopMux);
*/

    g_delay = g_delay + 1;
    if(g_delay>TIMER_FAST_INTERVAL + 10){g_delay=0;}

   load = load + 50;
//    Serial.println("Load:" + String(load));
    if(load > 900){load = 0;}

    previous = micros();
}