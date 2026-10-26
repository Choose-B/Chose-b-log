---
title: FreeRTOS
tag:
  - FreeRTOS
index_img: /img/Eyjafjalla.avif
hide: true
mathjax: false
excerpt: '[自用]本文用于常见函数声明速查'
date: 2026-04-17 19:53:43
category: 笔记
---
{% note primary %}
**写在前面**
将本文当作学习FreeRTOS的文档是不合适的。本文只是为了能够更加方便快捷地查到FreeRTOS的相关封装。  
建议在使用本文前，先阅读过[野火的FreeRTOS应用开发实战指南](https://doc.embedfire.com/linux/stm32mp1/freertos/zh/latest/README.html)  
{% endnote %}  

# 任务 Thread
## osThreadAttr_t
CMSIS-RTOS使用osThreadAttr_t结构体描述线程属性  
```cpp
 typedef struct {
     const char                   *name;   ///< 线程名，默认为NULL
     uint32_t                 attr_bits;   ///< 用于描述线程的类型，在FreeRTOS内核中没有使用
     void                      *cb_mem;    ///< 控制块内存
     uint32_t                   cb_size;   ///< 控制块大小
     void                   *stack_mem;    ///< 栈地址
     uint32_t                stack_size;   ///< 栈大小
     osPriority_t              priority;   ///< 最初的线程优先级 (默认值: osPriorityNormal)
     TZ_ModuleId_t            tz_module;   ///< 在FreeRTOS内核中没有使用
     uint32_t                  reserved;   ///< 保留 (必须为0)
 } osThreadAttr_t;
```
## osKernelInitalize 内核初始化
CMSIS-RTOS使用`osStatus_t osKernelInitialize(void)`函数用来初始化RTOS内核状态  
{% fold info @定义 %}
```cpp
 osStatus_t osKernelInitialize (void) {
 osStatus_t stat;

 if (IS_IRQ()) {
     stat = osErrorISR;
 }
 else {
     if (KernelState == osKernelInactive) {
     #if defined(USE_FREERTOS_HEAP_5) && (HEAP_5_REGION_SETUP == 1)
         vPortDefineHeapRegions (configHEAP_5_REGIONS);
     #endif
     KernelState = osKernelReady;
     stat = osOK;
     } else {
     stat = osError;
     }
 }

 return (stat);
 }
```
{% endfold %}  


## osThreadNew 新建任务
CMSIS-RTOS使用`osThreadId_t osThreadNew(osThreadFunc_t func, void *argument, const osThreadAttr_t *attr)`函数创建线程
CMSIS-RTOS使用`osStatus_t osKernelInitialize(void)`函数用来初始化RTOS内核状态  
{% fold info @定义 %}
```cpp
osThreadId_t osThreadNew (osThreadFunc_t func, void *argument, const osThreadAttr_t *attr) {
 const char *name;
 uint32_t stack;
 TaskHandle_t hTask;
 UBaseType_t prio;
 int32_t mem;                        //标记创建线程方式(动态、静态)

 hTask = NULL;

 //判断是否在中断中使用该函数以及线程函数是否为空
 if (!IS_IRQ() && (func != NULL)) {
     stack = configMINIMAL_STACK_SIZE;               //默认线程栈大小为128
     prio  = (UBaseType_t)osPriorityNormal;  //默认线程优先级为24

     name = NULL;            //默认线程名为空
     mem  = -1;

     //线程控制块不为空
     if (attr != NULL) {

     //设置线程名
     if (attr->name != NULL) {
         name = attr->name;
     }

     //设置线程优先级
     if (attr->priority != osPriorityNone) {
         prio = (UBaseType_t)attr->priority;
     }

     //线程优先级出错判断
     if ((prio < osPriorityIdle) || (prio > osPriorityISR) || ((attr->attr_bits & osThreadJoinable) == osThreadJoinable)) {
         return (NULL);
     }

     //设置栈大小
     if (attr->stack_size > 0U) {
         /* In FreeRTOS stack is not in bytes, but in sizeof(StackType_t) which is 4 on ARM ports.       */
         /* Stack size should be therefore 4 byte aligned in order to avoid division caused side effects */
         stack = attr->stack_size / sizeof(StackType_t);
     }

     //判断线程控制块创建线程的方式
     if ((attr->cb_mem    != NULL) && (attr->cb_size    >= sizeof(StaticTask_t)) &&
         (attr->stack_mem != NULL) && (attr->stack_size >  0U)) {
         mem = 1;
     }
     else {
         if ((attr->cb_mem == NULL) && (attr->cb_size == 0U) && (attr->stack_mem == NULL)) {
         mem = 0;
         }
     }
     }
     else {
     mem = 0;
     }

     //使用静态创建线程
     if (mem == 1) {
     hTask = xTaskCreateStatic ((TaskFunction_t)func, name, stack, argument, prio, (StackType_t  *)attr->stack_mem,
                                                                                     (StaticTask_t *)attr->cb_mem);
     }
     //使用动态创建线程
     else {
     if (mem == 0) {
         if (xTaskCreate ((TaskFunction_t)func, name, (uint16_t)stack, argument, prio, &hTask) != pdPASS) {
         hTask = NULL;
         }
     }
     }
 }
 //将使用FreeRTOS创建线程得到的线程id返回
 return ((osThreadId_t)hTask);
 }
```
{% endfold %}  
**Param**:
* `func`: 回调函数
* `argument`: 线程函数参数，一般填`nullptr`
* `attr`: osThreadAttr_t线程控制块

初始化代码示例:
```cpp
osThreadId_t         test_task_handle; 
const osThreadAttr_t test_handler_task_attributes = {
  .name       = "test_task",
  .stack_size = 128 * 4,
  .priority   = (osPriority_t)osPriorityNormal, // 绝大部分情况下优先级都用默认
};

void freertos_init()
{
  /*...*/
  test_task_handle = osThreadNew(test_handler_task, nullptr, &test_handler_task_attributes);
}
```

## osKernelStart 内核启动
osKernelStart函数用于启动任务调度器，该函数不能在中断中使用  
只有执行过内核启动函数，FreeRTOS才会真正启动
{% fold info @定义 %}
```cpp
 osStatus_t osKernelStart (void) {
 osStatus_t stat;

 //判读是否在中断中执行osKernelStart函数
 if (IS_IRQ()) {
     stat = osErrorISR;
 }
 else {
     if (KernelState == osKernelReady) {
     /* Ensure SVC priority is at the reset value */
     SVC_Setup();
     /* Change state to enable IRQ masking check */
     KernelState = osKernelRunning;   // 设置内核状态为运行态
     /* Start the kernel scheduler */
     vTaskStartScheduler();          //FreeRTOS启动任务调度器函数
     stat = osOK;
     } else {
     stat = osError;
     }
 }

 return (stat);
 }
```
{% endfold %} 
## osThreadGetId 返回当前线程ID
线程ID主要用于分辨不同的线程任务，当我们需要对线程进行挂起、唤醒、删除等操作时就需要知道相对应的线程ID。 除了创建线程函数osThreadNew外，还可以通过使用osThreadGetId函数获取本线程的线程ID  
**但是**，我们的初始化中一般都会把创建的线程id存入一个全局变量内，因此，一般情况下你不应该使用该函数来获取线程ID。  
{% fold info @定义 %}
```cpp
osThreadId_t osThreadGetId (void) {
osThreadId_t id;

   id = (osThreadId_t)xTaskGetCurrentTaskHandle();

   return (id);
}
```
{% endfold %} 
## osThreadSuspend 任务挂起函数
挂起指定任务。  
任务可以通过调用 osThreadSuspend()函数都可以将处于任何状态的任务挂起，被挂起的任务得不到CPU的使用权， 也不会参与调度，它相对于调度器而言是不可见的，除非它从挂起态中解除。  
{% fold info @定义 %}
```cpp
osStatus_t osThreadSuspend (osThreadId_t thread_id) {
TaskHandle_t hTask = (TaskHandle_t)thread_id;
osStatus_t stat;

//并不能在中断中使用挂起函数
if (IS_IRQ()) {
   stat = osErrorISR;
}
//判断任务句柄的有效性
else if (hTask == NULL) {
   stat = osErrorParameter;
}
//使用vTaskSuspend挂起线程
else {
   stat = osOK;
   vTaskSuspend (hTask);
}

return (stat);
}
```
{% endfold %} 
## osThreadResume 任务恢复函数
任务恢复就是让挂起的任务重新进入就绪状态， 恢复的任务会保留挂起前的状态信息，在恢复的时候根据挂起时的状态继续运行。 如果被恢复任务在所有就绪态任务中，处于最高优先级列表的第一位，那么系统将进行任务上下文的切换。
{% fold info @定义 %}
```cpp
osStatus_t osThreadResume (osThreadId_t thread_id) {
   TaskHandle_t hTask = (TaskHandle_t)thread_id;
   osStatus_t stat;

   //并不能在中断中使用恢复函数
   if (IS_IRQ()) {
      stat = osErrorISR;
   }
   //判断任务句柄的有效性
   else if (hTask == NULL) {
      stat = osErrorParameter;
   }
   //使用vTaskResume函数恢复线程
   else {
      stat = osOK;
      vTaskResume (hTask);
   }
   return (stat);
}
```
{% endfold %} 
## osThreadExit 终止当前线程
osThreadExit函数用于终止当前运行线程的执行，其函数源码如下
{% fold info @定义 %}
```cpp
__NO_RETURN void osThreadExit (void) {
#ifndef USE_FreeRTOS_HEAP_1
   vTaskDelete (NULL);
#endif
   for (;;);
}
```
{% endfold %} 
## osThreadTerminate 删除（其他）线程
CMSIS-RTOS提供了osThreadTerminate函数可用于删除其他的线程，其函数源码如下  
当内存分配策略为`heap_1`时，删除线程是被禁止的。
{% fold info @定义 %}
```cpp
osStatus_t osThreadTerminate (osThreadId_t thread_id) {
   TaskHandle_t hTask = (TaskHandle_t)thread_id;
   osStatus_t stat;
#ifndef USE_FreeRTOS_HEAP_1
   eTaskState tstate;

   //并不能在中断中使用线程结束函数
   if (IS_IRQ()) {
      stat = osErrorISR;
   }
   //判断任务句柄的有效性
   else if (hTask == NULL) {
      stat = osErrorParameter;
   }
   else {
      //获取线程状态
      tstate = eTaskGetState (hTask);

      if (tstate != eDeleted) {
         stat = osOK;
         vTaskDelete (hTask);  //使用vTaskDelete删除线程
      } else {
         stat = osErrorResource;
      }
   }
#else
   stat = osError;
#endif

   return (stat);
}
```
{% endfold %} 
## osDelay 延时
{% fold info @定义 %}
```cpp

```
{% endfold %} 

# 临界段 

# 消息队列 MessageQueue
## osMessageQueueAttr_t
## osMessageQueueNew 创建消息队列
## osMessageQueueDelete 删除消息队列
## osMessageQueuePut 向消息队列发送消息
## osMessageQueueGet 从消息队列读取消息

# 信号量 Semaphore
## osSemaphoreAttr_t
## osSemaphoreNew 创建信号量
## osSemaphoreDelete 删除信号量
## osSemaphoreRelease 释放信号量
## osSemaphoreAcquire 获取信号量

# 互斥量 Mutex
## osMutexAttr_t
## osMutexNew 创建互斥量
## osMutexDelete 删除信号量
## osMutexAcquire 获取信号量
## osMutexRelease 释放信号量

# 事件 EventFlags
## osEventFlagsAttr_t
## osEventFlagsNew 创建事件
## osEventFlagsDelete 删除事件
## osEventFlagsSet 事件组置位
## osEventFlagsWait 等待事件
## osEventFlagsClear 清除事件

# 软件定时器 Timer
## osTimerAttr_t
## osTimerNew 创建软件定时器
## osTimerDelete 删除软件定时器
## osTimerStart 启动软件定时器
## osTimerStor 停止软件定时器

# 任务通知 TaskGeneric
## xTaskGenericNotify 发送任务通知
## xTaskNotifyGive 
## xTaskNotify
## xTaskNotifyFromISR
## xTaskGenericNotifyFromISR 中断中发送任务通知通用函数
## xTaskNotifyAndQuery
## xTaskNotifyAndQueryFromISR
## 获取任务通知
### ulTaskNotifyTake
### xTaskNotifyWait
