#include "tp.h"
#include "tpl_os.h"

#define APP_Task_blink_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(int, OS_APPL_CODE) main(void)
{

  ledinit();
  ledBootInit();
  svDltInit();

  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

TASK(blink)
{
    ledon();
    for(long long i = 0; i < 2000000; i++);
    ledoff();
    for(long long i = 0; i < 2000000; i++);
    TerminateTask();
}
#define APP_Task_blink_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_life_START_SEC_CODE
#include "tpl_memmap.h"

TASK(life)
{
    svDltSend('1');
    svDltSend('\r');
    svDltSend('\n');
    TerminateTask();
}
#define APP_Task_life_STOP_SEC_CODE
#include "tpl_memmap.h"

#define OS_START_SEC_CODE
#include "tpl_memmap.h"
/*
 *  * This is necessary for ST libraries
 *   */
FUNC(void, OS_CODE) assert_failed(uint8_t* file, uint32_t line)
{
}

FUNC(void, OS_CODE) PreTaskHook()
{
  TaskType task_id = 0;
  GetTaskID(&task_id);
  if (task_id == blink) {
    svDltSend('2');
    svDltSend('\r');
    svDltSend('\n');
  }
}

FUNC(void, OS_CODE) PostTaskHook()
{
  TaskType task_id = 0;
  GetTaskID(&task_id);
  if (task_id == blink) {
    svDltSend('3');
    svDltSend('\r');
    svDltSend('\n');
  }
}
#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"

