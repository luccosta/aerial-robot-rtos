/*
 * FreeRTOS Kernel V10.0.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/******************************************************************************
 * This project provides two demo applications.  A simple blinky style project,
 * and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting is used to select between the two.
 * The simply blinky demo is implemented and described in main_blinky.c.  The
 * more comprehensive test and demo application is implemented and described in
 * main_full.c.
 *
 * This file implements the code that is not demo specific, including the
 * hardware setup and FreeRTOS hook functions.
 *
 *******************************************************************************
 * NOTE: Windows will not be running the FreeRTOS demo threads continuously, so
 * do not expect to get real time behaviour from the FreeRTOS Windows port, or
 * this demo application.  Also, the timing information in the FreeRTOS+Trace
 * logs have no meaningful units.  See the documentation page for the Windows
 * port for further information:
 * http://www.freertos.org/FreeRTOS-Windows-Simulator-Emulator-for-Visual-Studio-and-Eclipse-MingW.html
 *

 *
 *******************************************************************************
 */

/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
//#include <conio.h>

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <time.h>
#include <math.h>
#include <string.h>

/* This project provides two demo applications.  A simple blinky style demo
application, and a more comprehensive test and demo application.  The
mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting is used to select between the two.

If mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is 1 then the blinky demo will be built.
The blinky demo is implemented and described in main_blinky.c.

If mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is not 1 then the comprehensive test and
demo application will be built.  The comprehensive test and demo application is
implemented and described in main_full.c. */
#define mainCREATE_SIMPLE_BLINKY_DEMO_ONLY	1

/* This demo uses heap_5.c, and these constants define the sizes of the regions
that make up the total heap.  heap_5 is only used for test and example purposes
as this demo could easily create one large heap region instead of multiple
smaller heap regions - in which case heap_4.c would be the more appropriate
choice.  See http://www.freertos.org/a00111.html for an explanation. */
#define mainREGION_1_SIZE	7201
#define mainREGION_2_SIZE	29905
#define mainREGION_3_SIZE	6407

/*-----------------------------------------------------------*/

/*
 * main_blinky() is used when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1.
 * main_full() is used when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 0.
 */
//extern void main_blinky( void );
//extern void main_full( void );

/*
 * Only the comprehensive demo uses application hook (callback) functions.  See
 * http://www.freertos.org/a00016.html for more information.
 */
//void vFullDemoTickHookFunction( void );
//void vFullDemoIdleFunction( void );

/*
 * This demo uses heap_5.c, so start by defining some heap regions.  It is not
 * necessary for this demo to use heap_5, as it could define one large heap
 * region.  Heap_5 is only used for test and example purposes.  See
 * http://www.freertos.org/a00111.html for an explanation.
 */
static void  prvInitialiseHeap( void );

/*
 * Prototypes for the standard FreeRTOS application hook (callback) functions
 * implemented within this file.  See http://www.freertos.org/a00016.html .
 */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/*
 * Writes trace data to a disk file when the trace recording is stopped.
 * This function will simply overwrite any trace files that already exist.
 */
static void prvSaveTraceFile( void );

/*-----------------------------------------------------------*/

/* When configSUPPORT_STATIC_ALLOCATION is set to 1 the application writer can
use a callback function to optionally provide the memory required by the idle
and timer tasks.  This is the stack that will be used by the timer task.  It is
declared here, as a global, so it can be checked by a test that is implemented
in a different file. */
StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

/* Notes if the trace is running or not. */
static BaseType_t xTraceRunning = pdTRUE;

/*-----------------------------------------------------------*/

#define FLIGHT_CONTROLLER_PERIOD 10
#define ENCODERS_PERIOD 1000
#define MISSION_MOV_CONTROLLER_PERIOD 100
#define MISSION_DEL_CONTROLLER_PERIOD 33

#define DEFERRABLE_SERVER_TIME 10

#define DEBUG FALSE
#define TEST FALSE

xTaskHandle rg, pc, re, rgps, rl, proc, oa, traj, rcam, ip, gr, fc, mr, tr, ds;
QueueHandle_t buffer;
BOOL moving = TRUE;
FILE* fptr;
char task_log[10000];
int time_left = 0;
int deferrable_server_stack = 0;
int i = 0;

/* === FUNÇÕES AUXILIARES === */

/// \brief Simulação tempo de execução no formato de um delay.
/// A função é adaptada para funcionar num cenário preemptivo
/// onde é formada uma pilha de tempo de execução.
/// \param[in] execution_time tempo de execução a ser simulado
void delay(int execution_time)
{
	// Adicionando o tempo de execução da task atual à pilha de tempo de execução total
	int initial_time_left = time_left;
	time_left += execution_time;

	// Armazenando tempo inicial 
	TickType_t start_time = xTaskGetTickCount();

	// Executa o loop enquanto o tempo requerido não é atingido 
	while (xTaskGetTickCount() < start_time + time_left - initial_time_left)
		;

	time_left = initial_time_left + execution_time;
}

/// \brief Armazenamento das informações de funcionamento da
/// aplicação durante a execução. Mais especificamente da
/// ocorrência de tasks.
/// \param[in] task_id Identificação da task
/// \param[in] time tempo no momento da ocorrência
/// \param[in] start Se a ocorrência é de início ou fim 
void write_log(int task_id, int time, BOOL start)
{
	char temp[15];
	sprintf(temp, "%d %d %d, ", task_id, time, start);
	strcat(task_log, temp);

	if (DEBUG)
		printf("%d %d %d\n", task_id, time, start);

	i++;
	if (i >= 500) {
		// Término da simulação
		vTaskSuspendAll();

		printf("finish!\n");
		
		fprintf(fptr, "%s", task_log);
		fclose(fptr);

		vTaskEndScheduler();
	}
}

/// \brief Modelo para criação de tarefas periódicas
/// \param[in] task_id Identificação da task
/// \param[in] execution_time Tempo de execução em ticks
/// \param[in] period Período em ticks
/// \param[in] resume Tarefa que deve ser liberada por essa
/// \param[in] suspend Tarefa que deve ser suspendida por essa
void periodic_task(int task_id, TickType_t execution_time, TickType_t period, 
					xTaskHandle resume, xTaskHandle suspend)
{
	TickType_t start;

	while (1)
	{
		start = xTaskGetTickCount();

		write_log(task_id, start, TRUE);
		delay(execution_time);
		write_log(task_id, xTaskGetTickCount(), FALSE);

		// Resolvendo dependências
		if (resume != NULL) {
			vTaskResume(resume);
		}

		if (suspend != NULL) {
			vTaskSuspend(suspend);
		}

		vTaskDelayUntil(&start, period);
	}
}

/// \brief Modelo para criação de tarefas aperiódicas.
/// Ao invés de diretamente executar a tarefa, o tempo de
/// execução é adicionado à pilha de execução do deferrable
/// server. Na prática somente simboliza a percepção de
/// disponibilidade da tarefa.
/// \param[in] task_id Identificação da task
/// \param[in] execution_time Tempo de execução em ticks
/// \param[in] average Tempo médio do disparo da tarefa
/// \param[in] std_deviation Desvio padrão do tempo
/// \param[in] resume Tarefa que deve ser liberada por essa
/// \param[in] suspend Tarefa que deve ser suspendida por essa
void aperiodic_task(int task_id, TickType_t execution_time, 
					float average, float std_deviation, 
					xTaskHandle resume, xTaskHandle suspend) {
	TickType_t start;

	while (1)
	{
		deferrable_server_stack += execution_time;

		// Resolvendo dependências
		if (resume != NULL) {
			vTaskResume(resume);
		}

		if (suspend != NULL) {
			vTaskSuspend(suspend);
		}

		// Tempo de espera para a próxima chamada aleatório
		vTaskDelayUntil(&start, _normal(average, std_deviation));
	}
}

/// \brief Modelo para criação de tarefas esporádicas.
/// Muito semelhante aos modelos de tarefa anteriores
/// possui determinação de um período aleatório, como
/// nas aperiódicas, porém executado quando chamada 
/// como as periódicas. Concepção com alta prioridade.
/// \param[in] task_id Identificação da task
/// \param[in] execution_time Tempo de execução em ticks
/// \param[in] average Tempo médio do disparo da tarefa
/// \param[in] std_deviation Desvio padrão do tempo
/// \param[in] resume Tarefa que deve ser liberada por essa
/// \param[in] suspend Tarefa que deve ser suspendida por essa
void esporadic_task(int task_id, TickType_t execution_time,
	float average, float std_deviation,
	xTaskHandle resume, xTaskHandle suspend) {
	TickType_t start;

	while (1)
	{
		start = xTaskGetTickCount();

		write_log(task_id, start, TRUE);
		delay(execution_time);
		write_log(task_id, xTaskGetTickCount(), FALSE);

		// Resolvendo dependências
		if (resume != NULL) {
			vTaskResume(resume);
		}

		if (suspend != NULL) {
			vTaskSuspend(suspend);
		}

		// Tempo de espera para a próxima chamada aleatório
		vTaskDelayUntil(&start, _normal(average, std_deviation));
	}
}

// Funções para geração de números com distribuição Gaussiana
double _rand_gen() {
	// Retorna um número aleatório uniformemente distribuido
	return ((double)(rand()) + 1.) / ((double)(RAND_MAX)+1.);
}

double _normalRandom() {
	// return a normally distributed random value
	double v1 = _rand_gen();
	double v2 = _rand_gen();
	return cos(2 * 3.14 * v2) * sqrt(-2. * log(v1));
}

int _normal(float media, float desvio_padrao) {
	return (int)(1000 * (_normalRandom() * desvio_padrao + media));
}

/* === TASKS === */


/* --- Controlador de voo --- */

void read_encoders()
{
	periodic_task(1, 100, ENCODERS_PERIOD, NULL, NULL);
}

void read_gyroscope()
{
	// Libera a task pid_control que depende da atual
	periodic_task(2, 1, FLIGHT_CONTROLLER_PERIOD, pc, NULL);
}

void pid_control()
{
	// Suspende a task atual até que a dependência read_gyroscope seja executada
	periodic_task(3, 1, FLIGHT_CONTROLLER_PERIOD, NULL, pc);
}


/* --- Controlador de missão --- */

/* Em movimento */
void read_gps()
{
	// Libera processamento
	periodic_task(4, 10, MISSION_MOV_CONTROLLER_PERIOD, proc, NULL);
}

void read_lidar()
{
	// Libera procesamento
	periodic_task(5, 10, MISSION_MOV_CONTROLLER_PERIOD, proc, NULL);
}

void processing()
{
	// Suspende a si mesma
	periodic_task(6, 10, MISSION_MOV_CONTROLLER_PERIOD, NULL, proc);
}

void obstacle_avoidance()
{
	// Libera trajectory
	esporadic_task(7, 10, 10, 2, tr, NULL);
}

void trajectory()
{
	aperiodic_task(8, 20, 12.5, 2.5, NULL, NULL);
}

/* Em entrega */
void read_cam() 
{
	// Libera image_processing
	periodic_task(9, 5, MISSION_DEL_CONTROLLER_PERIOD, ip, NULL);
}

void image_processing()
{
	// Libera gradient e suspende image_processing
	periodic_task(9, 10, MISSION_DEL_CONTROLLER_PERIOD, gr, ip);
}

void gradient() 
{
	float position = 3;

	// Enviando posição para controle de descida
	xQueueSend(buffer, &position, 0);

	// Suspende a si mesma
	periodic_task(9, 5, MISSION_DEL_CONTROLLER_PERIOD, NULL, gr);
}

void fall_controll() 
{
	float position;

	// Recebendo altura do cálculo de gradiente
	xQueueReceive(buffer, &position, pdMS_TO_TICKS(1000));

	aperiodic_task(12, 5, 1, 0.1, mr, NULL);
}

void mechanism_retreat()
{
	aperiodic_task(13, 5, 15, 2.5, NULL, mr);
}


/* Infraestrutura */
void transition()
{
	// Em movimento rgps, rl, proc, oa, traj
	// Em entrega rcam, ip, gr, fc, mr
	while (1)
	{
		printf("transition!\n");
		// Altera as tasks vigentes, porém somente as tasks
		// sem dependências, as com depências só devem ser 
		// liberadas por suas dependências.
		if (moving)
		{
			vTaskResume(rcam);
			vTaskResume(fc);
			vTaskSuspend(rgps);
			vTaskSuspend(rl);
			vTaskSuspend(oa);
			
			moving = FALSE;
		}
		else
		{
			vTaskResume(rgps);
			vTaskResume(rl);
			vTaskResume(oa);
			vTaskSuspend(rcam);
			vTaskSuspend(fc);

			moving = TRUE;
		}

		vTaskDelay(_normal(20, 5));
	}
}

void deferrable_server() {
	TickType_t start;

	while (1)
	{
		start = xTaskGetTickCount();

		write_log(14, start, TRUE);

		// Caso o tempo acumulado na pilha do deferrable server
		// seja menor que o tempo reservado para execução são
		// executadas todas as tasks na pilha. Caso seja maior
		// é executado o que o tempo reservado comporta e o que
		// sobra tem de esperar o próximo ciclo na pilha.
		if (deferrable_server_stack < DEFERRABLE_SERVER_TIME)
		{
			deferrable_server_stack = 0;
			delay(deferrable_server_stack);
		}
		else
		{
			deferrable_server_stack -= DEFERRABLE_SERVER_TIME;
			delay(DEFERRABLE_SERVER_TIME);
		}

		write_log(14, xTaskGetTickCount(), FALSE);
		
		if (moving)
			vTaskDelayUntil(&start, MISSION_MOV_CONTROLLER_PERIOD);
		else
			vTaskDelayUntil(&start, MISSION_DEL_CONTROLLER_PERIOD);
	}
}

// Testa a preemptividade do delay
void test1() 
{
	periodic_task(1, 1000, 3000, NULL, NULL);
}

void test2()
{
	periodic_task(2, 1000, 3000, NULL, NULL);
}

// TODO: Implementar dependências
// TODO: Implementar delay próprio do RTOS, utilizando seus ticks
// TODO: Solucionar o problema de não conseguir tempos menores do que 1 ms
// TODO: Utilizar Queue para reduzir variáveis globais (https://www.embarcados.com.br/rtos-queue-sincronizacao-e-comunicacao/)
// TODO: Criar transition ou outra infraestrutura a partir de alguma task (https://exploreembedded.com/wiki/Creating_a_Task_from_other_Tasks)
// TODO: Deferrable Server

int main( void )
{
	/* This demo uses heap_5.c, so start by defining some heap regions.  heap_5
	is only used for test and example reasons.  Heap_4 is more appropriate.  See
	http://www.freertos.org/a00111.html for an explanation. */
	prvInitialiseHeap();

	/* Initialise the trace recorder.  Use of the trace recorder is optional.
	See http://www.FreeRTOS.org/trace for more information. */
	vTraceEnable( TRC_START );

	// Inicialização do log
	strcpy(task_log, "");
	fptr = fopen("log.txt", "w");

	// Criação da queue *buffer* com 10 slots de 4 Bytes
	buffer = xQueueCreate(10, sizeof(uint32_t));

	if (TEST)
	{
		xTaskCreate(test1, (signed char*)"test1",
			configMINIMAL_STACK_SIZE, (void*)NULL, 1, &rg);
		xTaskCreate(test2, (signed char*)"test2",
			configMINIMAL_STACK_SIZE, (void*)NULL, 1, &rg);
	}
	else
	{
		// Controlador de Voo
		xTaskCreate(read_gyroscope, (signed char*)"read_gyroscope",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9 - 1, &rg);

		xTaskCreate(pid_control, (signed char*)"pid_control",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9 - 2, &pc);

		xTaskCreate(read_encoders, (signed char*)"read_encoders",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9 - 8, &re);

		// Controlador de missão - Em movimento
		xTaskCreate(read_gps, (signed char*)"read_gps",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9 - 6, &rgps);

		xTaskCreate(read_lidar, (signed char*)"read_lidar",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9 - 4, &rl);

		xTaskCreate(processing, (signed char*)"processing",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9 - 5, &proc);

		xTaskCreate(obstacle_avoidance, (signed char*)"obstacle_avoidance",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9 - 3, &oa);

		xTaskCreate(trajectory, (signed char*)"trajectory",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9 - 7, &traj);

		// Controlador de missão - Em entrega
		xTaskCreate(read_cam, (signed char*)"read_cam",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9 - 6, &rcam);

		xTaskCreate(image_processing, (signed char*)"image_processing",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9 - 5, &ip);

		xTaskCreate(gradient, (signed char*)"gradient",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9 - 4, &gr);

		xTaskCreate(fall_controll, (signed char*)"fall_controll",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9 - 3, &fc);

		xTaskCreate(mechanism_retreat, (signed char*)"mechanism_retreat",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9 - 7, &mr);

		// Infraestrutura
		xTaskCreate(transition, (signed char*)"transition",
			configMINIMAL_STACK_SIZE, (void*)NULL, 9, &tr);

		// TODO: Definir a prioridade do servidor

		xTaskCreate(deferrable_server, (signed char*)"deferrable_server",
			configMINIMAL_STACK_SIZE, (void*)NULL, 8, &ds);

		// Começa no estado de movimento
		vTaskSuspend(rcam);
		vTaskSuspend(ip);
		vTaskSuspend(gr);
		vTaskSuspend(fc);
		vTaskSuspend(mr);

		// Suspende tasks que tem dependências
		vTaskSuspend(pc);	// Depende de read_gyroscope

		vTaskSuspend(proc); // Depende de read_lidar e read_gps
		vTaskSuspend(traj); // Depende de obstacle_avoidance

		vTaskSuspend(ip);	// Depende de read_cam
		vTaskSuspend(gr);	// Depende de image_processing
	}

	vTaskStartScheduler();
	for (;;);
	return 0;
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c, heap_2.c or heap_4.c is being used, then the
	size of the	heap available to pvPortMalloc() is defined by
	configTOTAL_HEAP_SIZE in FreeRTOSConfig.h, and the xPortGetFreeHeapSize()
	API function can be used to query the size of free heap space that remains
	(although it does not provide information on how the remaining heap might be
	fragmented).  See http://www.freertos.org/a00111.html for more
	information. */
	vAssertCalled( __LINE__, __FILE__ );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If application tasks make use of the
	vTaskDelete() API function to delete themselves then it is also important
	that vApplicationIdleHook() is permitted to return to its calling function,
	because it is the responsibility of the idle task to clean up memory
	allocated by the kernel to any task that has since deleted itself. */

	/* Uncomment the following code to allow the trace to be stopped with any
	key press.  The code is commented out by default as the kbhit() function
	interferes with the run time behaviour. */
	/*
		if( _kbhit() != pdFALSE )
		{
			if( xTraceRunning == pdTRUE )
			{
				vTraceStop();
				prvSaveTraceFile();
				xTraceRunning = pdFALSE;
			}
		}
	*/

	#if ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY != 1 )
	{
		/* Call the idle task processing used by the full demo.  The simple
		blinky demo does not use the idle task hook. */
		vFullDemoIdleFunction();
	}
	#endif
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  This function is
	provided as an example only as stack overflow checking does not function
	when running the FreeRTOS Windows port. */
	vAssertCalled( __LINE__, __FILE__ );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
	#if ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY != 1 )
	{
		vFullDemoTickHookFunction();
	}
	#endif /* mainCREATE_SIMPLE_BLINKY_DEMO_ONLY */
}
/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void )
{
	/* This function will be called once only, when the daemon task starts to
	execute	(sometimes called the timer task).  This is useful if the
	application includes initialisation code that would benefit from executing
	after the scheduler has been started. */
}
/*-----------------------------------------------------------*/

void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{
static BaseType_t xPrinted = pdFALSE;
volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;

	/* Called if an assertion passed to configASSERT() fails.  See
	http://www.freertos.org/a00110.html#configASSERT for more information. */

	/* Parameters are not used. */
	( void ) ulLine;
	( void ) pcFileName;

	printf( "ASSERT! Line %ld, file %s, GetLastError() %ld\r\n", ulLine, pcFileName, GetLastError() );

 	taskENTER_CRITICAL();
	{
		/* Stop the trace recording. */
		if( xPrinted == pdFALSE )
		{
			xPrinted = pdTRUE;
			if( xTraceRunning == pdTRUE )
			{
				vTraceStop();
				prvSaveTraceFile();
			}
		}

		/* You can step out of this function to debug the assertion by using
		the debugger to set ulSetToNonZeroInDebuggerToContinue to a non-zero
		value. */
		while( ulSetToNonZeroInDebuggerToContinue == 0 )
		{
			__asm{ NOP };
			__asm{ NOP };
		}
	}
	taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

static void prvSaveTraceFile( void )
{
FILE* pxOutputFile;

	fopen_s( &pxOutputFile, "Trace.dump", "wb");

	if( pxOutputFile != NULL )
	{
		fwrite( RecorderDataPtr, sizeof( RecorderDataType ), 1, pxOutputFile );
		fclose( pxOutputFile );
		printf( "\r\nTrace output saved to Trace.dump\r\n" );
	}
	else
	{
		printf( "\r\nFailed to create trace dump file\r\n" );
	}
}
/*-----------------------------------------------------------*/

static void  prvInitialiseHeap( void )
{
/* The Windows demo could create one large heap region, in which case it would
be appropriate to use heap_4.  However, purely for demonstration purposes,
heap_5 is used instead, so start by defining some heap regions.  No
initialisation is required when any other heap implementation is used.  See
http://www.freertos.org/a00111.html for more information.

The xHeapRegions structure requires the regions to be defined in start address
order, so this just creates one big array, then populates the structure with
offsets into the array - with gaps in between and messy alignment just for test
purposes. */
static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
volatile uint32_t ulAdditionalOffset = 19; /* Just to prevent 'condition is always true' warnings in configASSERT(). */
const HeapRegion_t xHeapRegions[] =
{
	/* Start address with dummy offsets						Size */
	{ ucHeap + 1,											mainREGION_1_SIZE },
	{ ucHeap + 15 + mainREGION_1_SIZE,						mainREGION_2_SIZE },
	{ ucHeap + 19 + mainREGION_1_SIZE + mainREGION_2_SIZE,	mainREGION_3_SIZE },
	{ NULL, 0 }
};

	/* Sanity check that the sizes and offsets defined actually fit into the
	array. */
	configASSERT( ( ulAdditionalOffset + mainREGION_1_SIZE + mainREGION_2_SIZE + mainREGION_3_SIZE ) < configTOTAL_HEAP_SIZE );

	/* Prevent compiler warnings when configASSERT() is not defined. */
	( void ) ulAdditionalOffset;

	vPortDefineHeapRegions( xHeapRegions );
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

	/* Pass out a pointer to the StaticTask_t structure in which the Idle task's
	state will be stored. */
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

	/* Pass out the array that will be used as the Idle task's stack. */
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;

	/* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xTimerTaskTCB;

	/* Pass out a pointer to the StaticTask_t structure in which the Timer
	task's state will be stored. */
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

	/* Pass out the array that will be used as the Timer task's stack. */
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;

	/* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

