/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TASKS_H__
#define __TASKS_H__

#include <unistd.h>
#include <iostream>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "messages.h"
#include "commonitor.h"
#include "comrobot.h"
#include "camera.h"
#include "img.h"

using namespace std;

class Tasks {
public:
    /**
     * @brief Initializes main structures (semaphores, tasks, mutex, etc.)
     */
    void Init();

    /**
     * @brief Starts tasks
     */
    void Run();

    /**
     * @brief Stops tasks
     */
    void Stop();
    
    /**
     * @brief Suspends main thread
     */
    void Join();
    
private:
    /**********************************************************************/
    /* Shared data                                                        */
    /**********************************************************************/
    ComMonitor monitor;
    ComRobot robot;

    bool activateWatchdog = false; // Permet de savoir si le robot doit démarrer avec ou sans le watchdog
    
    int robotStarted = 0;
    int moveRobot = MESSAGE_ROBOT_STOP; 
    
    bool getBattery=false; 
    
    Camera *cam = new Camera(); 
    bool cameraOpen; // correspond a la demande d'ouverture de la camera
    bool sendImage;  // correspond a la demande d'envoi periodique d'image
    
    Arena arena; 
    bool arenaOK=false; // passe a true si l'arene est validee 
    bool stopSearchArena = false;
    bool stopSendImageFromArenaSearch = false;
    
    /**********************************************************************/
    /* Tasks                                                              */
    /**********************************************************************/
    RT_TASK th_server;
    RT_TASK th_sendToMonitor;
    RT_TASK th_receiveFromMonitor;
    RT_TASK th_openRobotCommunication;
    RT_TASK th_startRobot;
    RT_TASK th_moveRobot;
    RT_TASK th_manageBatteryLevel;
    RT_TASK th_openCamera; 
    RT_TASK th_closeCamera; 
    RT_TASK th_sendImageToMonitor;
    RT_TASK th_manageArena;
    RT_TASK th_detectCommunicationLossMonitor;
    
    /**********************************************************************/
    /* Mutex                                                              */
    /**********************************************************************/
    RT_MUTEX mutex_monitor;
    RT_MUTEX mutex_robot;
    RT_MUTEX mutex_robotStarted;
    RT_MUTEX mutex_moveRobot;
    RT_MUTEX mutex_battery;
    RT_MUTEX mutex_cameraOpen; 
    RT_MUTEX mutex_cam; 
    RT_MUTEX mutex_sendImage; 
    RT_MUTEX mutex_arena;
    RT_MUTEX mutex_arenaOK; 
    RT_MUTEX mutex_stopSearchArena;
    RT_MUTEX mutex_stopSendImageFromArenaSearch;
    RT_MUTEX mutex_watchdog;

    /**********************************************************************/
    /* Semaphores                                                         */
    /**********************************************************************/
    RT_SEM sem_barrier;
    RT_SEM sem_openRobotCommunication;
    RT_SEM sem_serverOk;
    RT_SEM sem_startRobot;
    RT_SEM sem_getBattery;
    RT_SEM sem_openCamera; 
    RT_SEM sem_closeCamera; 
    RT_SEM sem_startSendingImage; 
    RT_SEM sem_sendImageFromArenaSearch;
    RT_SEM sem_searchArena;
    RT_SEM sem_arenaAns;
    RT_SEM sem_flowImage;
    RT_SEM sem_watchdog;

    /**********************************************************************/
    /* Message queues                                                     */
    /**********************************************************************/
    int MSG_QUEUE_SIZE;
    RT_QUEUE q_messageToMon;
    
    /**********************************************************************/
    /* Tasks' functions                                                   */
    /**********************************************************************/
    /**
     * @brief Thread handling server communication with the monitor.
     */
    void ServerTask(void *arg);
     
    /**
     * @brief Thread sending data to monitor.
     */
    void sendToMonitorTask(void *arg);
        
    /**
     * @brief Thread receiving data from monitor.
     */
    void receiveFromMonitorTask(void *arg);
    
    /**
     * @brief Thread opening communication with the robot.
     */
    void openRobotCommunication(void *arg);

    /**
     * @brief Thread starting the communication with the robot.
     */
    void StartRobotTask(void *arg);
    
    /**
     * @brief Thread handling control of the robot.
     */
    void moveRobotTask(void *arg);

    /**
     * @brief Thread handling the displaying of the battery level
     */
    void manageBatteryLevelTask(void *arg);
    
    /**
     * @brief Thread opening the camera
     */
    void openCameraTask(void *arg);
    
    /**
     * @brief Thread closing the camera
     */
    void closeCameraTask(void *arg); 
    
    /**
     * @brief Thread sending images from the camera to the monitor
     */
    void sendImageToMonitorTask(void *arg);

    /**
     * @brief Thread handling the research of the arena
     */
    void manageArenaTask(void *arg);
    
    /**********************************************************************/
    /* Queue services                                                     */
    /**********************************************************************/
    /**
     * Write a message in a given queue
     * @param queue Queue identifier
     * @param msg Message to be stored
     */
    void WriteInQueue(RT_QUEUE *queue, Message *msg);
    
    /**
     * Read a message from a given queue, block if empty
     * @param queue Queue identifier
     * @return Message read
     */
    Message *ReadInQueue(RT_QUEUE *queue);

};

#endif // __TASKS_H__ 
