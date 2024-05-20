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

// Path dans la raspi: cd .netbeans/remote/10.105.0.147/insa-10580-Linux-x86_64/home/inaji/AE/S8/tempsReel/dumber/software/raspberry/superviseur-robot/dist/Debug__RPI_/GNU-Linux

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TopenRobotCommunication 20
#define PRIORITY_TmoveRobot 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TreceiveFromMonitor 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TMANAGEBATTERYLEVEL 20
#define PRIORITY_TOPENCAMERA 23
#define PRIORITY_TCLOSECAMERA 22
#define PRIORITY_TSENDIMAGETOMONITOR 21
#define PRIORITY_TMANAGEARENA 22

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_moveRobot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_battery, NULL)) {
	cerr << "Error mutex create : " << strerror(-err) << endl << flush;
	exit (EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_cameraOpen, NULL)) {
	cerr << "Error mutex create : " << strerror(-err) << endl << flush;
	exit (EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_cam, NULL)) {
	cerr << "Error mutex create : " << strerror(-err) << endl << flush;
	exit (EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create : " << strerror(-err) << endl << flush;
        exit (EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_stopSearchArena, NULL)) {
        cerr << "Error mutex create : " << strerror(-err) << endl << flush;
        exit (EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_stopSendImageFromArenaSearch, NULL)) {
        cerr << "Error mutex create : " << strerror(-err) << endl << flush;
        exit (EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arenaOK, NULL)) {
        cerr << "Error mutex create : " << strerror(-err) << endl << flush;
        exit (EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_watchdog, NULL)) {
        cerr << "Error mutex create : " << strerror(-err) << endl << flush;
        exit (EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openRobotCommunication, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_getBattery, NULL, 0, S_FIFO)) {
	cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
	exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openCamera, NULL, 0, S_FIFO)) {
	cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
	exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_closeCamera, NULL, 0, S_FIFO)) {
	cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
	exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startSendingImage, NULL, 0, S_FIFO)) {
	cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
	exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_sendImageFromArenaSearch, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_searchArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_arenaAns, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_flowImage, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_watchdog, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMonitor, "th_sendToMonitoritoritor", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMonitor, "th_receiveFromMonitor", 0, PRIORITY_TreceiveFromMonitor, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openRobotCommunication, "th_openRobotCommunication", 0, PRIORITY_TopenRobotCommunication, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_moveRobot, "th_moveRobot", 0, PRIORITY_TmoveRobot, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_manageBatteryLevel, "th_manageBatteryLevel", 0, PRIORITY_TMANAGEBATTERYLEVEL, 0)){
	cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openCamera, "th_openCamera", 0, PRIORITY_TOPENCAMERA, 0)){
	cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_closeCamera, "th_closeCamera", 0, PRIORITY_TCLOSECAMERA, 0)){
	cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendImageToMonitor, "th_sendImageToMonitor", 0, PRIORITY_TMANAGEBATTERYLEVEL, 0)){
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_manageArena, "th_manageArena", 0, PRIORITY_TMANAGEBATTERYLEVEL, 0)){
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMonitor, (void(*)(void*)) & Tasks::sendToMonitorTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMonitor, (void(*)(void*)) & Tasks::receiveFromMonitorTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openRobotCommunication, (void(*)(void*)) & Tasks::openRobotCommunication, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_moveRobot, (void(*)(void*)) & Tasks::moveRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_manageBatteryLevel, (void(*)(void*)) &Tasks::manageBatteryLevelTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openCamera, (void(*)(void*)) &Tasks::openCameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_closeCamera, (void(*)(void*)) &Tasks::closeCameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendImageToMonitor, (void(*)(void*)) &Tasks::sendImageToMonitorTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_manageArena, (void(*)(void*)) &Tasks::manageArenaTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::sendToMonitorTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        //cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        //cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::receiveFromMonitorTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMonitor starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            exit(-1);
        } 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openRobotCommunication);
        } 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            // Démarrer le robot sans watchdog
            rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
            activateWatchdog = false;
            rt_mutex_release(&mutex_watchdog);
            cout << "Demarrage du robot sans watchdog" << endl;
            rt_sem_v(&sem_startRobot);
        }
        else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            // Démarrer le robot avec watchdog
            rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
            activateWatchdog = true;
            rt_mutex_release(&mutex_watchdog);
            cout << "Demarrage du robot avec watchdog" << endl;
            rt_sem_v(&sem_startRobot);
//            rt_sem_v(&sem_watchdog);
        }
        else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_moveRobot, TM_INFINITE);
            moveRobot = msgRcv->GetID();
            rt_mutex_release(&mutex_moveRobot);
        } 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)){
            // Quand la case get batterie est coche, on passe a true la variable get batterie
            //rt_sem_v(&sem_getBattery);
            rt_mutex_acquire(&mutex_battery, TM_INFINITE);  
            getBattery = true; 
            rt_mutex_release(&mutex_battery); 
        } 
        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)){
            //Quand la case open camera est cochee, on libere le semaphore d'ouverture de camera
            rt_sem_v(&sem_openCamera);
            cout << "demande ouverture camera" << endl; 
            
        } 
        else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)){
            //Quand la case open camera est decochee, on libere le semaphore de fermeture de camera
            rt_sem_v(&sem_closeCamera);
            cout << "demande fermeture camera" << endl; 
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)){
            rt_sem_v(&sem_searchArena); 
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)){
            cout << "arena confirmed" << endl; 
            rt_mutex_acquire(&mutex_arenaOK, TM_INFINITE); 
            arenaOK = true;
            rt_mutex_release(&mutex_arenaOK); 
            rt_sem_v(&sem_arenaAns); // il faut debloquer le semaphore apres
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)){
            cout << "arena infirmed" << endl;  
            rt_mutex_acquire(&mutex_arenaOK, TM_INFINITE); 
            arenaOK = false;
            rt_mutex_release(&mutex_arenaOK); 
            rt_sem_v(&sem_arenaAns);
        }
        delete(msgRcv); // must be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::openRobotCommunication(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openRobotCommunication starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openRobotCommunication, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

//        Message * msgSend;
//        rt_sem_p(&sem_startRobot, TM_INFINITE);
//        cout << "Start robot without watchdog (";
//        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
//        msgSend = robot.Write(robot.StartWithoutWD());
//        rt_mutex_release(&mutex_robot);
//        cout << msgSend->GetID();
//        cout << ")" << endl;
//
//        cout << "movementRobot answer: " << msgSend->ToString() << endl << flush;
//        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon
//
//        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
//            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
//            robotStarted = 1;
//            rt_mutex_release(&mutex_robotStarted);
//        }
        Message *msgSend;

        // Attendre que le sémaphore pour démarrer le robot soit disponible
        rt_sem_p(&sem_startRobot, TM_INFINITE);

        if (activateWatchdog == false) {
            // Démarrer le robot sans watchdog
            cout << "Start robot without watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithoutWD());
            rt_mutex_release(&mutex_robot);
        } else {
            // Démarrer le robot avec watchdog
            cout << "Start robot with watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithWD());
            rt_mutex_release(&mutex_robot);
        }

        // Afficher l'ID et la réponse du mouvement
        cout << msgSend->GetID();
        cout << ")" << endl;
        cout << "MovementRobot answer: " << msgSend->ToString() << endl << flush;

        // Envoyer le message dans la file de messages (msgSend sera supprimé par sendToMon)
        WriteInQueue(&q_messageToMon, msgSend);

        // Si la réponse est un accusé de réception, mettre à jour l'état du robot comme démarré
        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::moveRobotTask(void *arg) {
    int rs;
    int cpmoveRobot;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_moveRobot, TM_INFINITE);
            cpmoveRobot = moveRobot;
            rt_mutex_release(&mutex_moveRobot);
            
            cout << " moveRobot: " << cpmoveRobot;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpmoveRobot));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}

/**
* @brief Thread handling the displaying of the battery level
*/
void Tasks::manageBatteryLevelTask(void *arg){    
    
    Message *msgSend, *msgSend2; 
    int robotS; 
    bool getB; 
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 500000000); // Tache periodique de 500ms

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic update of the battery level" << endl;
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotS=robotStarted; 
        rt_mutex_release(&mutex_robotStarted);
        
        rt_mutex_acquire(&mutex_battery, TM_INFINITE);
        getB=getBattery;
        rt_mutex_release(&mutex_battery);
        
        cout << "robotS " << robotS << " -- getB " << getB << endl;
        
        // Si le robot est demarre et la case getBattery cochee
        if (robotS && getB){
            // On recupere le niveau de la batterie du robot
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.GetBattery();
            msgSend2 = robot.Write(msgSend);
            rt_mutex_release(&mutex_robot);
            
            // On envoie le niveau de batterie au moniteur
            WriteInQueue(&q_messageToMon, msgSend2);
            
            cout << msgSend2 << endl;
            
            // On remet a false get battery (permet de ne prendre en compte le decochage)
            rt_mutex_acquire(&mutex_battery, TM_INFINITE);
            getBattery = false;
            rt_mutex_release(&mutex_battery);
        }
    }
}

void Tasks::HandleLostCommunication() {

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

    // Synchronisation : attendre que toutes les tâches soient prêtes
    rt_sem_p(&sem_barrier, TM_INFINITE);
    // Déconnecter la caméra

    rt_mutex_acquire(&mutex_camera, TM_INFINITE);
    camera.Close();
    camstart = 0;
    rt_mutex_release(&mutex_camera);


    //Arrêter le robot
    rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    robot.Close();
    rt_mutex_release(&mutex_robot);

    // Mettre à jour l'état du robot comme non démarré
    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
    robotStarted = 0;
    rt_mutex_release(&mutex_robotStarted);


/**
 * @brief Thread opening the camera
 */
void Tasks::openCameraTask(void *arg) {

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openCameraTask starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openCamera, TM_INFINITE);
        cout << "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb" << endl;
        // ouverture de la camera           
        rt_mutex_acquire(&mutex_cam, TM_INFINITE);
        cam->Open();
        rt_mutex_release(&mutex_cam);  
        
        // On active l'envoi d'image periodique via le semaphore 
        rt_sem_v(&sem_flowImage); 
    }
}

/**
 * @brief Thread closing the camera
 */
void Tasks::closeCameraTask(void *arg) {

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task closeCameraTask starts here                                                  */
    /**************************************************************************************/
    
    
    while (1) {
        rt_sem_p(&sem_closeCamera, TM_INFINITE);
        cout << "on est dans close camera" << endl; 
        // On arrete l'envoi d'image en bloquant le semaphore 
        
        cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << endl; 
        // fermeture de la camera           
        rt_mutex_acquire(&mutex_cam, TM_INFINITE);
        cam->Close(); 
         
        rt_mutex_release(&mutex_cam);
        cout << "cccccccccccccccccccccccccccccccccccccccccccccccccc" << endl; 
    }
}

/**
 * @brief Thread sending images from the camera to the monitor
 */
void Tasks::sendImageToMonitorTask(void *arg){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;   
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    MessageImg * msgSend;
    /**************************************************************************************/
    /* The task sendImageToMonitor starts here                                            */
    /**************************************************************************************/

    rt_task_set_periodic(NULL, TM_NOW, 100000000); // Cette tâche est périodique toutes les 100ms

    while(1){
        rt_task_wait_period(NULL);
        
        rt_sem_p(&sem_flowImage, TM_INFINITE); 
        rt_mutex_acquire(&mutex_cam, TM_INFINITE);
        if (cam->IsOpen()){
            Img img = cam->Grab();

            msgSend = new MessageImg(MESSAGE_CAM_IMAGE,&img);

            WriteInQueue(&q_messageToMon, msgSend);
        }
        rt_mutex_release(&mutex_cam);
        rt_sem_v(&sem_flowImage); 
    }


}

//RT_SEM sem_sendImageFromArenaSearch;
//RT_SEM sem_arenaAns;
//RT_MUTEX mutex_stopSearchArena;
//RT_MUTEX mutex_stopSendImageFromArenaSearch;

    monitor.AcceptClient();

    // Afficher un message sur la console
    cout << "System reset to initial state due to communication loss" << endl;
}

/**
 * @brief Thread managing the arena
 */
void Tasks::manageArenaTask(void *arg){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    
    Img* last_image;
    Arena a ;
    MessageImg* msgImg;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task manageArenaTask starts here                                            */
    /**************************************************************************************/

    rt_sem_p(&sem_searchArena, TM_INFINITE); // Cette tâche restera bloquée tant que l'on ne lance pas la recherche de l'arène
    rt_sem_p(&sem_flowImage, TM_INFINITE); // Permet d'arreter l'envoi periodique d'image
    
    // Acquisition de la derniere ilmage envoyee a la camera avanrt la demande de recherche d'arene
    rt_mutex_acquire(&mutex_cam, TM_INFINITE);
    last_image = new Img(cam->Grab());
    rt_mutex_release(&mutex_cam);
    
    rt_mutex_acquire(&mutex_arena, TM_INFINITE);
    arena=last_image->SearchArena();
    a = arena;
    rt_mutex_release(&mutex_arena);
    
    if (!arena.IsEmpty()) {
        cout << "Arena trouvee" << endl;
        
        // On dessine l'arene sur l'image de la camera que l'on envoie au moniteur
        last_image->DrawArena(a);
        msgImg = new MessageImg(MESSAGE_CAM_IMAGE, last_image);
        WriteInQueue(&q_messageToMon, msgImg);
        
        rt_sem_p(&sem_arenaAns, TM_INFINITE); // On attend que l'utilisateur valide ou invalide l'arene
        rt_mutex_acquire(&mutex_arenaOK, TM_INFINITE); 
        if (arenaOK){
            cout << "Arena confirmed" << endl; 
        } else {
            cout << "Arena infirmed" << endl; 
        }
        rt_mutex_release(&mutex_arenaOK);        
        rt_sem_v(&sem_flowImage);
    }
    else {
        cout << "Arena not found" << endl; 
        WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_NACK));
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}
