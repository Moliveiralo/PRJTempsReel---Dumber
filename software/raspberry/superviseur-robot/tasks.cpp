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
#define PRIORITY_TSTOPROBOT 23
#define PRIORITY_TCHECKCOMROBOT 22


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
    if (err = rt_mutex_create(&mutex_cam, NULL)) {
	cerr << "Error mutex create : " << strerror(-err) << endl << flush;
	exit (EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
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
    if (err = rt_mutex_create(&mutex_errorRobot, NULL)) {
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
    if (err = rt_sem_create(&sem_closeRobot, NULL, 0, S_FIFO)) {
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
    if (err = rt_task_create(&th_stopRobot, "th_stopRobot", 0, PRIORITY_TSTOPROBOT, 0)){
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_checkRobotCommunication, "th_checkRobotCommunication", 0, PRIORITY_TCHECKCOMROBOT, 0)){
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
    if (err = rt_task_start(&th_stopRobot, (void(*)(void*)) &Tasks::closeRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_checkRobotCommunication, (void(*)(void*)) &Tasks::CheckRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close(); // Fermeture de la connexion avec le moniteur
    robot.Close();   // Fermeture de la communication avec le robot
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier); // Libération du sémaphore de synchronisation des tâches
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
    // Synchronisation barrière (attente que toutes les tâches démarrent)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* La tâche receiveFromMonitor commence ici                                            */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE); // Attendre que le serveur soit prêt
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read(); // Lire le message reçu depuis le moniteur
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush; // Afficher le message

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            // Si le message indique que la connexion au moniteur est perdue
            delete(msgRcv);
            exit(-1); // Quitter le programme
        } 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            // Si le message indique que la communication avec le robot doit être ouverte
            rt_sem_v(&sem_openRobotCommunication); // Libérer le sémaphore d'ouverture de la comm avec le robot
        } 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            // Si le message indique de démarrer le robot sans watchdog
            rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
            activateWatchdog = false; // Désactiver le watchdog
            rt_mutex_release(&mutex_watchdog);
            cout << "Demarrage du robot sans watchdog" << endl;
            rt_sem_v(&sem_startRobot); // Libérer le sémaphore pour démarrer le robot
        }
        else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            // Si le message indique de démarrer le robot avec watchdog
            rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
            activateWatchdog = true; // Activer le watchdog
            rt_mutex_release(&mutex_watchdog);
            cout << "Demarrage du robot avec watchdog" << endl;
            rt_sem_v(&sem_startRobot); // Libérer le sémaphore pour démarrer le robot
        }
        else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {
            // Si le message indique un mouvement du robot (avant, arrière, gauche, droite, ou stop)
            rt_mutex_acquire(&mutex_moveRobot, TM_INFINITE);
            moveRobot = msgRcv->GetID(); // Mettre à jour la commande de mouvement
            rt_mutex_release(&mutex_moveRobot);
        } 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)){
            // Si le message demande d'obtenir l'état de la batterie
            rt_mutex_acquire(&mutex_battery, TM_INFINITE);
            getBattery = true; // Indiquer que le niveau de batterie doit être vérifié
            rt_mutex_release(&mutex_battery);
        } 
        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)){
            // Si le message demande l'ouverture de la caméra
            rt_sem_v(&sem_openCamera);
            cout << "demande ouverture camera" << endl;
        } 
        else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)){
            // Si le message demande la fermeture de la caméra
            rt_sem_v(&sem_closeCamera);
            cout << "demande fermeture camera" << endl;
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)){
            // Si le message demande la recherche de l'arène
            rt_sem_v(&sem_searchArena);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)){
            // Si le message confirme l'arène qui a été trouvée
            cout << "arena confirmed" << endl;
            rt_mutex_acquire(&mutex_arenaOK, TM_INFINITE);
            arenaOK = true; // Marquer l'arène comme trouvée
            rt_mutex_release(&mutex_arenaOK);
            rt_sem_v(&sem_arenaAns); // Libérer le sémaphore de réponse
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)){
            // Si le message infirme l'arène trouvée
            cout << "arena infirmed" << endl;
            rt_mutex_acquire(&mutex_arenaOK, TM_INFINITE);
            arenaOK = false; // Marquer l'arène comme non validée
            rt_mutex_release(&mutex_arenaOK);
            rt_sem_v(&sem_arenaAns); // Libérer le sémaphore de réponse
        }
        delete(msgRcv); // Libérer la mémoire du message (doit être fait manuellement)
    }
}


/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::openRobotCommunication(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Barrière de synchronisation (attente que toutes les tâches démarrent)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* La tâche openRobotCommunication commence ici                                       */
    /**************************************************************************************/
    while (1) {
        // Attendre le sémaphore indiquant l'ouverture de la communication avec le robot
        rt_sem_p(&sem_openRobotCommunication, TM_INFINITE);
        cout << "Open serial com (";

        // Acquisition du mutex pour accéder à l'objet robot
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open(); // Tentative d'ouverture de la communication série avec le robot
        rt_mutex_release(&mutex_robot); // Libération du mutex
        cout << status; // Affichage du statut de l'opération d'ouverture
        cout << ")" << endl << flush;

        Message *msgSend;
        if (status < 0) {
            // Si l'ouverture a échoué, créer un message NACK (Negative Acknowledgment)
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            // Si l'ouverture a réussi, créer un message ACK (Acknowledgment)
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        // Envoyer le message dans la file de messages à destination du moniteur
        // (msgSend sera supprimé par la fonction sendToMon)
        WriteInQueue(&q_messageToMon, msgSend);
    }
}


void Tasks::closeRobotTask(void *arg) {
    int status;
    int com_error;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Barrière de synchronisation (attente que toutes les tâches démarrent)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* La tâche closeComRobot commence ici                                                */
    /**************************************************************************************/

    while (1) {
        // Attendre le sémaphore indiquant la fermeture de la communication avec le robot
        rt_sem_p(&sem_closeRobot, TM_INFINITE);

        // Acquisition du mutex pour lire l'état des erreurs de communication avec le robot
        rt_mutex_acquire(&mutex_errorRobot, TM_INFINITE);
        com_error = errorRobot; // Récupération de la valeur de l'erreur de communication
        rt_mutex_release(&mutex_errorRobot); // Libération du mutex

        // Acquisition du mutex pour accéder à l'objet robot
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        cout << "Close com (" << flush;
        status = robot.Close(); // Tentative de fermeture de la communication série avec le robot
        rt_mutex_release(&mutex_robot); // Libération du mutex
        cout << status; // Affichage du statut de l'opération de fermeture
        cout << ")" << endl << flush;

        // Mise à jour de l'état indiquant que le robot n'est plus démarré
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotStarted = 0; // Marquer le robot comme arrêté
        rt_mutex_release(&mutex_robotStarted);

        // Mise à jour de la commande de mouvement du robot pour arrêter le robot
        rt_mutex_acquire(&mutex_moveRobot, TM_INFINITE);
        moveRobot = MESSAGE_ROBOT_STOP; // Marquer le robot comme étant en arrêt
        rt_mutex_release(&mutex_moveRobot);
    }
}


/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Barrière de synchronisation (attente que toutes les tâches démarrent)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* La tâche startRobot commence ici                                                   */
    /**************************************************************************************/
    while (1) {
        Message *msgSend;

        // Attendre que le sémaphore pour démarrer le robot soit disponible
        rt_sem_p(&sem_startRobot, TM_INFINITE);

        // si le watchdog est désactivé
        if (activateWatchdog == false) {
            // Démarrer le robot sans watchdog
            cout << "Start robot without watchdog (";
            // Acquisition du mutex pour accéder à l'objet robot en toute sécurité
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            // Envoi de la commande pour démarrer le robot sans watchdog et récupération de la réponse
            msgSend = robot.Write(robot.StartWithoutWD());
            // Libération du mutex après l'opération
            rt_mutex_release(&mutex_robot);
        } else { // Sinon
            // Démarrer le robot avec watchdog
            cout << "Start robot with watchdog (";
            // Acquisition du mutex pour accéder à l'objet robot en toute sécurité
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            // Envoi de la commande pour démarrer le robot avec watchdog et récupération de la réponse
            msgSend = robot.Write(robot.StartWithWD());
            // Libération du mutex après l'opération
            rt_mutex_release(&mutex_robot);
        }

        // Affichage de l'ID du message reçu du robot
        cout << msgSend->GetID();
        cout << ")" << endl;
        cout << "MovementRobot answer: " << msgSend->ToString() << endl << flush;

        // Envoi du message dans la file de messages destinée au moniteur
        // (msgSend sera supprimé par la fonction sendToMon après traitement)
        WriteInQueue(&q_messageToMon, msgSend);

        // Vérifie si la réponse est un accusé de réception (ACK)
        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            // Acquisition du mutex pour mettre à jour l'état du robot en toute sécurité
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            // Met à jour l'état indiquant que le robot est démarré
            robotStarted = 1;
            // Libération du mutex après la mise à jour
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
    // Barrière de synchronisation (attente que toutes les tâches démarrent)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* La tâche commence ici                                                              */
    /**************************************************************************************/
    
    // Définir la tâche comme périodique avec une période de 100 ms
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    
    while (1) {
        // Attendre la prochaine période
        rt_task_wait_period(NULL);
        
        cout << "Periodic movement update";
        
        // Récupération du statut de démarrage du robot
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        if (rs == 1) {
            rt_mutex_acquire(&mutex_moveRobot, TM_INFINITE);
            cpmoveRobot = moveRobot;
            rt_mutex_release(&mutex_moveRobot);
            
            // Afficher la commande de mouvement actuelle
            cout << " moveRobot: " << cpmoveRobot;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpmoveRobot)); // Envoi de la commande
            rt_mutex_release(&mutex_robot);
        }
        
        cout << endl << flush;
    }
}


/**
* @brief Thread handling the displaying of the battery level
*/
void Tasks::manageBatteryLevelTask(void *arg) {    
    Message *msgSend, *msgSend2; 
    int robotS; 
    bool getB; 
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Barrière de synchronisation (attente que toutes les tâches démarrent)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* La tâche commence ici                                                              */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 500000000); // Tâche périodique de 500ms

    while (1) {
        // Attendre la prochaine période (500 ms)
        rt_task_wait_period(NULL);
        cout << "Periodic update of the battery level" << endl;
        
        // Vérifier si le robot est démarré
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotS = robotStarted; 
        rt_mutex_release(&mutex_robotStarted);
        
        // Vérifier si la demande de récupération du niveau de batterie est active
        rt_mutex_acquire(&mutex_battery, TM_INFINITE);
        getB = getBattery;
        rt_mutex_release(&mutex_battery);
        
        cout << "robotS " << robotS << " -- getB " << getB << endl;
        
        // Si le robot est démarré et la demande de récupération de la batterie est active
        if (robotS && getB) {
            // Récupérer le niveau de la batterie du robot
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.GetBattery();
            msgSend2 = robot.Write(msgSend);
            rt_mutex_release(&mutex_robot);
            
            // Envoyer le niveau de batterie au moniteur
            WriteInQueue(&q_messageToMon, msgSend2);
            
            cout << msgSend2 << endl;
            
            // Remettre la demande de récupération de la batterie à false
            // Cela permet de ne pas prendre en compte les demandes répétées non décochées
            rt_mutex_acquire(&mutex_battery, TM_INFINITE);
            getBattery = false;
            rt_mutex_release(&mutex_battery);
        }
    }
}


/**
 * @brief Thread handling the communication with the robot.
 */
void Tasks::CheckRobotTask(void *arg) {
    bool rs, wd;
    int err_cmp = 0;
    Message *status;

    cout << "Start CheckRobotTask" << __PRETTY_FUNCTION__ << endl << flush;
    // Barrière de synchronisation (attente que toutes les tâches démarrent)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* La tâche commence ici                                                              */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000); // Tâche périodique de 100ms

    while (1) {
        // Attendre la prochaine période (100 ms)
        rt_task_wait_period(NULL);

        // Vérifier si le robot est démarré
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);

        // Vérifier si le watchdog est activé
        rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
        wd = activateWatchdog;
        rt_mutex_release(&mutex_watchdog);

        // Si le robot est démarré et le watchdog activé
        if (rs && wd) {
            // Envoyer un ping au robot pour vérifier sa réponse
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            status = robot.Write(new Message(MESSAGE_ROBOT_PING));
            rt_mutex_release(&mutex_robot);

            // Si plus de 3 erreurs de communication consécutives
            if (err_cmp >= 3) {
                err_cmp = 0;
                cout << "------Message : The communication with the robot has ended.------" << endl;

                // Mettre à jour l'état d'erreur de la communication
                rt_mutex_acquire(&mutex_errorRobot, TM_INFINITE);
                errorRobot = 1;
                rt_mutex_release(&mutex_errorRobot);

                // Mettre à jour l'état du robot comme arrêté
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0;
                rt_mutex_release(&mutex_robotStarted);

                cout << "------Message : Closing the communication with the robot------" << endl;
                // Libérer le sémaphore pour fermer la communication avec le robot
                rt_sem_v(&sem_closeRobot);
                // Envoyer un message d'erreur de communication au moniteur
                WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_COM_ERROR));
            }
            // Si une erreur de communication est détectée
            else if (status->CompareID(MESSAGE_ANSWER_COM_ERROR)) {
                err_cmp++;
                cout << " status :  " << status->GetID() << endl;
                cout << " err_cmp: " << err_cmp << endl;
                cout << " errorRobot :" << errorRobot << endl;
                cout << " rs : " << robotStarted << endl << flush;
            }
            // Si le ping réussit, réinitialiser le compteur d'erreurs
            else {
                err_cmp = 0;
                cout << " err_cmp: " << err_cmp << endl;
            }
        }
    }
}


/**
 * @brief Thread opening the camera
 */
void Tasks::openCameraTask(void *arg) {

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    bool camOpen; 
    Message * msgSend; 
    /**************************************************************************************/
    /* The task openCameraTask starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openCamera, TM_INFINITE);

        // ouverture de la camera           
        rt_mutex_acquire(&mutex_cam, TM_INFINITE);
        cam->Open();
        camOpen = cam->IsOpen(); 
        rt_mutex_release(&mutex_cam);  
        
        // On active l'envoi d'image periodique via le semaphore 
        rt_sem_v(&sem_flowImage);
        
        /*if (camOpen) {
            
        } else {
            // Envoi d'un message d'erreur en cas d'echec
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } 
        WriteInQueue(&q_messageToMon, msgSend);*/
    }
}

/**
 * @brief Thread closing the camera
 */
void Tasks::closeCameraTask(void *arg) {

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    Message *msgSend;
    bool camClose; 
    
    /**************************************************************************************/
    /* The task closeCameraTask starts here                                                  */
    /**************************************************************************************/
    
    
    while (1) {
        rt_sem_p(&sem_closeCamera, TM_INFINITE);
 
        // fermeture de la camera           
        rt_mutex_acquire(&mutex_cam, TM_INFINITE);
        cam->Close();    
        camClose = not(cam->IsOpen()); 
        rt_mutex_release(&mutex_cam);
        
        if (camClose) {
            // Envoi du message d'acquittement
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        } else {
            // Envoi du message de non acquittement
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        }
        WriteInQueue(&q_messageToMon, msgSend);
        
        rt_sem_p(&sem_flowImage, TM_INFINITE); // Permet d'arreter l'envoi periodique d'image
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
            // Acquisition de l'image   
            Img img = cam->Grab();            
            
            rt_mutex_acquire(&mutex_arenaOK, TM_INFINITE); 
            if (arenaOK){
                // Envoi de l'image avec le dessin de l'arene
                rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                img.DrawArena(arena);
                msgSend = new MessageImg(MESSAGE_CAM_IMAGE, &img);
                rt_mutex_release(&mutex_arena);
            } else {
                // Envoi de l'image sans l'arene
                msgSend = new MessageImg(MESSAGE_CAM_IMAGE,&img);; 
            }
            rt_mutex_release(&mutex_arenaOK);        
            

            WriteInQueue(&q_messageToMon, msgSend);
        }
        rt_mutex_release(&mutex_cam);
        rt_sem_v(&sem_flowImage); 
    }


}

//RT_SEM sem_arenaAns;



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
    
    rt_mutex_acquire(&mutex_arenaOK, TM_INFINITE); 
    arenaOK = false; 
    rt_mutex_release(&mutex_arenaOK); 
    
    // Acquisition de la derniere image envoyee a la camera avanrt la demande de recherche d'arene
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
        
        // On libere le semaphore pour que l'envoi d'image reprend
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
