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

    // Affiche le début de l'exécution de la fonction avec son nom pour le débogage
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

    // Barrière de synchronisation pour s'assurer que toutes les tâches sont en cours de démarrage
    rt_sem_p(&sem_barrier, TM_INFINITE);

    bool camOpen;  // Variable pour vérifier si la caméra est ouverte avec succès
    Message *msgSend;  // Pointeur pour envoyer des messages

    /**************************************************************************************/
    /* La tâche openCameraTask commence ici                                                */
    /**************************************************************************************/
    while (1) {
        // Attente de la libération du sémaphore pour ouvrir la caméra
        rt_sem_p(&sem_openCamera, TM_INFINITE);

        // Ouverture de la caméra avec une section critique protégée par un mutex
        rt_mutex_acquire(&mutex_cam, TM_INFINITE);  // Acquisition du mutex pour accéder à la caméra
        cam->Open();  // Ouverture de la caméra
        camOpen = cam->IsOpen();  // Vérifie si la caméra s'est ouverte avec succès
        rt_mutex_release(&mutex_cam);

        // Active l'envoi périodique d'images en signalant le sémaphore
        rt_sem_v(&sem_flowImage);

        // Si la caméra s'ouvre correctement, rien ne se passe ici pour l'instant
        /*if (camOpen) {
            // Code pour traiter le cas où la caméra est ouverte avec succès
        } else {
            // Envoi d'un message d'erreur en cas d'échec de l'ouverture de la caméra
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        }
        WriteInQueue(&q_messageToMon, msgSend);*/
    }
}


/**
 * @brief Thread closing the camera
 */
void Tasks::closeCameraTask(void *arg) {

    // Affiche le début de l'exécution de la fonction avec son nom pour le débogage
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

    // Barrière de synchronisation pour s'assurer que toutes les tâches sont en cours de démarrage
    rt_sem_p(&sem_barrier, TM_INFINITE);

    Message *msgSend;  // Pointeur pour envoyer des messages
    bool camClose;  // Variable pour vérifier si la caméra est fermée avec succès

    /**************************************************************************************/
    /* La tâche closeCameraTask commence ici                                               */
    /**************************************************************************************/

    while (1) {
        // Attend un signal pour fermer la caméra
        rt_sem_p(&sem_closeCamera, TM_INFINITE);

        // Fermeture de la caméra avec une section critique protégée par un mutex
        rt_mutex_acquire(&mutex_cam, TM_INFINITE);
        cam->Close();  // Commande pour fermer la caméra
        camClose = not(cam->IsOpen());  // Vérifie si la caméra est fermée avec succès
        rt_mutex_release(&mutex_cam);

        // Envoie un message d'acquittement si la caméra s'est fermée correctement
        if (camClose) {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        } else {
            // Envoie un message de non acquittement en cas d'échec de la fermeture
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        }
        WriteInQueue(&q_messageToMon, msgSend);  // Écrit le message dans la file de messages

        // Bloque le flot d'images
        rt_sem_p(&sem_flowImage, TM_INFINITE);
    }
}


/**
 * @brief Thread sending images from the camera to the monitor
 */
void Tasks::sendImageToMonitorTask(void *arg){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Barrière de synchronisation pour s'assurer que toutes les tâches sont en cours de démarrage
    rt_sem_p(&sem_barrier, TM_INFINITE);

    MessageImg *msgSend;  // Pointeur pour envoyer des messages contenant des images
    /**************************************************************************************/
    /* La tâche sendImageToMonitor commence ici                                            */
    /**************************************************************************************/

    // Cette tâche est périodique toutes les 100ms
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        // Attend la prochaine période de 100ms
        rt_task_wait_period(NULL);

        // Attend un signal pour permettre l'envoi d'images
        rt_sem_p(&sem_flowImage, TM_INFINITE);
        rt_mutex_acquire(&mutex_cam, TM_INFINITE);
        if (cam->IsOpen()) {
            // Acquisition de l'image de la caméra
            Img img = cam->Grab();

            rt_mutex_acquire(&mutex_arenaOK, TM_INFINITE);
            if (arenaOK) {
                // Si l'arène est validée, dessine l'arène sur l'image avant envoi
                rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                img.DrawArena(arena);
                msgSend = new MessageImg(MESSAGE_CAM_IMAGE, &img);
                rt_mutex_release(&mutex_arena);
            } else {
                // Si l'arène n'est pas validée, envoie l'image sans dessin de l'arène
                msgSend = new MessageImg(MESSAGE_CAM_IMAGE, &img);
            }
            rt_mutex_release(&mutex_arenaOK);

            // Écrit le message contenant l'image dans la file de messages
            WriteInQueue(&q_messageToMon, msgSend);
        }
        rt_mutex_release(&mutex_cam);
        rt_sem_v(&sem_flowImage);  // Libère le sémaphore pour permettre l'envoi périodique d'images
    }
}



/**
 * @brief Thread managing the arena
 */
void Tasks::manageArenaTask(void *arg){
    // Affiche le début de l'exécution de la fonction avec son nom pour le débogage
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

    Img* last_image;  // Pointeur vers la dernière image capturée
    Arena a;  // Objet Arena pour stocker l'arène détectée
    MessageImg* msgImg;  // Pointeur pour envoyer des messages contenant des images
    // Barrière de synchronisation pour s'assurer que toutes les tâches sont en cours de démarrage
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* La tâche manageArenaTask commence ici                                               */
    /**************************************************************************************/

    // Attend le signal pour lancer la recherche de l'arène
    rt_sem_p(&sem_searchArena, TM_INFINITE);
    // Arrêt de l'envoi périodique d'images
    rt_sem_p(&sem_flowImage, TM_INFINITE);

    // Initialisation de l'état de l'arène à non trouvé
    rt_mutex_acquire(&mutex_arenaOK, TM_INFINITE);
    arenaOK = false;
    rt_mutex_release(&mutex_arenaOK);

    // Acquisition de la dernière image capturée par la caméra avant la demande de recherche d'arène
    rt_mutex_acquire(&mutex_cam, TM_INFINITE);
    last_image = new Img(cam->Grab());
    rt_mutex_release(&mutex_cam);

    // Recherche de l'arène dans l'image capturée
    rt_mutex_acquire(&mutex_arena, TM_INFINITE);
    arena = last_image->SearchArena();
    a = arena;
    rt_mutex_release(&mutex_arena);

    if (!arena.IsEmpty()) {
        cout << "Arena trouvée" << endl;

        // Dessine l'arène sur l'image capturée et envoie l'image au moniteur
        last_image->DrawArena(a);
        msgImg = new MessageImg(MESSAGE_CAM_IMAGE, last_image);
        WriteInQueue(&q_messageToMon, msgImg);

        // Attend que l'utilisateur valide ou invalide l'arène
        rt_sem_p(&sem_arenaAns, TM_INFINITE);
        rt_mutex_acquire(&mutex_arenaOK, TM_INFINITE);
        if (arenaOK) {
            cout << "Arena confirmed" << endl;
        } else {
            cout << "Arena infirmed" << endl;
        }
        rt_mutex_release(&mutex_arenaOK);

        // Libère le sémaphore pour que l'envoi d'images reprenne
        rt_sem_v(&sem_flowImage);
    } else { // La recherche d'arène n'ayant rien donné, on affiche un message dans la console de lancement et on envoie un message de non acquittement
        cout << "Arena not found" << endl;
        WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_NACK));
    }
}


/**
 * @brief Écrit un message dans une file de messages donnée
 * @param queue Identifiant de la file de messages
 * @param msg Message à stocker
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;  // Variable pour stocker les erreurs éventuelles

    // Essaye d'écrire le message dans la file de messages
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof((const void *) &msg), Q_NORMAL)) < 0) {
        // Si une erreur survient, affiche un message d'erreur
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        // Lance une exception en cas d'erreur
        throw std::runtime_error{"Error in write in queue"};
    }
}


/**
 * @brief Lit un message depuis une file de messages donnée, bloque si vide
 * @param queue Identifiant de la file de messages
 * @return Message lu
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;  // Variable pour stocker les erreurs éventuelles
    Message *msg;  // Pointeur pour stocker le message lu

    // Essaye de lire un message depuis la file de messages
    if ((err = rt_queue_read(queue, &msg, sizeof((void*) &msg), TM_INFINITE)) < 0) {
        // Si une erreur survient, affiche un message d'erreur
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        // Lance une exception en cas d'erreur
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        // Affiche le message lu pour le débogage (actuellement commenté)
        cout << "@msg :" << msg << endl << flush;
    } /**/

    // Retourne le message lu
    return msg;
}