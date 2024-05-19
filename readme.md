# Dumber

Depot du projet de temps reel 4eme année au departement GEI de l'INSA Toulouse.

## Repertoires
- hardware : contient les plans pour la partie mecanique du robot et de son chargeur, ainsi que les plans de conception des PCB du robot, du chargeur, de l'adaptateur Xbee pour la raspberry  et les plans des CAP du robot
- software: rassemble les parties logicielles du robot, du chargeur, les bibliotheques et superviseur coté raspberry et l'interface Web
- doc: contient les sujets de TD et TP
- aruco_markers: Script de generation des tags (aruco) utilisés sur les robots



void Tasks::FindArene(void *arg){
    int rs;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

    // Synchronisation : attendre que toutes les tâches soient prêtes
    rt_sem_p(&sem_barrier, TM_INFINITE);

    // Configure la tâche en tant que périodique avec une période de 100 ms
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while(1) {
        // Attendre la période suivante
        rt_task_wait_period(NULL);

            // Prendre le sémaphore pour trouver l'arène
            rt_sem_p(&sem_findarene, TM_INFINITE);

            if (camstart == 1) {
                // Acquérir le mutex de la caméra
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);

                // Capture une image avec la caméra
                Img* image = new Img(camera.Grab());
                MessageImg* messageimg2 = new MessageImg(MESSAGE_CAM_IMAGE, image);

                // Cherche l'arène dans l'image
                arene = new Arena(image->SearchArena());

                if (arene->IsEmpty()) {
                    // Si aucune arène trouvée, envoyer un message NACK
                    WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_NACK));
                } else {
                    // Si arène trouvée, dessine l'arène et envoie l'image
                    image->DrawArena(*arene);
                    MessageImg* messageimg2 = new MessageImg(MESSAGE_CAM_IMAGE, image);
                    WriteInQueue(&q_messageToMon, messageimg2);
                    cout << "Answer Confirmed" << endl;
                }

                // Libère le mutex de la caméra
                rt_mutex_release(&mutex_camera);
        }
    }
}
