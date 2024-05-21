clear all

load('new_echelon5_tronque.mat');
consignelampe_echelon5=consignelampe;
reponse_echelon5=reponse;

load('new_echelon8_tronque.mat');
consignelampe_echelon8=consignelampe;
reponse_echelon8=reponse;

load('new_echelon10_tronque.mat');
consignelampe_echelon10=consignelampe;
reponse_echelon10=reponse;

clear consignelampe
clear reponse

save('toutes_les_reponses_tronque.mat');