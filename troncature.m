clear all

load('new_echelon10.mat')

consignelampe = zeros(length(new_echelon10.reponse.Time),1); % Contiendra consignelampe tronqué
reponse = zeros(length(new_echelon10.reponse.Time),1); % Contiendra reponse tronqué
j=1; 

for i = 1:length(new_echelon10.consignelampe.Time) % On parcourt toutes les lignes de la liste des timecodes de consignelampe
    if new_echelon10.reponse.Time(j)==new_echelon10.consignelampe.Time(i) % Si le timecode associé à la ligne i de la matrice de consignelampe est dans la matrice de reponse, alors
                
        consignelampe(j)=new_echelon10.consignelampe.Data(i); % Ajout de la valeur dans data1
        reponse(j)=new_echelon10.reponse.Data(j); % Ajout de la valeur dans data2
        
        j=j+1;
    end
end

clear i
clear j
clear new_echelon10

save('new_echelon10_tronque.mat')