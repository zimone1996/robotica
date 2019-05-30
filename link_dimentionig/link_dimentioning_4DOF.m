%DIMENSIONAMENTO LINK -


%%
clear all
close all
clc

%Limiti di giunto del manipolatore
%estremi superiore e inferiore per il moto dei giunti
joint_lim=[deg2rad(-70) deg2rad(70);
    deg2rad(-70) deg2rad(70);
    deg2rad(0) deg2rad(140)
    deg2rad(-90) deg2rad(90)];
%Traiettoria di esempio nello spazio operativo
%possibile traiettoria per organo terminale; x e y per ogni istante di
%tempo
%meglio non inserire tutta la traiettoria ma 4 o 5 punti; punto iniziale
%finale e qualche intermedio

%% inserire valori da pianificazione di traiettoria
p=[ 25 30; %punto A (più lontano a sinistra)
    27.5  27.5;%prima pancia
];

%pianificazione dell'angolo
theta=[0.2618; %
       0.3927;
];
%% 

%Grandezza minima/massima dei link
link_lim=[7.5 7.5;
    9 20;
    3 20;
    3 20];

%Range di variazione dei link nel metodo di ottimizzazione
%valore minimo che algoritmo va a testare sulla base della posizione che mi
%serve
%Posso usare come funzione di costo la cui somma è minima (riduco materiale)
%Scelgo seconda funzione di costo per vedere quanto le lunghezze si
%discostano dalla media
resolution=0.5;

resolution_q4 = deg2rad(15);

%Vettori vuoti
links=[];
links_sum=[];
links_diff=[];
links_=[];

numero_di_verificate = 0;
count = 0;
%5 cicli for innestati per scorrere i vettoriq
%parametrizzare rispetto a q4
%aggiungo for che mi fa variare q4
for a1=link_lim(1,1):resolution:link_lim(1,2)
    for a2=link_lim(2,1):resolution:link_lim(2,2)
        for a3=link_lim(3,1):resolution:link_lim(3,2)
            for a4=link_lim(4,1):resolution:link_lim(4,2)
                for q4 = joint_lim(4,1):resolution_q4:joint_lim(4,2)
            %in ingresso vettori pianificazione, 4 possibili lunghezze e
            %poi limiti
            check=check_links_dimensions_4DoF(p,theta,q4,a1,a2,a3,a4,joint_lim);
            
            %Ciclo if
            if(check)
                numero_di_verificate = numero_di_verificate + 1
                %conservo le lunghezze per cui check è vero
                links=[links; a1, a2, a3,a4];
                %faccio somma dei link
                links_sum=[links_sum;a1+a2+a3+a4];
                %funzione di costo per vedere quanto lunghezze link si
                %discostano da media
                max_12=max(abs(a1-(a1+a2+a3+a4)/4),abs(a2-(a1+a2+a3+a4)/4));
                max_123=max(max_12,abs(a3-(a1+a2+a3+a4)/4));
                max_1234=max(max_123,abs(a4-(a1+a2+a3+a4)/4));
                links_diff=[links_diff;max_1234];
            end
            %tutte le combinazioni, anche quelle che non vanno bene
            links_=[links_; a1, a2, a3,a4];
                end
            
            end
          
        end
          %35
    end
     count = count +1 %35
end
 disp('fine ciclo a1'); %35

%Vengono applicate delle funzioni di costo
links_sum=links_sum-mean(links_sum);
links_diff=links_diff-mean(links_diff);
%normalizzo per il valore assoluto della differenza tra massimo e minimo
links_sum_norm=links_sum/abs((max(links_sum)-min(links_sum)));
links_diff_norm=links_diff/abs((max(links_diff)-min(links_diff)));

%funzione di costo
cost_function= links_sum_norm + links_diff_norm;

figure(1)
plot(links_sum,'-r','LineWidth',3)
hold on
plot(links_diff,'-b','LineWidth',3)

figure(2)
plot(cost_function,'-g','LineWidth',3)
hold on
plot(links_sum_norm,'-r','LineWidth',3)
hold on
plot(links_diff_norm,'-b','LineWidth',3)

%prendo minimo della funzione
[~, correct_ind] = min(cost_function);
%trovo i link che mi soddisfano le funzioni di costo
link=links(correct_ind,:)