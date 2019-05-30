Ridondanza...
Prima funzione calcola la traiettoria (noi già ce l'abbiamo)
Seconda funzione calcola la cinematica inversa: nell'esercitazione fatta in aula il task era quello di evitare un osacolo; il nostro è invece quello di evitare singolarità cinematica attraverso la funzione di costo a pagina 127 (misura di manipolabiltà).
La formula per algoritmo di inversione è quella riportata a pagina 135 dove al posto qdot bisognerà inserire il funzionale di costo. 


Controllo:
Inizializzazione dinamica: utile per il calcolo di B C e g presenti nell'equazione della dinamica (script Calcolo_matrici_dinamica).L'uscita saranno tre matrici simboliche che isonerà inserire in dynamic_model_3dof che restituisce l'accelerazione del sistema attarverso la dinamica diretta.

Test dynamic model: testa il manipolatore senza controllo con gravità che agisce su asse y.
Test_PIDcontroller: testa il PID per il raggiungimento della posizione desiderata.
Symulation_3dof: simulazione del robot con controllo PID. Molte funzioni sono state già modificate (analitycal_IK_3DoF utile per il calcolo dei primi angoli di giunto; percorso, circonferenza1, circonferenza2: traiettoria desiderata; inv_man_rid già modificata per cinematica diretta)
Necessario calcolare traietoria ONLINE
Selezionare i parametri PID da PID_controller
Dynamic_model_3dof interamente da modificare
 
