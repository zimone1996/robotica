function output=check_links_dimensions_4DoF(p,theta,q4,a1,a2,a3,a4,lim)

output=true;
if(isempty(p)==0 && isempty(theta)==0)%%verifico se i due vettori non sono vuoti
    p_=p(1,:);
    theta_=theta(1);
    
    if(length(p)>1 && length(theta)>1)%%passo all'elemento successivo
        p=p(2:end,:);
        theta=theta(2:end);
    else 
        p=[];
        theta=[];
    end
    %inversione cinematica 4dof
    Q=analitycal_IK_4DoF(p_,theta_,a1,a2,a3,a4,q4);
    %appartengono ai limiti degli angoli di giunto?
    if (isempty(Q)==0 && Q(1)>lim(1,1) && Q(1)<lim(1,2) ...
                      && Q(2)>lim(2,1) && Q(2)<lim(2,2) ...
                      && Q(3)>lim(3,1) && Q(3)<lim(3,2)...
                      && Q(4)>lim(4,1) && Q(4)<lim(4,2))
        %output=true;
        %richiamo la stessa funzione
        output=output && check_links_dimensions_4DoF(p,theta,q4,a1,a2,a3,a4,lim); %provo a 
        %verificare tutte le posizioni estremali dell'organo terminale
    else
        output=false;
        %se la condizione non è verificata,l'output della funzione sarà false
    end
end
end